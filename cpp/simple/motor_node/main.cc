/**
 * motor_node - シンプルなmoteusモータードライバ
 *
 * 入力: motor_commands (モード + 位置)
 * 出力: motor_status (位置 + 速度 + fault)
 */

extern "C" {
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <poll.h>
#include <sched.h>

#include "mjbots/moteus/moteus_protocol.h"
using namespace mjbots::moteus;

// 定数
static const double TWO_PI = 2.0 * M_PI;
static const int GROUP_BOUNDARY = 80;

// 軸設定
struct Axis {
    int device_id;
    int motdir;
    double velocity_limit;  // rad/s
    double accel_limit;     // rad/s^2
    double torque_limit;    // Nm
};

// グローバル
static std::vector<Axis> g_axes;
static int g_can_socket = -1;
static bool g_configured = false;

// ========== CAN ==========

bool can_init(const char* interface) {
    g_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_can_socket < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface, IFNAMSIZ);
    if (ioctl(g_can_socket, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int enable = 1;
    setsockopt(g_can_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable));

    if (bind(g_can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) return false;

    return true;
}

void can_send(uint32_t arb_id, const uint8_t* data, size_t len) {
    struct canfd_frame frame;
    frame.can_id = arb_id | CAN_EFF_FLAG;
    frame.len = len;
    frame.flags = CANFD_BRS;
    std::memcpy(frame.data, data, len);
    write(g_can_socket, &frame, sizeof(frame));
}

bool can_recv(int device_id, uint8_t* data, size_t* len, int timeout_ms) {
    struct pollfd pfd = {g_can_socket, POLLIN, 0};
    if (poll(&pfd, 1, timeout_ms) <= 0) return false;

    struct canfd_frame frame;
    ssize_t n = read(g_can_socket, &frame, sizeof(frame));
    if (n <= 0) return false;

    uint32_t expected = (device_id << 8) | CAN_EFF_FLAG;
    if (frame.can_id != expected) return false;

    *len = frame.len;
    std::memcpy(data, frame.data, frame.len);
    return true;
}

// ========== moteus フレーム構築 ==========

size_t build_stop_frame(uint8_t* buf, int device_id) {
    CanData frame;
    WriteCanData writer(&frame);

    // NaN位置 = 現在位置保持
    PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 0.0;
    cmd.maximum_torque = std::numeric_limits<double>::quiet_NaN();
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = std::numeric_limits<double>::quiet_NaN();
    cmd.accel_limit = std::numeric_limits<double>::quiet_NaN();

    PositionMode::Format fmt;
    PositionMode::Make(&writer, cmd, fmt);

    Query::Format qfmt;
    qfmt.fault = Resolution::kInt8;
    Query::Make(&writer, qfmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

size_t build_position_frame(uint8_t* buf, int device_id, double pos_rev, double vel_rev,
                            double vel_limit_rev, double accel_limit_rev, double torque_limit) {
    CanData frame;
    WriteCanData writer(&frame);

    PositionMode::Command cmd;
    cmd.position = pos_rev;
    cmd.velocity = vel_rev;
    cmd.maximum_torque = torque_limit;
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = vel_limit_rev;
    cmd.accel_limit = accel_limit_rev;

    PositionMode::Format fmt;
    PositionMode::Make(&writer, cmd, fmt);

    Query::Format qfmt;
    qfmt.fault = Resolution::kInt8;
    Query::Make(&writer, qfmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

size_t build_off_frame(uint8_t* buf, int device_id) {
    CanData frame;
    WriteCanData writer(&frame);

    StopMode::Command cmd;
    StopMode::Format fmt;
    StopMode::Make(&writer, cmd, fmt);

    Query::Format qfmt;
    qfmt.fault = Resolution::kInt8;
    Query::Make(&writer, qfmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

size_t build_velocity_frame(uint8_t* buf, int device_id, double vel_rev,
                            double vel_limit_rev, double accel_limit_rev, double torque_limit) {
    CanData frame;
    WriteCanData writer(&frame);

    PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();  // 位置はNaN
    cmd.velocity = vel_rev;
    cmd.maximum_torque = torque_limit;
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = 0.0;   // 位置制御無効
    cmd.kd_scale = 20.0;  // 速度追従ゲイン
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = vel_limit_rev;
    cmd.accel_limit = accel_limit_rev;

    PositionMode::Format fmt;
    fmt.kp_scale = Resolution::kFloat;
    fmt.kd_scale = Resolution::kFloat;
    PositionMode::Make(&writer, cmd, fmt);

    Query::Format qfmt;
    qfmt.fault = Resolution::kInt8;
    Query::Make(&writer, qfmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

size_t build_set_position_frame(uint8_t* buf, int device_id, double pos_rev) {
    CanData frame;
    WriteCanData writer(&frame);

    // OutputExact: エンコーダ位置をセット
    OutputExact::Command cmd;
    cmd.position = pos_rev;
    OutputExact::Format fmt;
    OutputExact::Make(&writer, cmd, fmt);

    std::memcpy(buf, frame.data, frame.size);
    return frame.size;
}

// ========== レスポンス解析 ==========

struct MotorStatus {
    double position;  // rev
    double velocity;  // rev/s
    double torque;
    uint8_t mode;
    uint8_t fault;
};

bool parse_response(const uint8_t* data, size_t len, MotorStatus& st) {
    st.mode = 0; st.fault = 0;
    st.position = 0; st.velocity = 0; st.torque = 0;

    // moteus_protocolのパーサーを使用
    auto result = Query::Parse(data, len);
    st.mode = static_cast<uint8_t>(result.mode);
    st.position = result.position;
    st.velocity = result.velocity;
    st.torque = result.torque;
    st.fault = result.fault;

    return true;
}

// ========== メイン処理 ==========

void handle_config(const char* data, size_t len) {
    std::string json(data, len);
    g_axes.clear();

    size_t pos = 0;
    while ((pos = json.find("device_id", pos)) != std::string::npos) {
        Axis ax = {0, 1, 100.0, 20.0, 15.0};  // defaults

        size_t next_brace = json.find("}", pos);

        size_t dpos = json.find(":", pos);
        if (dpos != std::string::npos) ax.device_id = std::atoi(json.c_str() + dpos + 1);

        size_t motdir_pos = json.find("motdir", pos);
        if (motdir_pos != std::string::npos && motdir_pos < next_brace) {
            motdir_pos = json.find(":", motdir_pos);
            ax.motdir = std::atoi(json.c_str() + motdir_pos + 1);
        }

        size_t vlim_pos = json.find("velocity_limit", pos);
        if (vlim_pos != std::string::npos && vlim_pos < next_brace) {
            vlim_pos = json.find(":", vlim_pos);
            ax.velocity_limit = std::atof(json.c_str() + vlim_pos + 1);
        }

        size_t alim_pos = json.find("accel_limit", pos);
        if (alim_pos != std::string::npos && alim_pos < next_brace) {
            alim_pos = json.find(":", alim_pos);
            ax.accel_limit = std::atof(json.c_str() + alim_pos + 1);
        }

        size_t tlim_pos = json.find("torque_limit", pos);
        if (tlim_pos != std::string::npos && tlim_pos < next_brace) {
            tlim_pos = json.find(":", tlim_pos);
            ax.torque_limit = std::atof(json.c_str() + tlim_pos + 1);
        }

        g_axes.push_back(ax);
        pos = next_brace != std::string::npos ? next_brace + 1 : pos + 1;
    }

    g_configured = true;
    std::cout << "[motor_node] " << g_axes.size() << " axes:" << std::endl;
    for (size_t i = 0; i < g_axes.size(); i++) {
        std::cout << "  [" << i << "] id=" << g_axes[i].device_id
                  << " vlim=" << g_axes[i].velocity_limit
                  << " alim=" << g_axes[i].accel_limit
                  << " tlim=" << g_axes[i].torque_limit << std::endl;
    }
}

void handle_commands(void* ctx, const char* data, size_t len) {
    if (!g_configured || len < 1) return;

    uint8_t count = data[0];
    size_t offset = 1;

    struct Cmd { uint8_t mode; double pos; };
    std::vector<Cmd> cmds(g_axes.size());

    for (size_t i = 0; i < g_axes.size() && i < count; i++) {
        cmds[i].mode = data[offset++];
        std::memcpy(&cmds[i].pos, data + offset, 8);
        offset += 8;
    }

    // グループ分け
    std::vector<size_t> grp1, grp2;
    for (size_t i = 0; i < g_axes.size(); i++) {
        if (g_axes[i].device_id < GROUP_BOUNDARY)
            grp1.push_back(i);
        else
            grp2.push_back(i);
    }

    std::vector<MotorStatus> status(g_axes.size());

    // グループ1: 1軸ずつ送信→受信
    for (size_t idx : grp1) {
        uint8_t frame[64];
        size_t flen;
        double val = cmds[idx].pos;  // pos or vel depending on mode
        double val_rev = (val / TWO_PI) * g_axes[idx].motdir;
        double vlim_rev = g_axes[idx].velocity_limit / TWO_PI;
        double alim_rev = g_axes[idx].accel_limit / TWO_PI;
        uint32_t arb_id = 0x8000 | g_axes[idx].device_id;

        if (cmds[idx].mode == 0)
            flen = build_off_frame(frame, g_axes[idx].device_id);
        else if (cmds[idx].mode == 1)
            flen = build_stop_frame(frame, g_axes[idx].device_id);
        else if (cmds[idx].mode == 3)  // 速度制御
            flen = build_velocity_frame(frame, g_axes[idx].device_id, val_rev, vlim_rev, alim_rev, g_axes[idx].torque_limit);
        else  // mode == 2: 位置制御
            flen = build_position_frame(frame, g_axes[idx].device_id, val_rev, 0, vlim_rev, alim_rev, g_axes[idx].torque_limit);

        can_send(arb_id, frame, flen);

        // すぐに受信
        uint8_t rx[64];
        size_t rxlen;
        if (can_recv(g_axes[idx].device_id, rx, &rxlen, 3)) {
            parse_response(rx, rxlen, status[idx]);
            status[idx].position *= g_axes[idx].motdir;
            status[idx].velocity *= g_axes[idx].motdir;
        }
    }

    // グループ2: 1軸ずつ送信→受信
    for (size_t idx : grp2) {
        uint8_t frame[64];
        size_t flen;
        double val = cmds[idx].pos;
        double val_rev = (val / TWO_PI) * g_axes[idx].motdir;
        double vlim_rev = g_axes[idx].velocity_limit / TWO_PI;
        double alim_rev = g_axes[idx].accel_limit / TWO_PI;
        uint32_t arb_id = 0x8000 | g_axes[idx].device_id;

        if (cmds[idx].mode == 0)
            flen = build_off_frame(frame, g_axes[idx].device_id);
        else if (cmds[idx].mode == 1)
            flen = build_stop_frame(frame, g_axes[idx].device_id);
        else if (cmds[idx].mode == 3)  // 速度制御
            flen = build_velocity_frame(frame, g_axes[idx].device_id, val_rev, vlim_rev, alim_rev, g_axes[idx].torque_limit);
        else  // mode == 2: 位置制御
            flen = build_position_frame(frame, g_axes[idx].device_id, val_rev, 0, vlim_rev, alim_rev, g_axes[idx].torque_limit);

        can_send(arb_id, frame, flen);

        uint8_t rx[64];
        size_t rxlen;
        if (can_recv(g_axes[idx].device_id, rx, &rxlen, 3)) {
            parse_response(rx, rxlen, status[idx]);
            status[idx].position *= g_axes[idx].motdir;
            status[idx].velocity *= g_axes[idx].motdir;
        }
    }

    // motor_status送信: [count][per axis: pos(8) + vel(8) + torque(8) + fault(1)]
    std::vector<uint8_t> out(1 + g_axes.size() * 25);
    out[0] = g_axes.size();
    size_t o = 1;
    for (size_t i = 0; i < g_axes.size(); i++) {
        double pos_rad = status[i].position * TWO_PI;
        double vel_rad = status[i].velocity * TWO_PI;
        std::memcpy(&out[o], &pos_rad, 8); o += 8;
        std::memcpy(&out[o], &vel_rad, 8); o += 8;
        std::memcpy(&out[o], &status[i].torque, 8); o += 8;
        out[o++] = status[i].fault;
    }

    dora_send_output(ctx, (char*)"motor_status", 12, (char*)out.data(), o);
}

// set_position: エンコーダ位置リセット
// フォーマット: [count][per axis: position(8)]
void handle_set_position(const char* data, size_t len) {
    if (!g_configured || len < 1) return;

    uint8_t count = data[0];
    size_t offset = 1;

    std::cout << "[motor_node] set_position for " << (int)count << " axes" << std::endl;

    for (size_t i = 0; i < g_axes.size() && i < count; i++) {
        if (offset + 8 > len) break;

        double pos_rad;
        std::memcpy(&pos_rad, data + offset, 8);
        offset += 8;

        double pos_rev = (pos_rad / TWO_PI) * g_axes[i].motdir;

        uint8_t frame[64];
        size_t flen = build_set_position_frame(frame, g_axes[i].device_id, pos_rev);
        uint32_t arb_id = 0x8000 | g_axes[i].device_id;
        can_send(arb_id, frame, flen);

        std::cout << "  axis " << i << ": " << pos_rad << " rad" << std::endl;
    }
}

int main() {
    std::cout << "[motor_node] Starting" << std::endl;

    if (!can_init("can0")) {
        std::cerr << "[motor_node] CAN init failed" << std::endl;
        return 1;
    }

    struct sched_param p = {80};
    sched_setscheduler(0, SCHED_FIFO, &p);

    auto ctx = init_dora_context_from_env();
    if (!ctx) return 1;

    while (true) {
        auto ev = dora_next_event(ctx);
        if (!ev) break;

        if (read_dora_event_type(ev) == DoraEventType_Stop) {
            for (auto& ax : g_axes) {
                uint8_t f[64];
                size_t flen = build_off_frame(f, ax.device_id);
                can_send(0x8000 | ax.device_id, f, flen);
            }
            free_dora_event(ev);
            break;
        }

        if (read_dora_event_type(ev) == DoraEventType_Input) {
            char *id, *data;
            size_t id_len, data_len;
            read_dora_input_id(ev, &id, &id_len);
            read_dora_input_data(ev, &data, &data_len);

            std::string input(id, id_len);
            if (input == "robot_config")
                handle_config(data, data_len);
            else if (input == "motor_commands")
                handle_commands(ctx, data, data_len);
            else if (input == "set_position")
                handle_set_position(data, data_len);
        }

        free_dora_event(ev);
    }

    close(g_can_socket);
    free_dora_context(ctx);
    return 0;
}
