/**
 * state_manager - シンプルな状態機械
 *
 * 状態: OFF / STOP / READY / RUN
 * 入力: tick, robot_config, motor_status, state_command, imu_data
 * 出力: robot_config_out, motor_commands, state_status, set_position
 */

extern "C" {
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

// 状態
enum State { OFF = 0, STOP = 1, READY = 2, RUN = 3 };
static State g_state = OFF;
static State g_base_state = OFF;  // READY中の元の状態（OFFかSTOP）

// 軸
struct Axis {
    double position;         // 現在位置 (rad)
    double velocity;         // 現在速度 (rad/s)
    double target;           // 目標位置 (rad)
    double initial_position; // 初期位置 (rad) - READY目標
    double reset_position;   // リセット位置 (rad) - エンコーダセット用
    double start_position;   // 補間開始位置 (rad)
    uint8_t fault;
};
static std::vector<Axis> g_axes;
static bool g_configured = false;
static std::string g_config_json;

// 補間
static double g_interp_time = 3.0;      // 補間時間（秒）
static double g_interp_progress = 0.0;  // 0.0〜1.0
static const double TICK_SEC = 0.003;   // 5ms

// IMU
static double g_pitch = 0;
static double g_pitch_rate = 0;

// PID (RUN状態用)
static double g_target_pitch = 0.0465;
static double g_kp = 12.0;
static double g_ki = 325.0;
static double g_kd = 0.17;
static double g_integral = 0.0;
static const double D_DEAD_ZONE = 0.1;        // D項用不感帯（rad/s）
static const double MAX_INTEGRAL = 0.21;
static const double MAX_WHEEL_SPEED = 30.0;

// カスケードPID（速度→目標角度補正）
static double g_velocity_integral = 0.0;
static const double Kp_vel = 0.0001;   // 速度誤差への応答（非常にゆっくり）
static const double Ki_vel = 0.00005;  // 定常ドリフト補正（非常にゆっくり）
static const double MAX_VEL_INTEGRAL = 0.5;  // 積分制限
static const double MAX_ANGLE_OFFSET = 0.05;  // 目標角度補正の制限（rad）約3度

// ホイール軸インデックス
static const int WHEEL_R = 2;  // wheel_r
static const int WHEEL_L = 5;  // wheel_l

// ========== 出力 ==========

void send_motor_commands(void* ctx) {
    std::vector<uint8_t> buf(1 + g_axes.size() * 9);
    buf[0] = g_axes.size();
    size_t o = 1;

    for (size_t i = 0; i < g_axes.size(); i++) {
        uint8_t mode;
        double val = 0;  // pos or vel

        bool is_wheel = (i == WHEEL_R || i == WHEEL_L);

        if (g_state == OFF) {
            mode = 0;  // servo off
        } else if (g_state == STOP) {
            mode = 1;  // stop (NaN position = hold)
        } else if (g_state == READY) {
            if (g_base_state == OFF) {
                mode = 0;  // READY中だがservo off
            } else if (is_wheel) {
                mode = 3;  // 速度制御
                val = 0;   // ホイールは速度0
            } else {
                mode = 2;  // position control
                val = g_axes[i].target;
            }
        } else {  // RUN
            if (is_wheel) {
                mode = 3;  // 速度制御
                val = g_axes[i].target;  // target = wheel_vel (rad/s)
            } else {
                mode = 2;  // position control
                val = g_axes[i].target;  // target = initial_position
            }
        }

        buf[o++] = mode;
        std::memcpy(&buf[o], &val, 8);
        o += 8;
    }

    dora_send_output(ctx, (char*)"motor_commands", 14, (char*)buf.data(), o);
}

void send_state_status(void* ctx) {
    uint8_t progress = static_cast<uint8_t>(g_interp_progress * 100);
    uint8_t buf[2] = {static_cast<uint8_t>(g_state), progress};
    dora_send_output(ctx, (char*)"state_status", 12, (char*)buf, 2);
}

// デバッグデータ送信（rerun.io可視化用）
static double g_debug_timestamp = 0.0;
void send_debug_data(void* ctx, double error, double wheel_vel, double control_output) {
    g_debug_timestamp += TICK_SEC;
    double buf[7] = {g_debug_timestamp, g_pitch, g_pitch_rate, error, g_integral, wheel_vel, control_output};
    dora_send_output(ctx, (char*)"debug_data", 10, (char*)buf, sizeof(buf));
}

void send_set_position(void* ctx) {
    std::vector<uint8_t> buf(1 + g_axes.size() * 8);
    buf[0] = g_axes.size();
    size_t o = 1;

    for (size_t i = 0; i < g_axes.size(); i++) {
        std::memcpy(&buf[o], &g_axes[i].reset_position, 8);
        o += 8;
    }

    dora_send_output(ctx, (char*)"set_position", 12, (char*)buf.data(), o);
    std::cout << "[state_manager] set_position sent" << std::endl;
}

// ========== 入力処理 ==========

void handle_robot_config(void* ctx, const char* data, size_t len) {
    g_config_json = std::string(data, len);

    // 軸数をカウント
    size_t count = 0;
    size_t pos = 0;
    while ((pos = g_config_json.find("device_id", pos)) != std::string::npos) {
        count++;
        pos++;
    }

    g_axes.resize(count);

    // 各軸のパラメータ抽出
    pos = 0;
    for (size_t i = 0; i < count; i++) {
        g_axes[i].position = 0;
        g_axes[i].velocity = 0;
        g_axes[i].target = 0;
        g_axes[i].initial_position = 0;
        g_axes[i].reset_position = 0;
        g_axes[i].start_position = 0;
        g_axes[i].fault = 0;

        // initial_position
        size_t ipos = g_config_json.find("initial_position", pos);
        if (ipos != std::string::npos && ipos < g_config_json.find("}", pos)) {
            size_t colon = g_config_json.find(":", ipos);
            if (colon != std::string::npos) {
                g_axes[i].initial_position = std::atof(g_config_json.c_str() + colon + 1);
            }
        }

        // reset_position
        size_t rpos = g_config_json.find("reset_position", pos);
        if (rpos != std::string::npos && rpos < g_config_json.find("}", pos)) {
            size_t colon = g_config_json.find(":", rpos);
            if (colon != std::string::npos) {
                g_axes[i].reset_position = std::atof(g_config_json.c_str() + colon + 1);
            }
        }

        // 次の軸へ
        pos = g_config_json.find("}", pos);
        if (pos != std::string::npos) pos++;
    }

    // interpolation_time
    size_t tpos = g_config_json.find("interpolation_time");
    if (tpos != std::string::npos) {
        size_t colon = g_config_json.find(":", tpos);
        if (colon != std::string::npos) {
            g_interp_time = std::atof(g_config_json.c_str() + colon + 1);
        }
    }

    g_configured = true;
    std::cout << "[state_manager] " << count << " axes, interp_time=" << g_interp_time << "s" << std::endl;
    for (size_t i = 0; i < count; i++) {
        std::cout << "  axis " << i
                  << " init=" << g_axes[i].initial_position
                  << " reset=" << g_axes[i].reset_position << std::endl;
    }

    dora_send_output(ctx, (char*)"robot_config_out", 16, (char*)data, len);
}

void handle_motor_status(const char* data, size_t len) {
    if (!g_configured || len < 1) return;

    uint8_t count = data[0];
    size_t o = 1;

    for (size_t i = 0; i < g_axes.size() && i < count; i++) {
        std::memcpy(&g_axes[i].position, data + o, 8); o += 8;
        std::memcpy(&g_axes[i].velocity, data + o, 8); o += 8;
        o += 8;  // torque skip
        g_axes[i].fault = data[o++];

        // fault 32-48: 実際のエラー, 96-101: 電力/電流制限（警告のみ）
        if (g_axes[i].fault >= 32 && g_axes[i].fault <= 48 && g_state == RUN) {
            std::cout << "[state_manager] Fault code=" << (int)g_axes[i].fault
                      << " on axis " << i << " -> STOP" << std::endl;
            g_state = STOP;
        }
    }
}

void handle_imu(const char* data, size_t len) {
    // imu_node format (61 bytes):
    // [0]: valid (1 byte)
    // [1-16]: q0,q1,q2,q3 (4 floats)
    // [17-28]: gx,gy,gz (3 floats) - gyro
    // [29-40]: ax,ay,az (3 floats)
    // [41-52]: roll,pitch,yaw (3 floats)
    // [53-60]: timestamp (uint64)
    if (len < 53) return;

    float pitch_f, gy_f;
    std::memcpy(&pitch_f, data + 45, 4);  // pitch at offset 45
    std::memcpy(&gy_f, data + 21, 4);     // gy (pitch rate) at offset 21
    g_pitch = pitch_f;
    g_pitch_rate = gy_f;
}

void handle_state_command(void* ctx, const char* data, size_t len) {
    if (len < 1) return;

    uint8_t cmd = data[0];
    State old = g_state;

    switch (cmd) {
        case 0:  // STOP
            if (g_state == READY || g_state == RUN) {
                g_state = STOP;
            }
            break;
        case 1:  // RUN
            if (g_state == READY && g_interp_progress >= 1.0) {
                g_state = RUN;
                // PIDリセット
                g_integral = 0.0;
                g_velocity_integral = 0.0;
                std::cout << "[state_manager] READY -> RUN" << std::endl;
            }
            break;
        case 2:  // SERVO_OFF
            g_state = OFF;
            break;
        case 3:  // SERVO_ON (-> STOP)
            if (g_state == OFF) g_state = STOP;
            break;
        case 4:  // INIT_POSITION_RESET
            send_set_position(ctx);
            return;
        case 5:  // READY
            g_base_state = g_state;  // 元の状態を保存
            g_state = READY;
            g_interp_progress = 0.0;
            // 補間開始位置を記録
            for (auto& ax : g_axes) {
                ax.start_position = ax.position;
            }
            std::cout << "[state_manager] READY start (base=" << g_base_state << ")" << std::endl;
            break;
    }

    if (g_state != old) {
        std::cout << "[state_manager] " << old << " -> " << g_state << std::endl;
        send_state_status(ctx);
    }
}

void handle_tick(void* ctx) {
    if (!g_configured) return;

    if (g_state == READY) {
        // 補間進行
        if (g_interp_progress < 1.0) {
            g_interp_progress += TICK_SEC / g_interp_time;
            if (g_interp_progress >= 1.0) {
                g_interp_progress = 1.0;
                std::cout << "[state_manager] READY complete (waiting for RUN command)" << std::endl;
                send_state_status(ctx);
            }

            // 線形補間でtarget計算
            for (auto& ax : g_axes) {
                ax.target = ax.start_position +
                            (ax.initial_position - ax.start_position) * g_interp_progress;
            }
        } else {
            // 補間完了後はinitial_positionを保持
            for (auto& ax : g_axes) {
                ax.target = ax.initial_position;
            }
        }

        // READY中もIMU角度を表示（1秒ごと）
        static int ready_imu_cnt = 0;
        if (++ready_imu_cnt >= 250) {
            std::cout << "[READY] pitch=" << g_pitch << " rate=" << g_pitch_rate << std::endl;
            ready_imu_cnt = 0;
        }
        send_debug_data(ctx, 0, 0, 0);
    } else if (g_state == RUN) {
        // === 外側ループ: 速度PID（倒立点自動調整） ===
        // ホイール速度を0に保つための目標角度補正を計算
        double wheel_velocity = (g_axes[WHEEL_R].velocity + g_axes[WHEEL_L].velocity) / 2.0;
        double velocity_error = 0.0 - wheel_velocity;  // 目標速度は0

        g_velocity_integral += velocity_error * TICK_SEC;
        if (g_velocity_integral > MAX_VEL_INTEGRAL) g_velocity_integral = MAX_VEL_INTEGRAL;
        if (g_velocity_integral < -MAX_VEL_INTEGRAL) g_velocity_integral = -MAX_VEL_INTEGRAL;

        double angle_offset = Kp_vel * velocity_error + Ki_vel * g_velocity_integral;
        if (angle_offset > MAX_ANGLE_OFFSET) angle_offset = MAX_ANGLE_OFFSET;
        if (angle_offset < -MAX_ANGLE_OFFSET) angle_offset = -MAX_ANGLE_OFFSET;

        // === 内側ループ: 角度PID ===
        // 前傾(pitch>0)で正の出力 → ホイール前進
        double effective_target = g_target_pitch + angle_offset;
        double error = g_pitch - effective_target;

        // 積分項（アンチワインドアップ）
        g_integral += error * TICK_SEC;
        if (g_integral > MAX_INTEGRAL) g_integral = MAX_INTEGRAL;
        if (g_integral < -MAX_INTEGRAL) g_integral = -MAX_INTEGRAL;

        // PID出力（D項はIMUの角速度を直接使用）
        // 傾きが増加中(pitch_rate>0)なら抑制 → 負の出力
        double d_term = (std::abs(g_pitch_rate) < D_DEAD_ZONE) ? 0.0 : -g_pitch_rate;
        double control_output = g_kp * error + g_ki * g_integral + g_kd * d_term;

        double wheel_vel = control_output;

        // 速度制限
        if (wheel_vel > MAX_WHEEL_SPEED) wheel_vel = MAX_WHEEL_SPEED;
        if (wheel_vel < -MAX_WHEEL_SPEED) wheel_vel = -MAX_WHEEL_SPEED;

        // rerun.io用デバッグデータ送信（毎tick）
        send_debug_data(ctx, error, wheel_vel, wheel_vel);

        static int debug_cnt = 0;
        if (++debug_cnt >= 100) {
            std::cout << "[RUN] pitch=" << g_pitch << " err=" << error
                      << " vel=" << wheel_vel << " offset=" << angle_offset << std::endl;
            debug_cnt = 0;
        }

        // 股・膝は初期位置保持、ホイールのみ速度制御
        for (size_t i = 0; i < g_axes.size(); i++) {
            if (i == WHEEL_R || i == WHEEL_L) {
                // ホイール: 速度を直接target（rad/s）に設定
                g_axes[i].target = wheel_vel;  // target = velocity (rad/s)
            } else {
                // 股・膝: 初期位置保持
                g_axes[i].target = g_axes[i].initial_position;
            }
        }
    } else {
        // RUN以外の状態でもdebug_data送信（デバッグ用）
        send_debug_data(ctx, 0, 0, 0);

        // IMU角度をコンソールに表示（1秒ごと）
        static int imu_cnt = 0;
        if (++imu_cnt >= 250) {
            std::cout << "[IMU] pitch=" << g_pitch << " rate=" << g_pitch_rate << std::endl;
            imu_cnt = 0;
        }
    }

    send_motor_commands(ctx);
}

// ========== main ==========

int main() {
    std::cout << "[state_manager] Starting" << std::endl;

    auto ctx = init_dora_context_from_env();
    if (!ctx) return 1;

    while (true) {
        auto ev = dora_next_event(ctx);
        if (!ev) break;

        if (read_dora_event_type(ev) == DoraEventType_Stop) {
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
                handle_robot_config(ctx, data, data_len);
            else if (input == "motor_status")
                handle_motor_status(data, data_len);
            else if (input == "imu_data")
                handle_imu(data, data_len);
            else if (input == "state_command")
                handle_state_command(ctx, data, data_len);
            else if (input == "tick")
                handle_tick(ctx);
        }

        free_dora_event(ev);
    }

    free_dora_context(ctx);
    return 0;
}
