extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cstdint>
#include <cmath>

#include "spresense_imu.hpp"

// IMUデータメッセージのサイズ
// [valid(1)][q0-q3(16)][gx-gz(12)][ax-az(12)][roll,pitch,yaw(12)][timestamp(8)] = 61 bytes
static constexpr size_t IMU_MESSAGE_SIZE = 61;

// クォータニオン → オイラー角変換 (Roll, Pitch, Yaw)
// q0=w, q1=x, q2=y, q3=z (Madgwickフィルタ出力順)
// IMU取り付け補正: Z軸周りに+90°回転
static void quaternionToEuler(float w, float x, float y, float z,
                               float& roll, float& pitch, float& yaw)
{
    // まずIMU座標系でのオイラー角を計算
    // Roll (X軸回転)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float imu_roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y軸回転)
    float sinp = 2.0f * (w * y - z * x);
    float imu_pitch;
    if (std::abs(sinp) >= 1.0f)
        imu_pitch = std::copysign(M_PI / 2.0f, sinp);  // ジンバルロック時
    else
        imu_pitch = std::asin(sinp);

    // Yaw (Z軸回転)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float imu_yaw = std::atan2(siny_cosp, cosy_cosp);

    // Z軸周りに+90°回転の座標変換
    // IMUのX軸 → ロボットのY軸、IMUのY軸 → ロボットの-X軸
    roll  = imu_pitch;
    pitch = -imu_roll;
    yaw   = imu_yaw;
}

int main()
{
    std::cout << "[imu_node] Starting..." << std::endl;

    // Dora context初期化
    void* dora_context = init_dora_context_from_env();
    if (dora_context == nullptr)
    {
        std::cerr << "[imu_node] Failed to init dora context" << std::endl;
        return 1;
    }

    // IMU初期化
    SpresenseImu imu("/dev/ttyUSB0", 921600);

    if (!imu.open())
    {
        std::cerr << "[imu_node] Failed to open serial port, continuing without IMU" << std::endl;
        // IMUなしでも継続（valid=falseで送信）
    }
    else
    {
        std::cout << "[imu_node] IMU initialized successfully" << std::endl;
    }

    std::cout << "[imu_node] Waiting for tick..." << std::endl;

    // イベントループ
    while (true)
    {
        void* event = dora_next_event(dora_context);
        if (event == nullptr)
        {
            std::cout << "[imu_node] Event is null, exiting" << std::endl;
            break;
        }

        DoraEventType event_type = read_dora_event_type(event);

        if (event_type == DoraEventType_Stop)
        {
            std::cout << "[imu_node] Received stop event" << std::endl;
            free_dora_event(event);
            break;
        }
        else if (event_type == DoraEventType_Input)
        {
            // 入力ID取得
            char* id_ptr = nullptr;
            size_t id_len = 0;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string input_id(id_ptr, id_len);

            if (input_id == "tick")
            {
                // 最新のIMUデータを取得
                ImuData data = imu.getLatestData();

                // クォータニオン → オイラー角変換
                float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
                if (data.valid)
                {
                    quaternionToEuler(data.q0, data.q1, data.q2, data.q3, roll, pitch, yaw);
                }

                // シリアライズ
                uint8_t buffer[IMU_MESSAGE_SIZE];
                size_t offset = 0;

                buffer[offset++] = data.valid ? 1 : 0;
                std::memcpy(buffer + offset, &data.q0, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.q1, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.q2, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.q3, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.gx, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.gy, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.gz, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.ax, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.ay, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.az, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &roll, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &pitch, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &yaw, sizeof(float)); offset += sizeof(float);
                std::memcpy(buffer + offset, &data.timestamp_ns, sizeof(uint64_t)); offset += sizeof(uint64_t);

                // 送信
                int result = dora_send_output(
                    dora_context,
                    const_cast<char*>("imu_data"), 8,
                    reinterpret_cast<char*>(buffer), IMU_MESSAGE_SIZE
                );

                if (result != 0)
                {
                    std::cerr << "[imu_node] Failed to send imu_data" << std::endl;
                }
            }
        }

        free_dora_event(event);
    }

    // クリーンアップ
    imu.close();
    free_dora_context(dora_context);

    std::cout << "[imu_node] Finished" << std::endl;
    return 0;
}
