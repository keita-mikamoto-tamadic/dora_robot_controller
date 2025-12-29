#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cstdint>

// IMUデータ構造体
struct ImuData {
    float q0, q1, q2, q3;  // クォータニオン（Madgwickフィルタ済み）
    float gx, gy, gz;       // 角速度 (rad/s)
    float ax, ay, az;       // 加速度 (m/s²)
    bool valid;             // データ有効フラグ
    uint64_t timestamp_ns;  // 受信時刻（ナノ秒）
};

// Spresense IMU シリアル読み取りクラス
class SpresenseImu {
public:
    // パケット定数
    static constexpr uint8_t PACKET_HEADER_1 = 0xAA;
    static constexpr uint8_t PACKET_HEADER_2 = 0x55;
    static constexpr size_t PACKET_SIZE = 43;  // ヘッダ2 + float×10 + チェックサム1

    SpresenseImu(const std::string& port = "/dev/ttyUSB0", int baudrate = 921600);
    ~SpresenseImu();

    // シリアルポートを開く
    bool open();

    // シリアルポートを閉じる
    void close();

    // 最新のIMUデータを取得（スレッドセーフ）
    ImuData getLatestData();

    // シリアルポートが開いているか
    bool isOpen() const { return serial_fd_ >= 0; }

private:
    // 読み取りスレッド関数
    void readThread();

    // チェックサム検証
    bool validateChecksum(const uint8_t* packet, size_t len);

    // パケットをパース
    bool parsePacket(const uint8_t* packet, size_t len);

    // 現在時刻をナノ秒で取得
    uint64_t getNowNs();

    std::string port_;
    int baudrate_;
    int serial_fd_;

    std::thread read_thread_;
    std::atomic<bool> running_;

    std::mutex data_mutex_;
    ImuData latest_data_;

    std::vector<uint8_t> buffer_;
};
