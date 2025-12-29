#include "spresense_imu.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <chrono>
#include <iostream>

SpresenseImu::SpresenseImu(const std::string& port, int baudrate)
    : port_(port),
      baudrate_(baudrate),
      serial_fd_(-1),
      running_(false)
{
    buffer_.reserve(PACKET_SIZE * 2);
    latest_data_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0};
}

SpresenseImu::~SpresenseImu()
{
    close();
}

bool SpresenseImu::open()
{
    if (serial_fd_ >= 0) {
        return true;  // 既に開いている
    }

    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "[SpresenseImu] Failed to open " << port_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    // シリアルポート設定
    struct termios options;
    tcgetattr(serial_fd_, &options);

    // ボーレート設定
    speed_t baud;
    switch (baudrate_) {
        case 115200:
            baud = B115200;
            break;
        case 921600:
            baud = B921600;
            break;
        default:
            baud = B921600;
            break;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1設定
    options.c_cflag &= ~PARENB;   // パリティなし
    options.c_cflag &= ~CSTOPB;   // ストップビット1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8ビット
    options.c_cflag |= CREAD | CLOCAL;  // 受信有効、モデム制御無視

    // Raw入力
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    // タイムアウト設定
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;  // 0.1秒タイムアウト

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);

    // DTR/RTSを明示的に設定
    int status;
    ioctl(serial_fd_, TIOCMGET, &status);
    status |= TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_fd_, TIOCMSET, &status);

    // 既存のバッファを全てクリア（起動時の古いデータを破棄）
    uint8_t discard_buffer[4096];
    int bytes_discarded = 0;
    while (::read(serial_fd_, discard_buffer, sizeof(discard_buffer)) > 0) {
        bytes_discarded += sizeof(discard_buffer);
        if (bytes_discarded > 100000) break;  // 無限ループ防止
    }

    if (bytes_discarded > 0) {
        std::cout << "[SpresenseImu] Discarded " << bytes_discarded << " bytes from serial buffer" << std::endl;
    }

    std::cout << "[SpresenseImu] Opened " << port_ << " at " << baudrate_ << " bps" << std::endl;

    // 読み取りスレッド開始
    running_ = true;
    read_thread_ = std::thread(&SpresenseImu::readThread, this);

    return true;
}

void SpresenseImu::close()
{
    running_ = false;

    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        std::cout << "[SpresenseImu] Closed serial port" << std::endl;
    }
}

ImuData SpresenseImu::getLatestData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}

uint64_t SpresenseImu::getNowNs()
{
    using namespace std::chrono;
    return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

void SpresenseImu::readThread()
{
    uint8_t read_buffer[256];

    while (running_) {
        int bytes_read = ::read(serial_fd_, read_buffer, sizeof(read_buffer));

        if (bytes_read > 0) {
            // バッファに追加
            buffer_.insert(buffer_.end(), read_buffer, read_buffer + bytes_read);

            // パケット探索
            while (buffer_.size() >= PACKET_SIZE) {
                // ヘッダー探索
                auto it = buffer_.begin();
                bool header_found = false;

                for (; it < buffer_.end() - 1; ++it) {
                    if (*it == PACKET_HEADER_1 && *(it + 1) == PACKET_HEADER_2) {
                        header_found = true;
                        break;
                    }
                }

                if (!header_found) {
                    // ヘッダーが見つからない場合、バッファクリア
                    buffer_.clear();
                    break;
                }

                // ヘッダーより前のデータを削除
                if (it != buffer_.begin()) {
                    buffer_.erase(buffer_.begin(), it);
                }

                // 完全なパケットがあるか確認
                if (buffer_.size() >= PACKET_SIZE) {
                    if (validateChecksum(buffer_.data(), PACKET_SIZE)) {
                        parsePacket(buffer_.data(), PACKET_SIZE);
                    }

                    // 処理済みパケットを削除
                    buffer_.erase(buffer_.begin(), buffer_.begin() + PACKET_SIZE);
                } else {
                    break;
                }
            }

            // バッファサイズ制限
            if (buffer_.size() > PACKET_SIZE * 10) {
                buffer_.clear();
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            std::cerr << "[SpresenseImu] Serial read error: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            // データなし、少し待つ
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

bool SpresenseImu::validateChecksum(const uint8_t* packet, size_t len)
{
    if (len != PACKET_SIZE) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }

    return checksum == packet[PACKET_SIZE - 1];
}

bool SpresenseImu::parsePacket(const uint8_t* packet, size_t len)
{
    if (len < PACKET_SIZE) {
        return false;
    }

    if (packet[0] != PACKET_HEADER_1 || packet[1] != PACKET_HEADER_2) {
        return false;
    }

    ImuData data;

    // クォータニオン (q0, q1, q2, q3)
    std::memcpy(&data.q0, &packet[2], sizeof(float));
    std::memcpy(&data.q1, &packet[6], sizeof(float));
    std::memcpy(&data.q2, &packet[10], sizeof(float));
    std::memcpy(&data.q3, &packet[14], sizeof(float));

    // ジャイロ
    std::memcpy(&data.gx, &packet[18], sizeof(float));
    std::memcpy(&data.gy, &packet[22], sizeof(float));
    std::memcpy(&data.gz, &packet[26], sizeof(float));

    // 加速度
    std::memcpy(&data.ax, &packet[30], sizeof(float));
    std::memcpy(&data.ay, &packet[34], sizeof(float));
    std::memcpy(&data.az, &packet[38], sizeof(float));

    data.valid = true;
    data.timestamp_ns = getNowNs();

    // データ更新
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = data;
    }

    return true;
}
