#include "alubus_sensor_node/spresense_imu_receiver.hpp"
#include <chrono>
#include <cmath>
#include <sys/ioctl.h>

SpresenseImuReceiver::SpresenseImuReceiver()
    : Node("spresense_imu_receiver"),
      serial_fd_(-1),
      serial_port_("/dev/ttyUSB0"),  // デフォルトポート
      baudrate_(921600),
      is_running_(false)
{
    // パラメータ宣言と取得
    this->declare_parameter<std::string>("serial_port", serial_port_);
    this->declare_parameter<int>("baudrate", baudrate_);
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baudrate", baudrate_);
    
    // バッファ初期化
    buffer_.reserve(PACKET_SIZE * 2);
    latest_imu_data_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
    
    // パブリッシャー作成
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
    
    // シリアルポート開く
    if (!openSerialPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_.c_str());
        // 代替ポートを試す
        serial_port_ = "/dev/ttyACM0";
        if (!openSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open alternative serial port %s", serial_port_.c_str());
            throw std::runtime_error("Cannot open serial port");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d bps", serial_port_.c_str(), baudrate_);
    
    // 読み取りスレッド開始
    is_running_ = true;
    read_thread_ = std::thread(&SpresenseImuReceiver::readSerialData, this);
    
    // 1msタイマー（1kHz）でパブリッシュ
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&SpresenseImuReceiver::publishImuData, this));
}

SpresenseImuReceiver::~SpresenseImuReceiver()
{
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    closeSerialPort();
}

bool SpresenseImuReceiver::openSerialPort()
{
    // 名前付きパイプの場合はシンプルに開く
    if (serial_port_.find("/tmp/") == 0) {
        serial_fd_ = open(serial_port_.c_str(), O_RDONLY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            return false;
        }
        return true;
    }
    
    // 通常のシリアルポートの場合
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        return false;
    }
    
    // シリアルポート設定
    struct termios options;
    tcgetattr(serial_fd_, &options);
    
    // ボーレート設定
    speed_t baud;
    switch(baudrate_) {
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
    options.c_cflag &= ~PARENB;  // パリティなし
    options.c_cflag &= ~CSTOPB;  // ストップビット1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8ビット
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
    while (read(serial_fd_, discard_buffer, sizeof(discard_buffer)) > 0) {
        bytes_discarded += sizeof(discard_buffer);
        if (bytes_discarded > 100000) break; // 無限ループ防止
    }
    
    if (bytes_discarded > 0) {
        RCLCPP_INFO(rclcpp::get_logger("spresense_imu_receiver"), 
                   "Discarded %d bytes from serial buffer", bytes_discarded);
    }
    
    return true;
}

void SpresenseImuReceiver::closeSerialPort()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void SpresenseImuReceiver::readSerialData()
{
    uint8_t read_buffer[256];
    
    while (is_running_) {
        int bytes_read = read(serial_fd_, read_buffer, sizeof(read_buffer));
        
        if (bytes_read > 0) {
            // バッファに追加
            buffer_.insert(buffer_.end(), read_buffer, read_buffer + bytes_read);
            
            // パケット探索
            while (buffer_.size() >= PACKET_SIZE) {
                // ヘッダー探索
                auto it = buffer_.begin();
                bool header_found = false;
                
                for (; it != buffer_.end() - 1; ++it) {
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
                    std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + PACKET_SIZE);
                    
                    if (validateChecksum(packet)) {
                        if (parsePacket(packet)) {
                            // パケット処理成功
                        }
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
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            // データなし、少し待つ
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

bool SpresenseImuReceiver::validateChecksum(const std::vector<uint8_t>& packet)
{
    if (packet.size() != PACKET_SIZE) {
        return false;
    }
    
    uint8_t checksum = 0;
    for (size_t i = 0; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }
    
    return checksum == packet[PACKET_SIZE - 1];
}

bool SpresenseImuReceiver::parsePacket(const std::vector<uint8_t>& packet)
{
    if (packet[0] != PACKET_HEADER_1 || packet[1] != PACKET_HEADER_2) {
        return false;
    }
    
    // float データ抽出
    ImuData data;
    // クォータニオン (q0, q1, q2, q3)
    memcpy(&data.q0, &packet[2], sizeof(float));
    memcpy(&data.q1, &packet[6], sizeof(float));
    memcpy(&data.q2, &packet[10], sizeof(float));
    memcpy(&data.q3, &packet[14], sizeof(float));
    // ジャイロ
    memcpy(&data.gx, &packet[18], sizeof(float));
    memcpy(&data.gy, &packet[22], sizeof(float));
    memcpy(&data.gz, &packet[26], sizeof(float));
    // 加速度
    memcpy(&data.ax, &packet[30], sizeof(float));
    memcpy(&data.ay, &packet[34], sizeof(float));
    memcpy(&data.az, &packet[38], sizeof(float));
    data.valid = true;
    
    // データ更新
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_imu_data_ = data;
    }
    
    return true;
}


void SpresenseImuReceiver::publishImuData()
{
    ImuData data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        data = latest_imu_data_;
    }
    
    if (!data.valid) {
        return;
    }
    
    // IMUメッセージ作成
    auto imu_msg = sensor_msgs::msg::Imu();
    
    // タイムスタンプ
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    
    // オリエンテーション（クォータニオン）
    // Madgwickフィルタから直接取得したクォータニオンを使用
    imu_msg.orientation.w = data.q0;
    imu_msg.orientation.x = data.q1;
    imu_msg.orientation.y = data.q2;
    imu_msg.orientation.z = data.q3;
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    
    // 角速度（ジャイロデータ）
    imu_msg.angular_velocity.x = data.gx;
    imu_msg.angular_velocity.y = data.gy;
    imu_msg.angular_velocity.z = data.gz;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    
    // 線形加速度
    imu_msg.linear_acceleration.x = data.ax;
    imu_msg.linear_acceleration.y = data.ay;
    imu_msg.linear_acceleration.z = data.az;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    
    // パブリッシュ
    imu_publisher_->publish(imu_msg);
}