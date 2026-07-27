/**
 * @file ext_imu_bridge.cpp
 * @brief L431_ADI 外部 IMU 桥接器实现
 *
 * 协议帧格式:
 *   SYNC0 SYNC1 TYPE LEN PAYLOAD[0..LEN-1] CRC8
 *   0xAA  0x55  1B   1B  LEN bytes          1B
 *
 * CRC-8/DALLAS: poly=0x31, init=0xFF, 覆盖 TYPE+LEN+PAYLOAD
 */

#include "ext_imu_bridge.h"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

namespace livox_ros {

// ═══════════════════════════════════════════════════════════════════════
// CRC-8/DALLAS 查找表 (多项式 0x31)
// ═══════════════════════════════════════════════════════════════════════

const uint8_t ExtImuBridge::kCrc8Table[256] = {
    0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E,
    0x43,0x72,0x21,0x10,0x87,0xB6,0xE5,0xD4,0xFA,0xCB,0x98,0xA9,0x3E,0x0F,0x5C,0x6D,
    0x86,0xB7,0xE4,0xD5,0x42,0x73,0x20,0x11,0x3F,0x0E,0x5D,0x6C,0xFB,0xCA,0x99,0xA8,
    0xC5,0xF4,0xA7,0x96,0x01,0x30,0x63,0x52,0x7C,0x4D,0x1E,0x2F,0xB8,0x89,0xDA,0xEB,
    0x3D,0x0C,0x5F,0x6E,0xF9,0xC8,0x9B,0xAA,0x84,0xB5,0xE6,0xD7,0x40,0x71,0x22,0x13,
    0x7E,0x4F,0x1C,0x2D,0xBA,0x8B,0xD8,0xE9,0xC7,0xF6,0xA5,0x94,0x03,0x32,0x61,0x50,
    0xBB,0x8A,0xD9,0xE8,0x7F,0x4E,0x1D,0x2C,0x02,0x33,0x60,0x51,0xC6,0xF7,0xA4,0x95,
    0xF8,0xC9,0x9A,0xAB,0x3C,0x0D,0x5E,0x6F,0x41,0x70,0x23,0x12,0x85,0xB4,0xE7,0xD6,
    0x7A,0x4B,0x18,0x29,0xBE,0x8F,0xDC,0xED,0xC3,0xF2,0xA1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5B,0x6A,0xFD,0xCC,0x9F,0xAE,0x80,0xB1,0xE2,0xD3,0x44,0x75,0x26,0x17,
    0xFC,0xCD,0x9E,0xAF,0x38,0x09,0x5A,0x6B,0x45,0x74,0x27,0x16,0x81,0xB0,0xE3,0xD2,
    0xBF,0x8E,0xDD,0xEC,0x7B,0x4A,0x19,0x28,0x06,0x37,0x64,0x55,0xC2,0xF3,0xA0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xB2,0xE1,0xD0,0xFE,0xCF,0x9C,0xAD,0x3A,0x0B,0x58,0x69,
    0x04,0x35,0x66,0x57,0xC0,0xF1,0xA2,0x93,0xBD,0x8C,0xDF,0xEE,0x79,0x48,0x1B,0x2A,
    0xC1,0xF0,0xA3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1A,0x2B,0xBC,0x8D,0xDE,0xEF,
    0x82,0xB3,0xE0,0xD1,0x46,0x77,0x24,0x15,0x3B,0x0A,0x59,0x68,0xFF,0xCE,0x9D,0xAC,
};

uint8_t ExtImuBridge::Crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    while (len--) crc = kCrc8Table[crc ^ *data++];
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════
// 辅助: 小端读取 int32 / int16 / float32, 大端读取 uint16
// ═══════════════════════════════════════════════════════════════════════

static inline int32_t ReadI32LE(const uint8_t* p) {
    return (int32_t)(p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static inline int16_t ReadI16LE(const uint8_t* p) {
    return (int16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t ReadU16BE(const uint8_t* p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline float ReadF32LE(const uint8_t* p) {
    float f; std::memcpy(&f, p, 4); return f;
}

// ═══════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════

ExtImuBridge::ExtImuBridge(rclcpp::Node* node, const ExtImuConfig& config)
    : node_(node), config_(config) {}

ExtImuBridge::~ExtImuBridge() { Stop(); }

// ═══════════════════════════════════════════════════════════════════════
// Start / Stop
// ═══════════════════════════════════════════════════════════════════════

bool ExtImuBridge::Start() {
    if (running_.load()) return true;

    // 创建 publisher
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(config_.imu_topic, 10);

    RCLCPP_INFO(node_->get_logger(),
        "[ExtIMU] Starting bridge: port=%s baud=%d topic=%s frame=%s",
        config_.port.c_str(), config_.baudrate,
        config_.imu_topic.c_str(), config_.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(),
        "[ExtIMU] Units: gyro=%s accel=%s",
        config_.gyro_unit == GyroUnit::RAD_PER_S ? "rad/s" : "deg/s",
        config_.accel_unit == AccelUnit::M_PER_S2 ? "m/s²" : "G");

    running_.store(true);

    // 启动串口读取线程
    serial_thread_ = std::make_unique<std::thread>(&ExtImuBridge::SerialThread, this);
    // 启动发布线程
    publish_thread_ = std::make_unique<std::thread>(&ExtImuBridge::PublishThread, this);

    return true;
}

void ExtImuBridge::Stop() {
    running_.store(false);
    if (serial_thread_ && serial_thread_->joinable()) {
        serial_thread_->join();
    }
    if (publish_thread_ && publish_thread_->joinable()) {
        publish_thread_->join();
    }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    connected_.store(false);
    RCLCPP_INFO(node_->get_logger(), "[ExtIMU] Bridge stopped.");
}

void ExtImuBridge::UpdateConfig(const ExtImuConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    bool unit_changed = (config_.gyro_unit != config.gyro_unit ||
                         config_.accel_unit != config.accel_unit);
    config_ = config;
    if (unit_changed) {
        RCLCPP_INFO(node_->get_logger(),
            "[ExtIMU] Units updated: gyro=%s accel=%s",
            config_.gyro_unit == GyroUnit::RAD_PER_S ? "rad/s" : "deg/s",
            config_.accel_unit == AccelUnit::M_PER_S2 ? "m/s²" : "G");
    }
}

ExtImuBridge::Stats ExtImuBridge::GetStats() const {
    Stats s{};
    s.frame_count = frame_count_;
    s.crc_err     = crc_err_;
    s.imu_count   = imu_count_;
    s.status_count = status_count_;
    s.latest_counter = latest_counter_;
    s.latest_temp    = latest_temp_;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::memcpy(s.latest_gyro,  latest_gyro_,  sizeof(s.latest_gyro));
        std::memcpy(s.latest_accel, latest_accel_, sizeof(s.latest_accel));
    }
    return s;
}

// ═══════════════════════════════════════════════════════════════════════
// 协议帧解析 — 字节级状态机
// ═══════════════════════════════════════════════════════════════════════

void ExtImuBridge::FeedByte(uint8_t b) {
    switch (rx_state_) {
    case RxState::SYNC0:
        if (b == SYNC0) rx_state_ = RxState::SYNC1;
        break;
    case RxState::SYNC1:
        if (b == SYNC1) rx_state_ = RxState::TYPE;
        else if (b != SYNC0) rx_state_ = RxState::SYNC0;
        break;
    case RxState::TYPE:
        rx_type_ = b;
        rx_buf_[0] = b;
        rx_idx_ = 1;
        rx_state_ = RxState::LEN;
        break;
    case RxState::LEN:
        if (b > MAX_PAYLOAD) { rx_state_ = RxState::SYNC0; break; }
        rx_len_ = b;
        rx_buf_[rx_idx_++] = b;
        rx_state_ = (rx_len_ == 0) ? RxState::CRC : RxState::PAYLOAD;
        break;
    case RxState::PAYLOAD:
        if (rx_idx_ - 2 < rx_len_) rx_buf_[rx_idx_++] = b;
        if (rx_idx_ - 2 >= rx_len_) rx_state_ = RxState::CRC;
        break;
    case RxState::CRC:
        frame_count_++;
        if (Crc8(rx_buf_, rx_len_ + 2) == b) {
            Dispatch(rx_type_, &rx_buf_[2], rx_len_);
        } else {
            crc_err_++;
        }
        rx_state_ = RxState::SYNC0;
        break;
    }
}

void ExtImuBridge::Dispatch(uint8_t type, const uint8_t* payload, uint8_t len) {
    constexpr float DEG2RAD = M_PI / 180.0f;

    switch (type) {
    case TYPE_IMU_DATA:  // 0x01, 28 bytes
        if (len >= 28) {
            uint16_t counter = ReadU16BE(payload);
            int32_t  raw_gx  = ReadI32LE(payload + 2);
            int32_t  raw_gy  = ReadI32LE(payload + 6);
            int32_t  raw_gz  = ReadI32LE(payload + 10);
            int32_t  raw_ax  = ReadI32LE(payload + 14);
            int32_t  raw_ay  = ReadI32LE(payload + 18);
            int32_t  raw_az  = ReadI32LE(payload + 22);
            int16_t  raw_t   = ReadI16LE(payload + 26);

            // 转换为物理量 → 内部统一用 rad/s + m/s² 存储
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_gyro_[0] = raw_gx * gyro_scale_ - gbias_[0] * DEG2RAD;
            latest_gyro_[1] = raw_gy * gyro_scale_ - gbias_[1] * DEG2RAD;
            latest_gyro_[2] = raw_gz * gyro_scale_ - gbias_[2] * DEG2RAD;
            latest_accel_[0] = raw_ax * accel_scale_ - aoffs_[0];
            latest_accel_[1] = raw_ay * accel_scale_ - aoffs_[1];
            latest_accel_[2] = raw_az * accel_scale_ - aoffs_[2];
            latest_temp_     = raw_t * temp_scale_;
            latest_counter_  = counter;
            imu_count_++;
        }
        break;

    case TYPE_STATUS:  // 0x03, 40 bytes
        if (len >= 40) {
            gyro_scale_  = ReadF32LE(payload + 3);
            accel_scale_ = ReadF32LE(payload + 7);
            temp_scale_  = ReadF32LE(payload + 11);
            gbias_[0]    = ReadF32LE(payload + 15);
            gbias_[1]    = ReadF32LE(payload + 19);
            gbias_[2]    = ReadF32LE(payload + 23);
            aoffs_[0]    = ReadF32LE(payload + 27);
            aoffs_[1]    = ReadF32LE(payload + 31);
            aoffs_[2]    = ReadF32LE(payload + 35);
            status_count_++;
        }
        break;

    case TYPE_DEBUG:  // 0x00
        if (len > 0) {
            // 限制日志长度防止刷屏
            char buf[129];
            uint8_t copy_len = (len < 128) ? len : 127;
            std::memcpy(buf, payload, copy_len);
            buf[copy_len] = '\0';
            RCLCPP_INFO(node_->get_logger(), "[ExtIMU DEBUG] %s", buf);
        }
        break;

    case TYPE_HEARTBEAT:  // 0x02
        // 心跳静默处理 (1Hz, 不打印)
        break;

    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════
// 串口读取线程
// ═══════════════════════════════════════════════════════════════════════

void ExtImuBridge::SerialThread() {
    const int reconnect_delay_ms = 3000;

    while (running_.load()) {
        // 打开串口
        serial_fd_ = open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_WARN(node_->get_logger(),
                "[ExtIMU] Cannot open %s: %s. Retrying in %ds...",
                config_.port.c_str(), strerror(errno), reconnect_delay_ms / 1000);
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
            continue;
        }

        // 配置串口
        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(node_->get_logger(), "[ExtIMU] tcgetattr failed: %s", strerror(errno));
            close(serial_fd_); serial_fd_ = -1;
            continue;
        }

        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;        // 无校验
        tty.c_cflag &= ~CSTOPB;        // 1 停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8 数据位
        tty.c_cflag &= ~CRTSCTS;       // 无流控
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 1;           // 0.1s 超时

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(node_->get_logger(), "[ExtIMU] tcsetattr failed: %s", strerror(errno));
            close(serial_fd_); serial_fd_ = -1;
            continue;
        }

        // 清空缓冲区
        tcflush(serial_fd_, TCIOFLUSH);

        connected_.store(true);
        RCLCPP_INFO(node_->get_logger(),
            "[ExtIMU] Connected to %s @ %d bps", config_.port.c_str(), config_.baudrate);

        // 读取循环
        uint8_t buf[4096];
        while (running_.load()) {
            ssize_t n = read(serial_fd_, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    FeedByte(buf[i]);
                }
            } else if (n == 0) {
                // 超时, 继续
                continue;
            } else {
                // 错误
                if (errno == EAGAIN || errno == EINTR) continue;
                RCLCPP_WARN(node_->get_logger(),
                    "[ExtIMU] Serial read error: %s", strerror(errno));
                break;  // 跳出内层循环, 重连
            }
        }

        // 断开
        connected_.store(false);
        close(serial_fd_);
        serial_fd_ = -1;

        if (running_.load()) {
            RCLCPP_INFO(node_->get_logger(),
                "[ExtIMU] Disconnected. Reconnecting in %ds...", reconnect_delay_ms / 1000);
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// 发布线程 — 按固定频率从 latest_ 读取并发布 sensor_msgs::Imu
// ═══════════════════════════════════════════════════════════════════════

void ExtImuBridge::PublishThread() {
    constexpr double G = 9.80665;
    constexpr double RAD2DEG = 180.0 / M_PI;

    auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / config_.publish_rate));
    auto last_report = std::chrono::steady_clock::now();

    while (running_.load()) {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.frame_id = config_.frame_id;
        msg->header.stamp = node_->now();

        // 协方差置 -1 (FAST_LIO 忽略)
        msg->orientation_covariance[0] = -1.0;
        msg->angular_velocity_covariance[0] = -1.0;
        msg->linear_acceleration_covariance[0] = -1.0;

        // 读取最新数据
        float gx, gy, gz, ax, ay, az;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            gx = latest_gyro_[0];
            gy = latest_gyro_[1];
            gz = latest_gyro_[2];
            ax = latest_accel_[0];
            ay = latest_accel_[1];
            az = latest_accel_[2];
        }

        // 根据单位标志位转换输出
        {
            std::lock_guard<std::mutex> lock(config_mutex_);

            if (config_.gyro_unit == GyroUnit::DEG_PER_S) {
                gx *= RAD2DEG;
                gy *= RAD2DEG;
                gz *= RAD2DEG;
            }

            if (config_.accel_unit == AccelUnit::G) {
                ax /= G;
                ay /= G;
                az /= G;
            }
        }

        msg->angular_velocity.x = gx;
        msg->angular_velocity.y = gy;
        msg->angular_velocity.z = gz;
        msg->linear_acceleration.x = ax;
        msg->linear_acceleration.y = ay;
        msg->linear_acceleration.z = az;

        imu_pub_->publish(std::move(msg));

        // 定期报告统计 (每 10 秒)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report).count() >= 10) {
            last_report = now;
            RCLCPP_INFO(node_->get_logger(),
                "[ExtIMU] frames:%lu imu:%lu status:%lu crc_err:%lu "
                "g=[%.3f,%.3f,%.3f] a=[%.2f,%.2f,%.2f] t=%.1f°C",
                frame_count_, imu_count_, status_count_, crc_err_,
                gx, gy, gz, ax, ay, az, latest_temp_);
        }

        // 频率控制
        std::this_thread::sleep_for(period);
    }
}

}  // namespace livox_ros
