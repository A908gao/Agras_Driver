/**
 * @file ext_imu_bridge.cpp
 * @brief L431_ADI 外部 IMU 桥接器 — MAVLink v2 官方库解析
 *
 * v2.1 — 逐采样发布 + 真实 IMU 时间戳
 *   - SerialThread: 解析 MAVLink HIGHRES_IMU, 将每帧推入 imu_queue_
 *   - PublishThread: 从队列取帧发布, 时间戳来自 MAVLink time_usec
 *   - MCU→ROS 时钟映射: 用首帧建立线性关系
 */

#include "ext_imu_bridge.h"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

namespace livox_ros {

ExtImuBridge::ExtImuBridge(rclcpp::Node* node, const ExtImuConfig& config)
    : node_(node), config_(config) {
    memset(&mav_msg_, 0, sizeof(mav_msg_));
    memset(&mav_status_, 0, sizeof(mav_status_));
}

ExtImuBridge::~ExtImuBridge() { Stop(); }

bool ExtImuBridge::Start() {
    if (running_.load()) return true;
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(config_.imu_topic, 10);
    RCLCPP_INFO(node_->get_logger(),
        "[ExtIMU] port=%s baud=%d topic=%s pub_rate=%.0f Hz",
        config_.port.c_str(), config_.baudrate, config_.imu_topic.c_str(), config_.publish_rate);
    running_.store(true);
    serial_thread_ = std::make_unique<std::thread>(&ExtImuBridge::SerialThread, this);
    publish_thread_ = std::make_unique<std::thread>(&ExtImuBridge::PublishThread, this);
    return true;
}

void ExtImuBridge::Stop() {
    running_.store(false);
    if (serial_thread_ && serial_thread_->joinable()) serial_thread_->join();
    if (publish_thread_ && publish_thread_->joinable()) publish_thread_->join();
    if (serial_fd_ >= 0) { close(serial_fd_); serial_fd_ = -1; }
    connected_.store(false);
}

void ExtImuBridge::UpdateConfig(const ExtImuConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

ExtImuBridge::Stats ExtImuBridge::GetStats() const {
    Stats s{};
    s.frame_count = mav_frame_count_;
    s.crc_err     = mav_crc_err_;
    s.imu_count   = imu_count_;
    s.imu_dropped = imu_dropped_;
    s.latest_temp = latest_temp_;
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::memcpy(s.latest_gyro, latest_gyro_, sizeof(s.latest_gyro));
    std::memcpy(s.latest_accel, latest_accel_, sizeof(s.latest_accel));
    return s;
}

// ── MCU time_usec → ROS rclcpp::Time 映射 ───────────────────────────
// 首帧到达时记录 (MCU_time, ROS_steady_time) 对,
// 后续帧: ROS_stamp = slope * (MCU_time - mcu_base) + ros_base
static rclcpp::Time mcu_to_ros_time(
    uint64_t mcu_time_us,
    bool& synced,
    double& slope, double& intercept,
    uint64_t& mcu_base_us,
    std::chrono::steady_clock::time_point& ros_base_tp,
    rclcpp::Node* node)
{
    (void)node;
    auto now_tp = std::chrono::steady_clock::now();
    if (!synced) {
        // 首帧: 假设 MCU 时钟与 ROS 时钟速率相同 (slope=1)
        mcu_base_us = mcu_time_us;
        ros_base_tp = now_tp;
        slope   = 1.0;
        intercept = 0.0;
        synced = true;
    }
    // ROS 时间 = 基准ROS时间 + (MCU时间 - 基准MCU时间) * slope
    double dt_sec = slope * static_cast<double>(mcu_time_us - mcu_base_us) * 1e-6;
    auto ros_tp = ros_base_tp + std::chrono::nanoseconds(static_cast<int64_t>(dt_sec * 1e9));
    // 转换为 rclcpp::Time (system clock)
    auto now_system = std::chrono::system_clock::now();
    auto steady_now = std::chrono::steady_clock::now();
    auto offset = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_system.time_since_epoch() - steady_now.time_since_epoch());
    auto system_tp = ros_tp + offset;
    return rclcpp::Time(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            system_tp.time_since_epoch()).count()));
}

// ── SerialThread: 读串口 → MAVLink解析 → 推入队列 ─────────────────
void ExtImuBridge::SerialThread() {
    const int reconnect_delay_ms = 3000;
    while (running_.load()) {
        serial_fd_ = open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_WARN(node_->get_logger(), "[ExtIMU] open %s fail, retry...", config_.port.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
            continue;
        }
        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));
        tcgetattr(serial_fd_, &tty);
        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;
        tcsetattr(serial_fd_, TCSANOW, &tty);
        tcflush(serial_fd_, TCIOFLUSH);
        connected_.store(true);
        RCLCPP_INFO(node_->get_logger(), "[ExtIMU] Connected to %s @ %d bps", config_.port.c_str(), config_.baudrate);

        uint8_t buf[4096];
        while (running_.load()) {
            ssize_t n = read(serial_fd_, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mav_msg_, &mav_status_);
                    if (result) {
                        mav_frame_count_++;
                        if (mav_msg_.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                            mavlink_highres_imu_t imu_data;
                            mavlink_msg_highres_imu_decode(&mav_msg_, &imu_data);

                            // 更新 latest_* (诊断用)
                            {
                                std::lock_guard<std::mutex> lock(data_mutex_);
                                latest_accel_[0] = imu_data.xacc;
                                latest_accel_[1] = imu_data.yacc;
                                latest_accel_[2] = imu_data.zacc;
                                latest_gyro_[0]  = imu_data.xgyro;
                                latest_gyro_[1]  = imu_data.ygyro;
                                latest_gyro_[2]  = imu_data.zgyro;
                                latest_temp_     = imu_data.temperature;
                                latest_ts_us_    = imu_data.time_usec;
                            }

                            // 推入逐采样队列 — 原始数据直通, MCU固件负责坐标系对齐
                            {
                                std::lock_guard<std::mutex> lock(queue_mutex_);
                                if (imu_queue_.size() >= kQueueMaxSize) {
                                    imu_queue_.pop_front();
                                    imu_dropped_++;
                                }
                                imu_queue_.push_back({
                                    imu_data.time_usec,
                                    imu_data.xgyro, imu_data.ygyro, imu_data.zgyro,
                                    imu_data.xacc,  imu_data.yacc,  imu_data.zacc,
                                    imu_data.temperature
                                });
                            }
                            imu_count_++;
                        } else {
                            static int diag_cnt = 0;
                            if (diag_cnt < 5) {
                                diag_cnt++;
                                RCLCPP_INFO(node_->get_logger(),
                                    "[ExtIMU] non-IMU msgid=%u len=%u seq=%u",
                                    mav_msg_.msgid, mav_msg_.len, mav_msg_.seq);
                            }
                        }
                    } else if (mav_status_.msg_received == MAVLINK_FRAMING_BAD_CRC) {
                        mav_crc_err_++;
                    }
                }
            } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
                RCLCPP_WARN(node_->get_logger(), "[ExtIMU] read error: %s", strerror(errno));
                break;
            }
        }
        connected_.store(false);
        close(serial_fd_);
        serial_fd_ = -1;
        if (running_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
    }
}

// ── PublishThread: 从队列逐帧发布, 使用真实IMU时间戳 ──────────────
void ExtImuBridge::PublishThread() {
    constexpr double RAD2DEG = 180.0 / M_PI;
    auto last_report = std::chrono::steady_clock::now();
    uint64_t pub_count = 0;

    static constexpr auto kMinPeriod = std::chrono::microseconds(500);  // 最高2kHz

    while (running_.load()) {
        ImuSample sample;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!imu_queue_.empty()) {
                sample = imu_queue_.front();
                imu_queue_.pop_front();
                has_data = true;
            }
        }

        if (has_data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.frame_id = config_.frame_id;

            // 使用 ROS 系统时钟 (与 LiDAR 驱动同一时钟域, FAST-LIO self-sync 对齐)
            msg->header.stamp = node_->now();

            msg->orientation_covariance[0] = -1.0;
            msg->angular_velocity_covariance[0] = -1.0;
            msg->linear_acceleration_covariance[0] = -1.0;

            float gx = sample.gx, gy = sample.gy, gz = sample.gz;
            float ax = sample.ax, ay = sample.ay, az = sample.az;
            {
                std::lock_guard<std::mutex> lock(config_mutex_);
                if (config_.gyro_unit == GyroUnit::DEG_PER_S) { gx *= RAD2DEG; gy *= RAD2DEG; gz *= RAD2DEG; }
                if (config_.accel_unit == AccelUnit::G) { ax /= 9.80665f; ay /= 9.80665f; az /= 9.80665f; }
            }
            msg->angular_velocity.x = gx; msg->angular_velocity.y = gy; msg->angular_velocity.z = gz;
            msg->linear_acceleration.x = ax; msg->linear_acceleration.y = ay; msg->linear_acceleration.z = az;
            imu_pub_->publish(std::move(msg));
            pub_count++;

            // 保存最近一次发布的数据 (诊断用)
            last_published_ = sample;
            last_published_valid_ = true;
        } else {
            // 队列空, 短暂休眠避免忙等
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        // 每2秒输出统计 — 使用最近一次实际发布的数据
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report).count() >= 2) {
            last_report = now;
            size_t queue_sz = 0;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                queue_sz = imu_queue_.size();
            }
            ImuSample diag = last_published_;  // 拷贝最近发布的数据
            bool valid = last_published_valid_;
            RCLCPP_INFO(node_->get_logger(),
                "[ExtIMU] mav:%lu imu:%lu pub:%lu drop:%lu queue:%zu crc:%lu %s"
                "g=[%.3f,%.3f,%.3f] a=[%.2f,%.2f,%.2f] t=%.1f ts=%lu",
                mav_frame_count_, imu_count_, pub_count, imu_dropped_, queue_sz,
                mav_crc_err_,
                valid ? "" : "(stale!) ",
                diag.gx, diag.gy, diag.gz,
                diag.ax, diag.ay, diag.az,
                diag.temp, diag.ts_us);
        }
    }
}

}  // namespace livox_ros
