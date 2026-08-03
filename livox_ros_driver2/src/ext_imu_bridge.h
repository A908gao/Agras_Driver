/**
 * @file ext_imu_bridge.h
 * @brief L431_ADI 外部 IMU 桥接器 — MAVLink v2 解析 (官方库)
 *
 * 从串口 (/dev/ttyIMU) 接收 MAVLink v2 HIGHRES_IMU 数据，
 * 解析并发布为 sensor_msgs::msg::Imu。
 *
 * CRC/帧解析/零尾修剪: 全部委托 MAVLink 官方库, 无手写逻辑。
 */

#ifndef LIVOX_EXT_IMU_BRIDGE_H
#define LIVOX_EXT_IMU_BRIDGE_H

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "include/ros_headers.h"

// MAVLink 配置 (无命名空间, C 接口)
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "mavlink/common/mavlink.h"

namespace livox_ros {

// ── 单位标志 ──────────────────────────────────────────────────────────
enum class GyroUnit : uint8_t  { RAD_PER_S = 0, DEG_PER_S = 1 };
enum class AccelUnit : uint8_t { M_PER_S2  = 0, G          = 1 };

// ── 配置 ──────────────────────────────────────────────────────────────
struct ExtImuConfig {
    bool      enabled       = false;
    std::string port        = "/dev/ttyIMU";
    int       baudrate      = 921600;
    GyroUnit  gyro_unit     = GyroUnit::RAD_PER_S;
    AccelUnit accel_unit    = AccelUnit::M_PER_S2;
    std::string imu_topic   = "/livox/imu";
    std::string frame_id    = "livox_frame";
    double    publish_rate  = 500.0;   // Hz, ★ 外置IMU 500Hz, 逐采样发布
};

/**
 * @class ExtImuBridge
 * @brief 外部串口 IMU → ROS2 sensor_msgs::Imu 桥接器
 *
 * 用法:
 *   auto bridge = std::make_unique<ExtImuBridge>(node, config);
 *   bridge->Start();   // 启动串口读取 + 发布线程
 *   // ... 点云也在发布 ...
 *   bridge->Stop();    // 停止
 */
class ExtImuBridge {
  public:
    /**
     * @param node   ROS2 节点指针 (用于创建 publisher 和日志)
     * @param config 桥接器配置
     */
    ExtImuBridge(rclcpp::Node* node, const ExtImuConfig& config);
    ~ExtImuBridge();

    ExtImuBridge(const ExtImuBridge&) = delete;
    ExtImuBridge& operator=(const ExtImuBridge&) = delete;

    /** 启动串口读取和发布线程 */
    bool Start();

    /** 停止所有线程 */
    void Stop();

    /** 运行时更新配置 (单位切换等) */
    void UpdateConfig(const ExtImuConfig& config);

    /** 查询运行状态 */
    bool IsRunning() const { return running_.load(); }
    bool IsConnected() const { return connected_.load(); }

    /** 获取统计信息 */
    struct Stats {
        uint64_t frame_count;
        uint64_t crc_err;
        uint64_t imu_count;
        uint64_t status_count;
        float    latest_gyro[3];   // rad/s
        float    latest_accel[3];  // m/s²
        float    latest_temp;      // °C
        uint16_t latest_counter;
        uint64_t imu_dropped;
        uint64_t imu_published;
        double   imu_interval_ms;  // 最近两帧IMU间隔
    };
    Stats GetStats() const;

  private:
    // ── MAVLink v2 解析 ─────────────────────────────────────────
    mavlink_message_t  mav_msg_;
    mavlink_status_t   mav_status_;

    // ── 串口读写 ──────────────────────────────────────────────────
    void SerialThread();
    int  serial_fd_ = -1;

    // ── ROS2 发布 ──────────────────────────────────────────────────
    void PublishThread();

    rclcpp::Node* node_;               // 非拥有
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // ── 配置 ──────────────────────────────────────────────────────
    ExtImuConfig config_;
    std::mutex config_mutex_;

    // ── 运行时状态 ────────────────────────────────────────────────
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    // 最新 IMU 数据 + 时间戳
    mutable std::mutex data_mutex_;
    float latest_gyro_[3]  = {0, 0, 0};   // rad/s
    float latest_accel_[3] = {0, 0, 9.8f}; // m/s²
    float latest_temp_     = 0.0f;
    uint64_t latest_ts_us_ = 0;            // MAVLink time_usec (MCU 采样时刻, us)

    // ── 逐采样队列 (SerialThread → PublishThread) ────────────────
    //   替代原来的 "latest value" 模式, 确保不丢帧、时间戳真实连续
    struct ImuSample {
        uint64_t ts_us;     // MAVLink 时间戳 (us, MCU 时钟)
        float gx, gy, gz;   // rad/s
        float ax, ay, az;   // m/s²
        float temp;         // °C
    };
    static constexpr size_t kQueueMaxSize = 64;  // ~128ms @ 500Hz
    std::deque<ImuSample> imu_queue_;
    std::mutex queue_mutex_;
    uint64_t imu_dropped_     = 0;       // 队列满时丢弃数
    ImuSample last_published_ = {0, 0,0,0, 0,0,0, 0};  // 最近一次发布的数据(诊断用)
    bool      last_published_valid_ = false;

    // ── 时钟映射: MCU time_usec → ROS steady_time ────────────────
    //   ROS_stamp = mcu_to_ros_slope_ * MCU_time_usec + mcu_to_ros_intercept_
    bool   mcu_clock_synced_ = false;
    double mcu_to_ros_slope_     = 1.0;
    double mcu_to_ros_intercept_ = 0.0;
    uint64_t mcu_base_us_   = 0;
    std::chrono::steady_clock::time_point ros_base_tp_;

    // 统计
    uint64_t mav_frame_count_ = 0;
    uint64_t mav_crc_err_     = 0;
    uint64_t imu_count_       = 0;

    // 线程
    std::unique_ptr<std::thread> serial_thread_;
    std::unique_ptr<std::thread> publish_thread_;
};

}  // namespace livox_ros

#endif  // LIVOX_EXT_IMU_BRIDGE_H
