/**
 * @file ext_imu_bridge.h
 * @brief L431_ADI 外部 IMU 桥接器 — 内嵌于 Livox ROS2 驱动
 *
 * 从串口 (ttyACM0) 读取 ADIS16500/ADIS16470 IMU 二进制协议帧，
 * 解析并发布为 sensor_msgs::msg::Imu，与 Livox 点云同步输出。
 *
 * 协议: L431_ADI v2.0, 帧头 AA 55, CRC-8/DALLAS
 * 单位标志位: 可独立切换陀螺 (rad/s ↔ deg/s) 和加速度 (m/s² ↔ G)
 */

#ifndef LIVOX_EXT_IMU_BRIDGE_H
#define LIVOX_EXT_IMU_BRIDGE_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "include/ros_headers.h"

namespace livox_ros {

// ── 单位标志 ──────────────────────────────────────────────────────────
enum class GyroUnit : uint8_t  { RAD_PER_S = 0, DEG_PER_S = 1 };
enum class AccelUnit : uint8_t { M_PER_S2  = 0, G          = 1 };

// ── 配置 ──────────────────────────────────────────────────────────────
struct ExtImuConfig {
    bool      enabled       = false;
    std::string port        = "/dev/ttyACM0";
    int       baudrate      = 921600;
    GyroUnit  gyro_unit     = GyroUnit::RAD_PER_S;
    AccelUnit accel_unit    = AccelUnit::M_PER_S2;
    std::string imu_topic   = "/livox/imu";
    std::string frame_id    = "livox_frame";
    double    publish_rate  = 200.0;   // Hz, IMU 数据源 1000Hz, 降频发布
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
    };
    Stats GetStats() const;

  private:
    // ── 协议解析 (状态机) ─────────────────────────────────────────
    static constexpr uint8_t SYNC0 = 0xAA;
    static constexpr uint8_t SYNC1 = 0x55;
    static constexpr uint8_t TYPE_IMU_DATA  = 0x01;
    static constexpr uint8_t TYPE_STATUS    = 0x03;
    static constexpr uint8_t TYPE_HEARTBEAT = 0x02;
    static constexpr uint8_t TYPE_DEBUG     = 0x00;
    static constexpr uint8_t MAX_PAYLOAD    = 128;

    // CRC-8/DALLAS 查找表
    static const uint8_t kCrc8Table[256];
    static uint8_t Crc8(const uint8_t* data, uint8_t len);

    // 状态机
    enum class RxState { SYNC0, SYNC1, TYPE, LEN, PAYLOAD, CRC };
    void FeedByte(uint8_t byte);
    void Dispatch(uint8_t type, const uint8_t* payload, uint8_t len);

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

    RxState rx_state_ = RxState::SYNC0;
    uint8_t rx_buf_[MAX_PAYLOAD + 2];  // TYPE + LEN + PAYLOAD
    uint8_t rx_type_ = 0;
    uint8_t rx_len_  = 0;
    uint8_t rx_idx_  = 0;

    // 标定参数 (由 STATUS 帧更新)
    float gyro_scale_  = 2.663161e-08f;
    float accel_scale_ = 1.870470e-07f;
    float temp_scale_  = 0.1f;
    float gbias_[3]    = {0, 0, 0};   // deg/s
    float aoffs_[3]    = {0, 0, 0};   // m/s²

    // 最新 IMU 数据
    mutable std::mutex data_mutex_;
    float latest_gyro_[3]  = {0, 0, 0};   // rad/s (内部存储)
    float latest_accel_[3] = {0, 0, 9.8f}; // m/s²
    float latest_temp_     = 0.0f;
    uint16_t latest_counter_ = 0;

    // 统计
    uint64_t frame_count_  = 0;
    uint64_t crc_err_      = 0;
    uint64_t imu_count_    = 0;
    uint64_t status_count_ = 0;

    // 线程
    std::unique_ptr<std::thread> serial_thread_;
    std::unique_ptr<std::thread> publish_thread_;
};

}  // namespace livox_ros

#endif  // LIVOX_EXT_IMU_BRIDGE_H
