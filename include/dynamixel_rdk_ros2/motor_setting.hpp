#ifndef MOTOR_SETTING_HPP_
#define MOTOR_SETTING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_rdk_ros2/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_rdk_ros2
{
  class MotorSetting
  {
  public:

    struct MotorSettingConfig
    {
      uint32_t min_position_limit;
      uint32_t max_position_limit;
      uint32_t max_velocity_limit;
      uint32_t max_acceleration_limit;
      uint8_t temperature_limit;
      uint16_t current_limit;
      uint16_t pwm_limit;
      uint8_t shutdown;
    };

    MotorSetting(dynamixel::PortHandler* port_handler,
                 dynamixel::PacketHandler* packet_handler,
                 rclcpp::Logger logger);
    ~MotorSetting();

    // 모터 설정 함수들 (Individual)
    bool setTorque(uint8_t id, bool enable);
    bool setOperatingMode(uint8_t id, uint8_t mode);
    bool setGoalPosition(uint8_t id, uint32_t position);
    bool setGoalCurrent(uint8_t id, uint16_t current);

    // 모터 설정 함수들 (Sync)
    bool setTorqueSync(const std::vector<uint8_t>& motor_ids, bool enable);

    // 모터 제한값 설정 함수들 (Individual)
    bool setMinPositionLimit(uint8_t id, uint32_t limit);
    bool setMaxPositionLimit(uint8_t id, uint32_t limit);
    bool setMaxVelocityLimit(uint8_t id, uint32_t limit);
    bool setMaxAccelerationLimit(uint8_t id, uint32_t limit);
    bool setTemperatureLimit(uint8_t id, uint8_t limit);
    bool setCurrentLimit(uint8_t id, uint16_t limit);
    bool setPwmLimit(uint8_t id, uint16_t limit);
    bool setShutdown(uint8_t id, uint8_t shutdown);

    // 모터 설정 함수들 (Bulk)
    bool setMinPositionLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setMaxPositionLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setMaxVelocityLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setMaxAccelerationLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setTemperatureLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setCurrentLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setPwmLimitBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool setShutdownBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);

    // 모터 제어 패킷 전송
    bool sendMotorPacket(const std::vector<uint8_t>& motor_ids, const std::vector<double>& position,
                        const std::vector<double>& velocity, const std::vector<double>& acceleration);
    bool setGoalPositionBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings);
    bool switchToCurrentMode(uint8_t id, uint8_t mode = 0); // CURRENT_CONTROL_MODE = 0

    template <typename C>
    bool DefaultSettingChange(uint8_t change_set_mode, const std::vector<C> &change_set_value_vec,
                                         const std::vector<uint8_t> &motor_ids, std::vector<MotorSettingConfig> &motor_settings);
  private:
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;
    rclcpp::Logger logger_;
    uint8_t dxl_error_;
    int dxl_comm_result_;

    void dxl_variable_init();
    template <typename T>
    bool TxRx(uint8_t id, const std::pair<int, int> &control_table_address, T &value, const std::string &status_name, int mode);
    bool BulkWrite(const std::pair<int, int> &control_table_address, const std::vector<uint8_t>& motor_ids,
                   const std::vector<MotorSettingConfig>& settings, const std::string &status_name);
    void divide_byte(std::vector<uint8_t>& data, int address, int byte_size);
    uint32_t radianToTick(double rad)
    { return static_cast<uint32_t>((rad + M_PI) * (4095.0 / (2 * M_PI))); }
  };
}

#endif // MOTOR_SETTING_HPP_
