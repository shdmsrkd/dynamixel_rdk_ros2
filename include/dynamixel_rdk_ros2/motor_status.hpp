#ifndef MOTOR_STATUS_HPP_
#define MOTOR_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_rdk_ros2/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#define READ 0
#define WRITE 1

namespace dynamixel_rdk_ros2
{
  class MotorStatus
  {
  public:

    struct MotorStatusConfig
    {
      uint32_t position, goal_position;
      uint8_t velocity, temperature, moving_status, error_status;
      uint16_t voltage, torque;
    };

    MotorStatus(dynamixel::PortHandler* port_handler,
                 dynamixel::PacketHandler* packet_handler,
                 rclcpp::Logger logger);
    ~MotorStatus();

    // 모터 상태 읽기 함수들 (Individual)
    bool getCurrentPosition(uint8_t id, uint32_t &position);
    bool getGoalPosition(uint8_t id, uint32_t &goal_position);
    bool getCurrentVelocity(uint8_t id, uint8_t &velocity);
    bool getInputVoltage(uint8_t id, uint16_t &voltage);
    bool getCurrentTemperature(uint8_t id, uint8_t &temperature);
    bool getCurrentTorque(uint8_t id, uint16_t &torque);
    bool getMovingStatus(uint8_t id, uint8_t &moving_status);
    bool HardwareErrorStatus(uint8_t id, uint8_t &error_status);

    // 모터 상태 읽기 함수들 (Sync)
    bool getCurrentPositionSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getGoalPositionSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getCurrentVelocitySync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getInputVoltageSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getCurrentTemperatureSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getCurrentTorqueSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool getMovingStatusSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);
    bool HardwareErrorStatusSync(const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values);

  private:
    std::vector<MotorStatus::MotorStatusConfig> motor_status_;
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;
    rclcpp::Logger logger_;
    uint8_t dxl_error_;
    int dxl_comm_result_;

    void dxl_variable_init();
    template <typename T>
    bool TxRx(uint8_t id, const std::pair<int, int> &control_table_address, T &value, const std::string &status_name, int mode);
    bool SyncRead(const std::pair<int, int> &control_table_address, const std::vector<uint8_t>& motor_ids, std::vector<MotorStatusConfig> &status_values, const std::string &status_name);
  };
}

#endif // MOTOR_STATUS_HPP_
