#ifndef MOTOR_STATUS_NODE_HPP_
#define MOTOR_STATUS_NODE_HPP_

#include "dynamixel_rdk_ros2/base_setting_node.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/current_motor_status.hpp"


namespace dynamixel_rdk_ros2
{
  class MotorStatusNode : public BaseSettingNode
  {
  public:
    MotorStatusNode();
    virtual ~MotorStatusNode();

  private:
    // 모터와 관련된 값 받아오는 함수 (Getter)
    bool getCurrentPosition(uint8_t id, u_int32_t &position);
    // bool getCurrentTemperaturesSync(const std::vector<uint8_t> &ids, std::vector<uint8_t> &temperatures);
    // bool getCurrentTorquesSync(const std::vector<uint8_t> &ids, std::vector<uint16_t> &torques);
    bool getCurrentVelocity(uint8_t id, u_int8_t &velocity);
    bool getInputVoltage(uint8_t id, u_int16_t &voltage);
    bool HardwareErrorStatus(uint8_t id, uint8_t &error_status);
    bool getMovingStatus(uint8_t id, uint8_t &moving_status);
    bool getGoalPosition(uint8_t id, uint32_t &goal_position);
    bool getCurrentTorque(u_int8_t id, u_int16_t &torque);
    bool getCurrentTemperature(uint8_t id, uint8_t &temperature);

    // 모터 기본값 설정하는 함수 (Setter)
    bool setMinPositionLimit();
    bool setMaxPositionLimit();
    bool setMaxVelocityLimit();
    bool setMaxAccelerationLimit();
    bool setTemperatureLimit();
    bool setCurrentLimit();
    bool setPwmLimit();
    bool setShutdown();

    bool DefaultSettingChange(uint8_t change_set_mode, uint8_t change_set_value_arr[]);

    struct MotorSettings
    {
      u_int8_t min_position_limit;
      u_int8_t max_position_limit;

      u_int8_t max_velocity_limit;
      u_int8_t max_acceleration_limit;

      u_int8_t temperature_limit;
      u_int8_t current_limit;
      u_int8_t pwm_limit;
      u_int8_t shutdown;
    };

    std::vector<MotorSettings> motors_setting;

    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>::SharedPtr S2C_pub_;
    rclcpp::TimerBase::SharedPtr getting_timer_;

    void timer_callback();

    // 에러 감지 인터페이스
    bool errorInterface(uint8_t id);
  };


}

#endif // MOTOR_STATUS_NODE_HPP_
