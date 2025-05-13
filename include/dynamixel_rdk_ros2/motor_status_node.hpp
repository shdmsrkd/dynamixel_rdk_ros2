#ifndef MOTOR_STATUS_NODE_HPP_
#define MOTOR_STATUS_NODE_HPP_

#include "dynamixel_rdk_ros2/base_setting_node.hpp"

namespace dynamixel_rdk_ros2
{

class MotorStatusNode : public BaseSettingNode
{
public:
  MotorStatusNode();
  virtual ~MotorStatusNode();
private:

  // 모터와 관련된 값 받아오는 함수
  bool getCurrentPosition(uint8_t id, uint32_t &position);
  bool getCurrentTemperature(uint8_t id, uint8_t &temperature);
  bool getCurrentTorque(u_int8_t id, u_int16_t &torque);
  bool getInputVoltage(uint8_t id, u_int16_t &voltage);
  bool HardwareErrorStatus(uint8_t id, uint8_t &error_status);
  bool getMovingStatus(uint8_t id, uint8_t &moving_status);
  bool getGoalPosition(uint8_t id, uint32_t &goal_position);

  // 에러 감지 인터페이스
  bool errorInterface(uint8_t id);
};

}

#endif  // MOTOR_STATUS_NODE_HPP_
