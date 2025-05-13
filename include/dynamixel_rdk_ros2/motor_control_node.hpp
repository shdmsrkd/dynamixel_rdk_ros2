#ifndef MOTOR_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_NODE_HPP_

#include "dynamixel_rdk_ros2/base_setting_node.hpp"
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>


namespace dynamixel_rdk_ros2
{

class MotorControlNode : public BaseSettingNode
{
public:
  MotorControlNode();
  virtual ~MotorControlNode();

  struct MotorKinematics
  {
    double position;
    double velocity;
    double acceleration;
  };

  std::vector<MotorKinematics> motors;

  // 위치 설정 및 읽기 함수
  bool setGoalPosition(uint8_t id, uint32_t position);
  uint32_t getPresentPosition(uint8_t id);

  bool setGoalPositionSync(u_int8_t id, uint32_t position);

};

}  // namespace dynamixel_rdk_ros2
#endif  // MOTOR_CONTROL_NODE_HPP_
