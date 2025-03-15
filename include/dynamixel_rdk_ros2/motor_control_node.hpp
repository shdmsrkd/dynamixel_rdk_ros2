#ifndef MOTOR_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode();
  virtual ~MotorControlNode();

private:
  // 파라미터
  std::string port_name_;
  int baud_rate_;
  float protocol_version_;
  std::vector<int> motor_ids_;

  // 모터 레지스터 주소
  uint16_t addr_torque_enable_;
  uint16_t addr_goal_position_;
  uint16_t addr_present_position_;

  // Dynamixel 핸들러
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;

  bool initDynamixel();
  void initParameters();

  bool setupMotor(uint8_t id);


};

#endif  // MOTOR_CONTROL_NODE_HPP_
