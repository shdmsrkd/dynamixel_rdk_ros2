#ifndef DYNAMIXEL_COMMUNICATION_NODE_HPP_
#define DYNAMIXEL_COMMUNICATION_NODE_HPP_

#include "dynamixel_rdk_ros2/base_setting_node.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/motor_position_test.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/current_motor_status.hpp"



namespace dynamixel_rdk_ros2
{
class DynamixelCommunicationNode : public BaseSettingNode
{
public:
  DynamixelCommunicationNode();
  virtual ~DynamixelCommunicationNode();
  using CurrentMotorStatus = dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus;

private:
  // 외부 Publisher 초기화
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>::SharedPtr status_republisher_;

  // 외부 Subscription 초기화

  // 내부 Publisher 초기화

  // 내부 Subscription 초기화
  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>::SharedPtr S2C_sub_;

  void motor_status_callback(const dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus::SharedPtr msg);
};
}
#endif  // DYNAMIXEL_COMMUNICATION_NODE_HPP_
