#ifndef DYNAMIXEL_COMMUNICATION_NODE_HPP_
#define DYNAMIXEL_COMMUNICATION_NODE_HPP_

#include "dynamixel_rdk_ros2/base_setting_node.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"


namespace dynamixel_rdk_ros2
{
class DynamixelCommunicationNode : public BaseSettingNode
{
public:
  DynamixelCommunicationNode();
  virtual ~DynamixelCommunicationNode();

private:
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};
}
#endif  // DYNAMIXEL_COMMUNICATION_NODE_HPP_
