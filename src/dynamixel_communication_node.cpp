#include "dynamixel_rdk_ros2/dynamixel_communication_node.hpp"

namespace dynamixel_rdk_ros2
{

using namespace dynamixel_rdk_ros2;

DynamixelCommunicationNode::DynamixelCommunicationNode()
: BaseSettingNode("dynamixel_communication_node", rclcpp::NodeOptions())
{
  // Publisher 초기화
  publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
  // Timer 초기화
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DynamixelCommunicationNode::timer_callback, this));
}

DynamixelCommunicationNode::~DynamixelCommunicationNode()
{
  RCLCPP_INFO(this->get_logger(), "Dynamixel Communication Node has been shut down");
}

void DynamixelCommunicationNode::timer_callback()
{
  dynamixel_sdk_custom_interfaces::msg::SetPosition msg;
  msg.id = 1;
  msg.position = 512;
  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "msg published: id=%d, position=%d", msg.id, msg.position);
}
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dynamixel_rdk_ros2::DynamixelCommunicationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// namespace dynamixel_rdk_ros2
