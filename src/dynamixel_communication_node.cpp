#include "dynamixel_rdk_ros2/dynamixel_communication_node.hpp"

namespace dynamixel_rdk_ros2
{

  using namespace dynamixel_rdk_ros2;

  DynamixelCommunicationNode::DynamixelCommunicationNode()
      : BaseSettingNode("dynamixel_communication_node", rclcpp::NodeOptions())
  {
    // 외부 Publisher 초기화
    status_republisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>("motor_status_repub", 10);
    warning_status_republisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::WarningStatus>("motor_warning", 10);

    // 외부 Subscription 초기화

    // 내부 Publisher 초기화

    // 내부 Subscription 초기화
    S2C_sub_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>("S2C_topic", 10, std::bind(&DynamixelCommunicationNode::motor_status_callback, this, std::placeholders::_1));
    B2C_sub_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::WarningStatus>("B2C_topic", 10, std::bind(&DynamixelCommunicationNode::motor_warning_callback, this, std::placeholders::_1));
  }

  void DynamixelCommunicationNode::motor_status_callback(const dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus::SharedPtr msg)
  {
    status_republisher_->publish(*msg);
  }

  void DynamixelCommunicationNode::motor_warning_callback(const dynamixel_sdk_custom_interfaces::msg::WarningStatus::SharedPtr msg)
  {
    warning_status_republisher_->publish(*msg);
    // 로그 출력
    RCLCPP_INFO(this->get_logger(), "Motor warning status republished");
  }


  DynamixelCommunicationNode::~DynamixelCommunicationNode()
  {
    RCLCPP_INFO(this->get_logger(), "Dynamixel Communication Node has been shut down");
  }

} // namespace dynamixel_rdk_ros2
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::DynamixelCommunicationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// namespace dynamixel_rdk_ros2
