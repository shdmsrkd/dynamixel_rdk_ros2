#include "dynamixel_rdk_ros2/motor_control_node.hpp"

#include <utility>
#include <iostream>

namespace dynamixel_rdk_ros2
{

using namespace dynamixel_rdk_ros2;

MotorControlNode::MotorControlNode()
: BaseSettingNode("motor_control_node", rclcpp::NodeOptions())
{
  // 노드 초기화
  RCLCPP_INFO(this->get_logger(), "Initializing Motor Control Node");

  // 파라미터 초기화
  initParameters();

  // Dynamixel 초기화
  if (!initDynamixel())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel motors");
    return;
  }

  // 모터 설정
  for (auto id : motor_ids_)
  {
    if (!setupMotor(id, -1, -1))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup motor ID: %d", id);
    }
  }

  setGoalPosition(1, 512); // 예시로 ID 1에 대해 목표 위치를 512로 설정
}

MotorControlNode::~MotorControlNode()
{
  // 모든 모터 토크 해제
  for (auto id : motor_ids_)
  {
    setTorque(id, false);
  }

  port_handler_->closePort();

  RCLCPP_INFO(this->get_logger(), "Motor Control Node has been shut down");
}

bool MotorControlNode::setGoalPosition(uint8_t id, uint32_t position)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, MXRAM::GOAL_POSITION.first, position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to set goal position for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  return true;
}

bool MotorControlNode::setGoalPositionSync(uint8_t id, uint32_t position)
{
  dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);
}

uint32_t MotorControlNode::getPresentPosition(uint8_t id)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;
  uint32_t present_position = 0;

  dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, MXRAM::PRESENT_POSITION.first, &present_position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get present position for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return 0;
  }

  return present_position;
}

} // namespace dynamixel_rdk_ros2

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

