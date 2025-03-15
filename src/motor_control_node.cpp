#include "dynamixel_rdk_ros2/motor_control_node.hpp"
#include "dynamixel_rdk_ros2/control_table.hpp"

#include <utility>
#include <iostream>

namespace dynamixel_rdk_ros2
{

using namespace dynamixel_rdk_ros2;

MotorControlNode::MotorControlNode()
: Node("motor_control_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Motor Control Node");

  // 파라미터 초기화
  initParameters();

  // 모터 제어 관련 변수 초기화
  uint8_t dxl_error = 0;
  int dxl_set_result = COMM_TX_FAIL;


  // Dynamixel 초기화
  if (!initDynamixel()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel motors");
    return;
  }
}

MotorControlNode::~MotorControlNode()
{
  // 모든 모터 토크 해제
  for (auto id : motor_ids_) {
    setTorque(id, false);
  }

  // 포트 닫기
  if (port_handler_->isOpen()) {
    port_handler_->closePort();
  }

  RCLCPP_INFO(this->get_logger(), "Motor Control Node has been shut down");
}

void MotorControlNode::initParameters()
{
  // 파라미터 선언
  this->declare_parameter("device_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 57600);
  this->declare_parameter("protocol_version", 2.0);
  this->declare_parameter("motor_ids", std::vector<int64_t>{1});


  // 파라미터 가져오기
  device_port_ = this->get_parameter("device_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  protocol_version_ = this->get_parameter("protocol_version").as_double();
  std::vector<int64_t> ids = this->get_parameter("motor_ids").as_integer_array();
  for (size_t i = 0; i < ids.size(); ++i)
  {
      motor_ids_.push_back(static_cast<uint8_t>(ids[i]));
  }


  // 파라미터 출력
  RCLCPP_INFO(this->get_logger(), "Device port: %s", device_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "Protocol version: %.1f", protocol_version_);
  RCLCPP_INFO(this->get_logger(), "Number of motors: %zu", motor_ids_.size());
}

bool MotorControlNode::initDynamixel()
{
  // 포트 핸들러 및 패킷 핸들러 초기화
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // 포트 열기
  if (!port_handler_->openPort())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", device_port_.c_str());
    return false;
  }

  // 통신 속도 설정
  if (!port_handler_->setBaudRate(baud_rate_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate: %d", baud_rate_);
    return false;
  }

  // 모터 설정
  for (auto id : motor_ids_)
  {
    if (!setupMotor(id))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup motor ID: %d", id);
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Dynamixel motors initialized successfully");
  return true;
}

// 모터 초기설정 (토크 해제, 위치제어 모드, 최소/최대 위치 한계 설정, 토크 설정)
bool setupMotor(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_set_result = COMM_TX_FAIL;

  // 토크 해제
  if (!setTorque(id, false))
  {
    return false;
  }

  // 모터 작동모드 설정 - 위치제어 모드(3)
  if (!setOperatingMode(id, 3))
  {
    return false;
  }

  // // 최소 위치 한계 설정 (기본값: 0)
  // dxl_set_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::MIN_POSITION_LIMIT.first, 0, &dxl_error);

  // if (dxl_set_result != COMM_SUCCESS)
  // {
  //   RCLCPP_ERROR(this->get_logger(),
  //               "Failed to set min position limit for ID %d: %s",
  //               id, packet_handler_->getTxRxResult(dxl_set_result));
  //   return false;
  // }

  // // 최대 위치 한계 설정 (기본값 : 4095)
  // dxl_set_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::MAX_POSITION_LIMIT.first, 4095, &dxl_error);

  // if (dxl_set_result != COMM_SUCCESS)
  // {
  //   RCLCPP_ERROR(this->get_logger(),
  //               "Failed to set max position limit for ID %d: %s",
  //               id, packet_handler_->getTxRxResult(dxl_set_result));
  //   return false;
  // }

  // // 최대 속도 설정 (기본값 : 210)
  // dxl_set_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::VELOCITY_LIMIT.first, 210, &dxl_error);
  //   if (dxl_set_result != COMM_SUCCESS)
  // {
  //   RCLCPP_ERROR(this->get_logger(),
  //               "Failed to set max velocity limit for ID %d: %s",
  //               id, packet_handler_->getTxRxResult(dxl_set_result));
  //   return false;
  // }


  // 토크 설정
  if (!setTorque(id, true))
  {
    return false;
  }

  return true;

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
