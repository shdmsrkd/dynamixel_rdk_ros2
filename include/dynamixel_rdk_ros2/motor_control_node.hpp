#ifndef MOTOR_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_rdk_ros2
{

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode();
  virtual ~MotorControlNode();

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // 위치 설정 및 읽기 함수
  bool setGoalPosition(uint8_t id, uint32_t position);
  uint32_t getPresentPosition(uint8_t id);

private:
  // 파라미터
  std::string device_port_;
  int baud_rate_;
  float protocol_version_;
  std::vector<int> motor_ids_;

  // Dynamixel 핸들러
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;

  // 모터 제어 관련 변수
  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr get_position_server_;


  // 파라미터 초기화 및 적용
  bool initDynamixel();
  void initParameters();

  // 모터 토크 활성화/비활성화 함수
  bool setTorque(uint8_t id, bool enable);

  //  모터 초기값 설정 함수
  bool setupMotor(uint8_t id, uint8_t change_set_mode, uint8_t change_set_value);

  // 모터 작동 모드 설정 함수
  bool setOperatingMode(uint8_t id, uint8_t mode);

};

}  // namespace dynamixel_rdk_ros2
#endif  // MOTOR_CONTROL_NODE_HPP_
