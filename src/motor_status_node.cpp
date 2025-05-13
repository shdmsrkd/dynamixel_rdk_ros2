#include "dynamixel_rdk_ros2/motor_status_node.hpp"

namespace dynamixel_rdk_ros2
{

using namespace dynamixel_rdk_ros2;

MotorStatusNode::MotorStatusNode()
: BaseSettingNode("motor_status_node", rclcpp::NodeOptions())
{
  RCLCPP_INFO(this->get_logger(), "Initializing Motor Status Node");

  // 모터 에러 검사
  for (auto id : motor_ids_)
  {
    errorInterface(id);
  }

}

MotorStatusNode::~MotorStatusNode()
{
  // 모든 모터 토크 해제
  for (auto id : motor_ids_)
  {
    setTorque(id, false);
  }

  port_handler_->closePort();

  RCLCPP_INFO(this->get_logger(), "Motor Control Node has been shut down");
}

bool MotorStatusNode::getCurrentTemperature(uint8_t id, uint8_t &temperature)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, MXRAM::PRESENT_TEMPERATURE.first, &temperature, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get temperature for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Temperature: %d", id, temperature);
  }

  return true;
}

bool MotorStatusNode::getCurrentTorque(u_int8_t id, u_int16_t &torque)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read2ByteTxRx(port_handler_, id, MXRAM::PRESENT_CURRENT.first, &torque, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get torque for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Torque: %d", id, torque);
  }

  return true;

}

bool MotorStatusNode::getCurrentPosition(uint8_t id, uint32_t &position)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, MXRAM::PRESENT_POSITION.first, &position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get position for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Position: %d", id, position);
  }

  return true;
}

bool MotorStatusNode::getInputVoltage(uint8_t id, u_int16_t &voltage)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read2ByteTxRx(port_handler_, id, MXRAM::PRESENT_INPUT_VOLTAGE.first, &voltage, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get input voltage for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Input Voltage: %d", id, voltage);
  }

  return true;
}

bool MotorStatusNode::HardwareErrorStatus(uint8_t id, uint8_t &error_status)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, MXRAM::HARDWARE_ERROR_STATUS.first, &error_status, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get hardware error status for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Hardware Error Status: %d", id, error_status);
  }

  return true;
}

bool MotorStatusNode::getMovingStatus(uint8_t id, uint8_t &moving_status)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, MXRAM::MOVING_STATUS.first, &moving_status, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get moving status for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Moving Status: %d", id, moving_status);
  }

  return true;

}

bool MotorStatusNode::getGoalPosition(uint8_t id, uint32_t &goal_position)
{
  dxl_error = 0;
  dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, MXRAM::GOAL_POSITION.first, &goal_position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(),
                "Failed to get goal position for ID %d: %s",
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ID %d Goal Position: %d", id, goal_position);
  }

  return true;

}

bool MotorStatusNode::errorInterface(uint8_t id)
{
  u_int8_t temperature = 0;
  u_int16_t torque = 0;
  u_int32_t position = 0;
  u_int16_t voltage = 0;
  u_int8_t error_status = 0;
  u_int8_t moving_status = 0;
  u_int32_t goal_position = 0;

  HardwareErrorStatus(id, error_status);

  if (error_status & 0b00000001)  // 비트 0: 입력 전압 오류
  {
    RCLCPP_ERROR(this->get_logger(), "ID %d: Input voltage error detected!", id);
    getInputVoltage(id, voltage);
  }

  if (error_status & 0b00000100)  // 비트 2: 과열 오류
  {
    RCLCPP_ERROR(this->get_logger(), "ID %d: Overheating error detected!", id);
    getCurrentTemperature(id, temperature);
  }

  if (error_status & 0b00001000)  // 비트 3: 엔코더 오류
  {
    RCLCPP_ERROR(this->get_logger(), "ID %d: Encoder error detected!", id);
    RCLCPP_ERROR(this->get_logger(), "ID %d: Please check the cable connection", id);
    getCurrentPosition(id, position);
    getGoalPosition(id, goal_position);
    getMovingStatus(id, moving_status);
  }

  if (error_status & 0b00010000)  // 비트 4: 전기적 충격 오류
  {
    RCLCPP_ERROR(this->get_logger(), "ID %d: Electrical shock error detected!", id);
  }

  if (error_status & 0b00100000)  // 비트 5: 과부하 오류
  {
    RCLCPP_ERROR(this->get_logger(), "ID %d: Overload error detected!", id);
    getCurrentTorque(id, torque);
  }
  return true;
}

}  // namespace dynamixel_rdk_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::MotorStatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
