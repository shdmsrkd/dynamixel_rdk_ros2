#include "dynamixel_rdk_ros2/base_setting_node.hpp"

namespace dynamixel_rdk_ros2
{

  using namespace dynamixel_rdk_ros2;

  BaseSettingNode::BaseSettingNode(const std::string &node_name,const rclcpp::NodeOptions &options)
      : rclcpp::Node(node_name, options)
  {

    // 노드 초기화
    RCLCPP_INFO(this->get_logger(), "Initializing Base Setting Node");

    // 파라미터 초기화-
    initParameters();

    // Dynamixel 초기화
    if (!initDynamixel())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel motors");
    }

    // 모터 설정
    for (auto id : motor_ids_)
    {
      if (!setupMotor(id, -1, -1))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup motor ID: %d", id);
      }
    }

  }

  BaseSettingNode::~BaseSettingNode()
  {
    // 모든 모터 토크 해제
    for (auto id : motor_ids_)
    {
      setTorque(id, false);
    }

    port_handler_->closePort();

    RCLCPP_INFO(this->get_logger(), "Base Setting Node has been shut down");
  }

  void BaseSettingNode::initParameters()
  {
    // 파라미터 선언

    if (!this->has_parameter("device_port"))
    {
      this->declare_parameter("device_port", "/dev/ttyUSB0");
    }
    if(!this->has_parameter("baud_rate"))
    {
      this->declare_parameter("baud_rate", 1000000);
    }
    if(!this->has_parameter("protocol_version"))
    {
      this->declare_parameter("protocol_version", 2.0);
    }
    if(!this->has_parameter("motor_ids"))
    {
      this->declare_parameter("motor_ids", std::vector<int64_t>{1});
    }
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

  bool BaseSettingNode::initDynamixel()
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
    return true;
  }

  bool BaseSettingNode::setTorque(uint8_t id, bool enable)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, MXRAM::TORQUE_ENABLE.first, enable, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set torque for ID %d: %s",
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    return true;
  }

  // 0 - Current Control Mode, 1 - Velocity Control Mode, 3(Default) - Position Control Mode, 4 - Extended Position Control Mode (Multi-turn), 5 - Current-based Position Control Mode, 16 - PWM Control Mode
  bool BaseSettingNode::setOperatingMode(uint8_t id, uint8_t mode)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, EEPROM::OPERATING_MODE.first, mode, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set operating mode for ID %d: %s",
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    return true;
  }

  // change_set_mode : 0 - Operating Mode, 1 - Min Position Limit, 2 - Max Position Limit, 3 - Max Velocity Limit, 4 - Acceleration Limit, 5 - Temperature Limit, 6 - Current Limit, 7 - PWM Limit, 8 - Shutdown
  // change_set_value : 설정 값
  // change_set_mode가 -1일 경우 모두 기본값으로 설정
  bool BaseSettingNode::setupMotor(uint8_t id, uint8_t change_set_mode, uint8_t change_set_value)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    // 토크 해제
    if (!setTorque(id, false))
    {
      return false;
    }

    switch (change_set_mode)
    {
    case 0:
      // 모터 작동모드 설정 - 위치제어 모드(3)
      if (!setOperatingMode(id, change_set_value))
      {
        return false;
      }
      break;

    case 1:
      // 최소 위치 한계 설정 (기본값: 0)
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::MIN_POSITION_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set min position limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 2:
      // 최대 위치 한계 설정 (기본값 : 4095)
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::MAX_POSITION_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set max position limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 3:
      // 최대 속도 설정 (기본값 : 210)
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::VELOCITY_LIMIT.first, change_set_value, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set max velocity limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 4:
      // 가속도 제한 설정 (기본값: 32767)
      dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::ACCELERATION_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set acceleration limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 5:
      // 온도 제한 설정 (기본값: 80)
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, EEPROM::TEMPERATURE_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set temperature limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 6:
      // 전류 제한 설정 (기본값: 2047)
      dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, EEPROM::CURRENT_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set current limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 7:
      // PWM 제한 설정 (기본값: 885)
      dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, EEPROM::PWM_LIMIT.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set PWM limit for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    case 8:
      // 셧다운 설정 (기본값: 52)
      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, EEPROM::SHUTDOWN.first, change_set_value, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set shutdown configuration for ID %d: %s",
                     id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      break;

    default:
      break;
    }

    // 토크 설정
    if (!setTorque(id, true))
    {
      return false;
    }

    return true;
  }
}
// namespace dynamixel_rdk_ros2
