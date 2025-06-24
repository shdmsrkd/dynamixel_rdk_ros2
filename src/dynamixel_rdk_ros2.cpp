#include "dynamixel_rdk_ros2/dynamixel_rdk_ros2.hpp"

namespace dynamixel_rdk_ros2
{
  dynamixel_rdk_ros2::dynamixel_rdk_ros2() : Node("dynamixel_rdk_ros2")
  {
    // 외부 Publisher 초기화
    motor_status_pubisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>("motor_status", 10);
    warning_status_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::WarningStatus>("motor_warning", 10);

    // 외부 Subscription 초기화

    // 파라미터 초기화
    initParameters();

    // Dynamixel 초기화
    initDynamixel();

    // 모터 기본값 설정
    int8_t exarray[256] = {
        0,
    };
    DefaultSettingChange(-1, exarray);

    dxl_current_ratio = MX_CURRENT_PROFILE;
    dxl_rps_ratio = MX_RPS_PROFILE * M_PI / 30;
    dxl_acc_ratio = MX_ACC_PROFILE * M_PI / 1800;

    // 타이머
    getting_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&dynamixel_rdk_ros2::timer_callback, this));

    // ik2rdk_sub = create_subscription<dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs>(
    //     "dynamixel_control", 10,
    //     std::bind(&dynamixel_rdk_ros2::dynamixel_control_callback, this, std::placeholders::_1));
  }

  dynamixel_rdk_ros2::~dynamixel_rdk_ros2()
  {
    if (port_handler_)
    {
      port_handler_->closePort();
      delete port_handler_;
    }
    if (packet_handler_)
    {
      delete packet_handler_;
    }
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_rdk_ros2"), "Dynamixel RDK ROS2 Node Shutdown");
  }

  void dynamixel_rdk_ros2::initParameters()
  {
    // 파라미터 선언
    this->declare_parameter("device_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("protocol_version", 2.0);
    this->declare_parameter("ids", std::vector<int64_t>{-100, -100, -100, -100, -100, -100, -100, -100, -100, -100});
    // this->declare_parameter("dynamixels.max_position_limits", std::vector<double>{M_PI});
    // this->declare_parameter("dynamixels.min_position_limits", std::vector<double>{-M_PI});
    // this->declare_parameter("dynamixels.max_velocity_limits", std::vector<int64_t>{1});
    // this->declare_parameter("dynamixels.temperature_limits", std::vector<int64_t>{-1});

    // 파라미터 가져오기
    device_port_ = this->get_parameter("device_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    protocol_version_ = this->get_parameter("protocol_version").as_double();
    std::vector<int64_t> ids = this->get_parameter("ids").as_integer_array();
    // max_position_limits_ = get_parameter("dynamixels.max_position_limits").as_double_array();
    // min_position_limits_ = get_parameter("dynamixels.min_position_limits").as_double_array();
    // std::vector<int64_t> max_velocity_limits = get_parameter("dynamixels.max_velocity_limits").as_integer_array();
    // std::vector<int64_t> temperature_limits = get_parameter("dynamixels.temperature_limits").as_integer_array();

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

  bool dynamixel_rdk_ros2::initDynamixel()
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

    motorCheck();

    return true;
  }

  bool dynamixel_rdk_ros2::pingMotor(int id)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->ping(port_handler_, id, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool dynamixel_rdk_ros2::motorCheck()
  {
    dynamixel_sdk_custom_interfaces::msg::WarningStatus msg;

    connected_motor_ids_.clear();
    disconnected_motor_ids_.clear();

    if (motor_ids_.empty())
    {
      return false;
    }

    for (const auto &id : motor_ids_)
    {
      msg.ids.push_back(id);
    }

    for (const auto &id : motor_ids_)
    {
      if (pingMotor(id))
      {
        connected_motor_ids_.push_back(id);
        msg.connected_motor_ids.push_back(id);
      }
      else
      {
        disconnected_motor_ids_.push_back(id);
        msg.disconnected_motor_ids.push_back(id);
      }
    }

    if (connected_motor_ids_.size() == motor_ids_.size())
    {
      RCLCPP_INFO(this->get_logger(), "All motors are connected successfully.");
    }

    else
    {
      for (const auto &id : connected_motor_ids_)
      {
        RCLCPP_INFO(this->get_logger(), "Connected motor ID: %d", id);
      }

      for (const auto &id : disconnected_motor_ids_)
      {
        RCLCPP_WARN(this->get_logger(), "Disconnected motor ID: %d", id);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Total motors: %zu", msg.ids.size());

    warning_status_publisher_->publish(msg);
    return true;
  }

  bool dynamixel_rdk_ros2::setTorque(uint8_t id, bool enable)
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
  bool dynamixel_rdk_ros2::setOperatingMode(uint8_t id, uint8_t mode)
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
  bool dynamixel_rdk_ros2::setupMotor(uint8_t id, uint8_t change_set_mode, uint8_t change_set_value)
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

  // -------------------------------------------------------------- Motor Status Getter --------------------------------------------------------------

  bool dynamixel_rdk_ros2::getCurrentPosition(uint8_t id, uint32_t &position)
  {
    return TxRx(id, MXRAM::PRESENT_POSITION, position, "Current Position");
  }

  bool dynamixel_rdk_ros2::getGoalPosition(uint8_t id, uint32_t &goal_position)
  {
    return TxRx(id, MXRAM::GOAL_POSITION, goal_position, "Goal Position");
  }

  bool dynamixel_rdk_ros2::getCurrentVelocity(uint8_t id, uint8_t &velocity)
  {
    return TxRx(id, MXRAM::PRESENT_VELOCITY, velocity, "Current Velocity");
  }

  bool dynamixel_rdk_ros2::getInputVoltage(uint8_t id, uint16_t &voltage)
  {
    return TxRx(id, MXRAM::PRESENT_INPUT_VOLTAGE, voltage, "Input Voltage");
  }

  bool dynamixel_rdk_ros2::getCurrentTemperature(uint8_t id, uint8_t &temperature)
  {
    return TxRx(id, MXRAM::PRESENT_TEMPERATURE, temperature, "Current Temperature");
  }

  bool dynamixel_rdk_ros2::getCurrentTorque(uint8_t id, uint16_t &torque)
  {
    return TxRx(id, MXRAM::PRESENT_CURRENT, torque, "Current Torque");
  }

  bool dynamixel_rdk_ros2::getMovingStatus(uint8_t id, uint8_t &moving_status)
  {
    return TxRx(id, MXRAM::MOVING_STATUS, moving_status, "Moving Status");
  }

  bool dynamixel_rdk_ros2::HardwareErrorStatus(uint8_t id, uint8_t &error_status)
  {
    return TxRx(id, MXRAM::HARDWARE_ERROR_STATUS, error_status, "Hardware Error Status");
  }

  // -------------------------------------------------------------- Motor Status Getter (Sync) --------------------------------------------------------------

  bool dynamixel_rdk_ros2::getCurrentPositionSync(uint32_t &position)
  {
    return SyncRead(MXRAM::PRESENT_POSITION, position, "Current Position");
  }

  bool dynamixel_rdk_ros2::getGoalPositionSync(uint32_t &goal_position)
  {
    return SyncRead(MXRAM::GOAL_POSITION, goal_position, "Goal Position");
  }

  bool dynamixel_rdk_ros2::getCurrentVelocitySync(uint8_t &velocity)
  {
    return SyncRead(MXRAM::PRESENT_VELOCITY, velocity, "Current Velocity");
  }

  bool dynamixel_rdk_ros2::getInputVoltageSync(uint16_t &voltage)
  {
    return SyncRead(MXRAM::PRESENT_INPUT_VOLTAGE, voltage, "Input Voltage");
  }

  bool dynamixel_rdk_ros2::getCurrentTemperatureSync(uint8_t &temperature)
  {
    return SyncRead(MXRAM::PRESENT_TEMPERATURE, temperature, "Current Temperature");
  }

  bool dynamixel_rdk_ros2::getCurrentTorqueSync(uint16_t &torque)
  {
    return SyncRead(MXRAM::PRESENT_CURRENT, torque, "Current Torque");
  }

  bool dynamixel_rdk_ros2::getMovingStatusSync(uint8_t &moving_status)
  {
    return SyncRead(MXRAM::MOVING_STATUS, moving_status, "Moving Status");
  }

  bool dynamixel_rdk_ros2::HardwareErrorStatusSync(uint8_t &error_status)
  {
    return SyncRead(MXRAM::HARDWARE_ERROR_STATUS, error_status, "Hardware Error Status");
  }

  // -------------------------------------------------------------- Motor Status Setter --------------------------------------------------------------

  bool dynamixel_rdk_ros2::setMinPositionLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::MIN_POSITION_LIMIT.first,
            4,
            &(motors_setting[i + j].min_position_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting min position limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setMaxPositionLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::MAX_POSITION_LIMIT.first,
            4,
            &(motors_setting[i + j].max_position_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting max position limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setMaxVelocityLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::VELOCITY_LIMIT.first,
            4,
            &(motors_setting[i + j].max_velocity_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting max velocity limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setMaxAccelerationLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::ACCELERATION_LIMIT.first,
            4,
            &(motors_setting[i + j].max_acceleration_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting max acceleration limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setCurrentLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::CURRENT_LIMIT.first,
            2,
            &(motors_setting[i + j].current_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting current limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setTemperatureLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::TEMPERATURE_LIMIT.first,
            1,
            &(motors_setting[i + j].temperature_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting temperature limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setPwmLimit()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::PWM_LIMIT.first,
            2,
            &(motors_setting[i + j].pwm_limit));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting PWM limit: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setShutdown()
  {
    int TOTAL_MOTOR = motor_ids_.size();

    // 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(
            motor_ids_[i + j],
            EEPROM::SHUTDOWN.first,
            1,
            &(motors_setting[i + j].shutdown));

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to bulk write for ID %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Bulk write failed when setting shutdown: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  // double dynamixel_rdk_ros2::convertPositionToRadian(int position, int min, int max)
  // {
  //   if (position < min || position > max)
  //   {
  //     return 0.0;
  //   }

  //   // 0~1 비율로 정규화한 뒤 -π ~ +π로 변환
  //   double ratio = static_cast<double>(position - min) / (max - min);
  //   return (2.0 * M_PI * ratio) - M_PI;
  // }

  // change_set_mode : 0 - Operating Mode, 1 - Min Position Limit, 2 - Max Position Limit, 3 - Max Velocity Limit, 4 - Acceleration Limit, 5 - Temperature Limit, 6 - Current Limit, 7 - PWM Limit, 8 - Shutdown
  bool dynamixel_rdk_ros2::DefaultSettingChange(uint8_t change_set_mode, int8_t change_set_value_arr[])
  {
    // change_set_mode : 설정할 모드
    // change_set_value : 설정 값
    // change_set_mode가 -1일 경우 모두 기본값으로 설정

    for (auto id : motor_ids_)
    {
      // 토크 해제
      if (!setTorque(id, false))
      {
        return false;
      }
    }

    switch (change_set_mode)
    {
    case MIN_POSITION_LIMIT_CASE:
      // 최소 위치 한계 설정 (기본값: 0)
      for (auto id : motor_ids_)
      {
        motors_setting[id].min_position_limit = change_set_value_arr[id];
      }
      setMinPositionLimit();
      break;

    case MAX_POSITION_LIMIT_CASE:
      // 최대 위치 한계 설정 (기본값 : 4095)
      for (auto id : motor_ids_)
      {
        motors_setting[id].max_position_limit = change_set_value_arr[id];
      }
      setMaxPositionLimit();
      break;

    case VELOCITY_LIMIT_CASE:
      // 최대 속도 설정 (기본값 : 210)
      for (auto id : motor_ids_)
      {
        motors_setting[id].max_velocity_limit = change_set_value_arr[id];
      }
      setMaxVelocityLimit();
      break;

    case ACCELERATION_LIMIT_CASE:
      // 최대 가속도 설정 (기본값 : 50)
      for (auto id : motor_ids_)
      {
        motors_setting[id].max_acceleration_limit = change_set_value_arr[id];
      }
      setMaxAccelerationLimit();
      break;

    case TEMPERATURE_LIMIT_CASE:
      // 온도 제한 설정 (기본값: 80)
      for (auto id : motor_ids_)
      {
        motors_setting[id].temperature_limit = change_set_value_arr[id];
      }
      setTemperatureLimit();
      break;

    case CURRENT_LIMIT_CASE:
      // 전류 제한 설정 (기본값: 2047)
      for (auto id : motor_ids_)
      {
        motors_setting[id].current_limit = change_set_value_arr[id];
      }
      setCurrentLimit();
      break;

    case PWM_LIMIT_CASE:
      // PWM 제한 설정 (기본값: 885)
      for (auto id : motor_ids_)
      {
        motors_setting[id].pwm_limit = change_set_value_arr[id];
      }
      setPwmLimit();
      break;

    case SHUTDOWN_CASE:
      // 셧다운 설정 (기본값: 52)
      for (auto id : motor_ids_)
      {
        motors_setting[id].shutdown = change_set_value_arr[id];
      }
      setShutdown();
      break;

    default:
      break;
    }

    // 토크 설정
    for (auto id : motor_ids_)
    {
      if (!setTorque(id, true))
      {
        return false;
      }
    }

    return true;
  }

  // -------------------------------------------------------------- Motor Control --------------------------------------------------------------

  bool dynamixel_rdk_ros2::setGoalPosition(uint8_t id, uint32_t position)
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

  bool dynamixel_rdk_ros2::setGoalPositionBulk()
  {
    int TOTAL_MOTOR = motor_ids_.size();
    dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

    // 다이나믹셀 모터를 5개씩 묶어서 처리
    for (int i = 0; i < TOTAL_MOTOR; i += 5)
    {
      for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = bulkWrite.addParam(motor_ids_[i + j], MXRAM::GOAL_POSITION.first, 4, &(motors[i + j].position));
        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to sync write on Dynamixel ID: %d",
                       motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = bulkWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to sync write goal position: %s",
                     packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::switchTocurrentMode(uint8_t id)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packet_handler_->write1ByteTxRx(
        port_handler_,
        id,
        EEPROM::OPERATING_MODE.first,
        0, // 0 = Current Control Mode
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to switch to current mode for ID %d: %s",
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    return true;
  }

  bool dynamixel_rdk_ros2::setGoalCurrent(uint8_t id, uint16_t current)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packet_handler_->write2ByteTxRx(
        port_handler_,
        id,
        MXRAM::GOAL_CURRENT.first,
        current,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set goal current for ID %d: %s",
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    return true;
  }

  // -------------------------------------------------------------- Timer Callback --------------------------------------------------------------
  // getting and publishing motor status
  void dynamixel_rdk_ros2::timer_callback()
  {
    motorCheck(); // 연결 상태 갱신 및 메세지 관리 (WarningStatus)

    dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus msg;

    size_t TOTAL_CONNECT_MOTORS = connected_motor_ids_.size();
    uint32_t position = 0;
    uint32_t goal_position = 0;
    uint8_t velocity = 0;
    uint16_t voltage = 0;
    uint8_t temperature = 0;
    uint16_t torque = 0;
    uint8_t moving_status = 0;
    uint8_t error_status = 0;

    msg.position.resize(TOTAL_CONNECT_MOTORS);
    msg.velocity.resize(TOTAL_CONNECT_MOTORS);
    msg.temperature.resize(TOTAL_CONNECT_MOTORS);
    msg.torque.resize(TOTAL_CONNECT_MOTORS);
    msg.input_voltage.resize(TOTAL_CONNECT_MOTORS);
    msg.moving_status.resize(TOTAL_CONNECT_MOTORS);
    msg.error_status.resize(TOTAL_CONNECT_MOTORS);

    // 현재 모터 상태를 sync로 동시에 가져오지 못할 경우, 개별 모터에 대해 순차적으로 모터 상태를 가져옴
    if (!(getCurrentPositionSync(position) &&
          getGoalPositionSync(goal_position) &&
          getCurrentVelocitySync(velocity) &&
          getInputVoltageSync(voltage) &&
          getCurrentTemperatureSync(temperature) &&
          getCurrentTorqueSync(torque) &&
          getMovingStatusSync(moving_status) &&
          HardwareErrorStatusSync(error_status)))
    {
      // 연결된 모터만 상태 받아오기
      for (size_t i = 0; i < TOTAL_CONNECT_MOTORS; ++i)
      {
        uint8_t connect_id = connected_motor_ids_[i];

        RCLCPP_INFO(this->get_logger(), "\n\n현재 처리 중인 ID: %d", connect_id);

        getCurrentPosition(connect_id, position);
        getGoalPosition(connect_id, goal_position);
        getCurrentVelocity(connect_id, velocity);
        getInputVoltage(connect_id, voltage);
        getCurrentTemperature(connect_id, temperature);
        getCurrentTorque(connect_id, torque);
        getMovingStatus(connect_id, moving_status);
        HardwareErrorStatus(connect_id, error_status);

        msg.position[i] = position;
        msg.velocity[i] = velocity;
        msg.temperature[i] = temperature;
        msg.torque[i] = torque;
        msg.input_voltage[i] = voltage;
        msg.moving_status[i] = moving_status;
        msg.error_status[i] = error_status;
      }
    }
    RCLCPP_INFO(this->get_logger(), "모터 총 개수 : %zu", motor_ids_.size());
    RCLCPP_INFO(this->get_logger(), "모터 상태 퍼블리시");
    RCLCPP_INFO(this->get_logger(), "총 연결된 모터 개수 : %ld", TOTAL_CONNECT_MOTORS);

    // 퍼블리시
    motor_status_pubisher_->publish(msg);
  }

  // -------------------------------------------------------------- Utility --------------------------------------------------------------

  bool dynamixel_rdk_ros2::errorInterface(uint8_t id)
  {
    uint8_t temperature = 0;
    uint16_t torque = 0;
    uint32_t position = 0;
    uint16_t voltage = 0;
    uint8_t error_status = 0;
    uint8_t moving_status = 0;
    uint32_t goal_position = 0;

    HardwareErrorStatus(id, error_status);

    if (error_status & 0b00000001) // 비트 0: 입력 전압 오류
    {
      RCLCPP_ERROR(this->get_logger(), "ID %d: Input voltage error detected!", id);
    }

    if (error_status & 0b00000100) // 비트 2: 과열 오류
    {
      RCLCPP_ERROR(this->get_logger(), "ID %d: Overheating error detected!", id);
    }

    if (error_status & 0b00001000) // 비트 3: 엔코더 오류
    {
      RCLCPP_ERROR(this->get_logger(), "ID %d: Encoder error detected!", id);
      RCLCPP_ERROR(this->get_logger(), "ID %d: Please check the cable connection", id);
    }

    if (error_status & 0b00010000) // 비트 4: 전기적 충격 오류
    {
      RCLCPP_ERROR(this->get_logger(), "ID %d: Electrical shock error detected!", id);
    }

    if (error_status & 0b00100000) // 비트 5: 과부하 오류
    {
      RCLCPP_ERROR(this->get_logger(), "ID %d: Overload error detected!", id);
    }
    return true;
  }

  template <typename T>
  bool dynamixel_rdk_ros2::TxRx(uint8_t id, const std::pair<int, int> &control_table_adress, T &value, const std::string &status_name)
  { // mode: 0 - Read, 1 - Write

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // // Read - 모터에서 읽은 raw 바이트 데이터를 채워넣을 포인터 주소를 전달해야 하므로 reinterpret_cast
    // if (mode == 0)
    // {
    if constexpr (sizeof(T) == 1)
    {
      dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, control_table_adress.first, reinterpret_cast<uint8_t *>(&value), &dxl_error);
    }
    else if constexpr (sizeof(T) == 2)
    {
      dxl_comm_result = packet_handler_->read2ByteTxRx(port_handler_, id, control_table_adress.first, reinterpret_cast<uint16_t *>(&value), &dxl_error);
    }
    else if constexpr (sizeof(T) == 4)
    {
      dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, control_table_adress.first, reinterpret_cast<uint32_t *>(&value), &dxl_error);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unsupported data size for %s", status_name.c_str());
      return false;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                    "Failed to get %s for ID %d: %s",
                    status_name.c_str(), id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    if (dxl_error != 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                    "Dynamixel error while reading %s for ID %d: %s",
                    status_name.c_str(), id, packet_handler_->getRxPacketError(dxl_error));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "ID %d %s: %d", id, status_name.c_str(), static_cast<int>(value));
    return true;
    // }

    // // value의 실제 값을 보내야 하므로 static_cast
    // if (mode == 1)
    // {
    //   if constexpr (sizeof(T) == 1)
    //   {
    //     dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint8_t>(value), &dxl_error);
    //   }
    //   else if constexpr (sizeof(T) == 2)
    //   {
    //     dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint16_t>(value), &dxl_error);
    //   }
    //   else if constexpr (sizeof(T) == 4)
    //   {
    //     dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint32_t>(value), &dxl_error);
    //   }
    //   else
    //   {
    //     RCLCPP_ERROR(this->get_logger(), "Unsupported data size for %s", status_name.c_str());
    //     return false;
    //   }

    //   if (dxl_comm_result != COMM_SUCCESS)
    //   {
    //     RCLCPP_ERROR(this->get_logger(),
    //                  "Failed to write %s for ID %d: %s",
    //                  status_name.c_str(), id, packet_handler_->getTxRxResult(dxl_comm_result));
    //     return false;
    //   }

    //   if (dxl_error != 0)
    //   {
    //     RCLCPP_ERROR(this->get_logger(),
    //                  "Dynamixel error while writing %s for ID %d: %s",
    //                  status_name.c_str(), id, packet_handler_->getRxPacketError(dxl_error));
    //     return false;
    //   }

    //   return true;
    // }
  }

  template <typename D>
  bool dynamixel_rdk_ros2::SyncRead(const std::pair<int, int> &control_table_adress, D &status_save, const std::string &status_name, int batch_size)
  {
    int TOTAL_MOTOR = motor_ids_.size();
    dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus msg;

    for (int i = 0; i < TOTAL_MOTOR; i += batch_size)
    {
      dynamixel::GroupSyncRead syncRead(port_handler_, packet_handler_, control_table_adress.first, control_table_adress.second);

      for (int j = 0; j < batch_size && i + j < TOTAL_MOTOR; ++j)
      {
        bool dxl_addparam_result = syncRead.addParam(motor_ids_[i + j]);

        if (!dxl_addparam_result)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to add parameter to sync read for %s, ID %d",
                       status_name.c_str(), motor_ids_[i + j]);
          return false;
        }
      }

      int dxl_comm_result = syncRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Sync read failed when getting %s",
                     status_name.c_str());
        return false;
      }

      for (int j = 0; j < batch_size && i + j < TOTAL_MOTOR; ++j)
      {
        if (syncRead.isAvailable(motor_ids_[i + j], control_table_adress.first, control_table_adress.second))
        {
          status_save = syncRead.getData(motor_ids_[i + j], control_table_adress.first, control_table_adress.second);

          if (status_name == "Current Position")
          {
            msg.position[i + j] = status_save;
          }
          else if (status_name == "Current Velocity")
          {
            msg.velocity[i + j] = status_save;
          }
          else if (status_name == "Input Voltage")
          {
            msg.input_voltage[i + j] = status_save;
          }
          else if (status_name == "Current Temperature")
          {
            msg.temperature[i + j] = status_save;
          }
          else if (status_name == "Current Torque")
          {
            msg.torque[i + j] = status_save;
          }
          else if (status_name == "Moving Status")
          {
            msg.moving_status[i + j] = status_save;
          }
        }

        else
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to get %s for ID %d",
                       status_name.c_str(), motor_ids_[i + j]);
          return false;
        }
      }
    }

    return true;
  }

  bool dynamixel_rdk_ros2::sendMotorPacket(std::vector<double> position, std::vector<double> velocity, std::vector<double> acceleration)
  {
    dynamixel::GroupSyncWrite SyncWrite(port_handler_, packet_handler_, MXRAM::PROFILE_ACCELERATION.first, 12);

    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      uint32_t goal_position = posToRadian(position[i]);
      uint32_t goal_velocity = velToRadian(velocity[i]);
      uint32_t goal_acceleration = accToRadian(acceleration[i]);

      uint8_t data[12];
      data[0] = DXL_LOBYTE(DXL_LOWORD(goal_acceleration));
      data[1] = DXL_HIBYTE(DXL_LOWORD(goal_acceleration));
      data[2] = DXL_LOBYTE(DXL_HIWORD(goal_acceleration));
      data[3] = DXL_HIBYTE(DXL_HIWORD(goal_acceleration));

      data[4] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
      data[5] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
      data[6] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
      data[7] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));

      data[8] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      data[9] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      data[10] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      data[11] = DXL_HIBYTE(DXL_HIWORD(goal_position));

      if (!SyncWrite.addParam(motor_ids_[i], data))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to add parameter for ID %d", motor_ids_[i]);
        return false;
      }
    }

    int dxl_comm_result = SyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send motor packet: %s", packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    SyncWrite.clearParam();
    return true;
  }

  // void dynamixel_rdk_ros2::dynamixel_control_callback(const dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs &msg)
  // {
  //   std::vector<double> goal_positions;
  //   std::vector<double> goal_velocities;
  //   std::vector<double> goal_accelerations;

  //   for (auto &motor_control : msg.motor_control)
  //   {
  //     goal_positions.push_back(motor_control.goal_position);
  //     goal_velocities.push_back(motor_control.profile_velocity);
  //     goal_accelerations.push_back(motor_control.profile_acceleration);
  //   }

  //   sendMotorPacket(goal_positions, goal_velocities, goal_accelerations);
  // }

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::dynamixel_rdk_ros2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
