#include "dynamixel_rdk_ros2/motor_status_node.hpp"
#define MIN_POSITION_LIMIT_CASE 0
#define MAX_POSITION_LIMIT_CASE 1
#define VELOCITY_LIMIT_CASE 2
#define ACCELERATION_LIMIT_CASE 3
#define TEMPERATURE_LIMIT_CASE 4
#define CURRENT_LIMIT_CASE 5
#define PWM_LIMIT_CASE 6
#define SHUTDOWN_CASE 7

namespace dynamixel_rdk_ros2
{

  using namespace dynamixel_rdk_ros2;

  MotorStatusNode::MotorStatusNode()
      : BaseSettingNode("motor_status_node", rclcpp::NodeOptions())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Motor Status Node");

    S2C_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>("S2C_topic", 10);
    // 벡터 크기 초기화 (ID를 인덱스로 사용하기 위해 최대 ID + 1 크기로 초기화)
    uint8_t max_id = 0;
    for (const auto &id : motor_ids_)
    {
      if (id > max_id)
        max_id = id;
    }

    // 벡터 초기화 (ID를 인덱스로 사용하기 위해 최대 ID + 1 크기로 초기화)
    motors_setting.resize(max_id + 1);

    getting_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotorStatusNode::timer_callback, this));
  }

  MotorStatusNode::~MotorStatusNode()
  {
    // 모든 모터 토크 해제
    for (auto id : connected_motor_ids_)
    {
      setTorque(id, false);
    }

    port_handler_->closePort();

    RCLCPP_INFO(this->get_logger(), "Motor Control Node has been shut down");
  }
  // -------------------------------------------------------------- Motor Status Getter --------------------------------------------------------------
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

  // bool MotorStatusNode::getCurrentTemperaturesSync(const std::vector<uint8_t> &ids, std::vector<uint8_t> &temperatures)
  // {
  //   dxl_error = 0;
  //   dxl_comm_result = COMM_TX_FAIL;

  //   dynamixel::GroupSyncRead sync_read(port_handler_, packet_handler_, MXRAM::PRESENT_TEMPERATURE.first, MXRAM::PRESENT_TEMPERATURE.second);

  //   for (uint8_t id : ids)
  //   {
  //     if (!sync_read.addParam(id))
  //     {
  //       RCLCPP_ERROR(this->get_logger(), "Failed to add param for ID %d", id);
  //       return false;
  //     }
  //   }

  //   dxl_comm_result = sync_read.txRxPacket();
  //   if (dxl_comm_result != COMM_SUCCESS)
  //   {
  //     RCLCPP_ERROR(this->get_logger(),
  //                 "SyncRead communication failed: %s",
  //                 packet_handler_->getTxRxResult(dxl_comm_result));
  //     return false;
  //   }

  //   return true;
  // }

  // bool MotorStatusNode::getCurrentTorquesSync(const std::vector<uint8_t> &ids, std::vector<uint16_t> &torques)
  // {
  //   dxl_error = 0;
  //   dxl_comm_result = COMM_TX_FAIL;

  //   dynamixel::GroupSyncRead sync_read(port_handler_, packet_handler_, MXRAM::PRESENT_CURRENT.first, MXRAM::PRESENT_CURRENT.second);

  //   for (uint8_t id : ids)
  //   {
  //     if (!sync_read.addParam(id))
  //     {
  //       RCLCPP_ERROR(this->get_logger(), "Failed to add param for ID %d", id);
  //       return false;
  //     }
  //   }

  //   dxl_comm_result = sync_read.txRxPacket();
  //   if (dxl_comm_result != COMM_SUCCESS)
  //   {
  //     RCLCPP_ERROR(this->get_logger(),
  //                 "SyncRead communication failed: %s",
  //                 packet_handler_->getTxRxResult(dxl_comm_result));
  //     return false;
  //   }

  //   return true;
  // }

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

  bool MotorStatusNode::getCurrentVelocity(uint8_t id, u_int8_t &velocity)
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, MXRAM::PRESENT_VELOCITY.first, &velocity, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get velocity for ID %d: %s",
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "ID %d Velocity: %d", id, velocity);
    }

    return true;
  }

  bool MotorStatusNode::getInputVoltage(uint8_t id, uint16_t &voltage)
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

    uint32_t threshold = 100;

    dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, EEPROM::MOVING_THRESHOLD.first, threshold, &dxl_error);

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

  // -------------------------------------------------------------- Motor Status Getter --------------------------------------------------------------

  void MotorStatusNode::timer_callback()
  {
    motorCheck(); // 연결 상태 갱신

    dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus msg;
    size_t total_motors = connected_motor_ids_.size();

    // resize 연결된 모터 수 만큼
    msg.id.resize(total_motors);
    msg.position.resize(total_motors);
    msg.velocity.resize(total_motors);
    msg.temperature.resize(total_motors);
    msg.torque.resize(total_motors);
    msg.input_voltage.resize(total_motors);
    msg.moving_status.resize(total_motors);

    for (size_t i = 0; i < total_motors; ++i)
    {
      uint8_t id = connected_motor_ids_[i];
      msg.id[i] = id;

      RCLCPP_INFO(this->get_logger(), "\n\n현재 처리 중인 ID: %d", id);

      uint8_t temperature = 0;
      uint16_t torque = 0;
      uint32_t position = 0;
      uint8_t velocity = 0;
      uint16_t voltage = 0;
      uint8_t moving_status = 0;

      // 연결된 모터에 대해서만 정보 수집
      getCurrentTemperature(id, temperature);
      getCurrentTorque(id, torque);
      getCurrentPosition(id, position);
      getCurrentVelocity(id, velocity);

      if(voltage < 60000)
      {
        getInputVoltage(id, voltage);
      }
      getMovingStatus(id, moving_status);

      msg.position[i] = position;
      msg.velocity[i] = velocity;
      msg.temperature[i] = temperature;
      msg.torque[i] = torque;
      msg.input_voltage[i] = voltage;
      msg.moving_status[i] = moving_status;

      RCLCPP_INFO(this->get_logger(), "ID: %d | Pos: %u | Vel: %u | Temp: %u | Torque: %u | Volt: %u | Moving: %u",
                  id, position, velocity, temperature, torque, voltage, moving_status);
    }

    RCLCPP_INFO(this->get_logger(), "모터 상태 퍼블리시");
    RCLCPP_INFO(this->get_logger(), "총 연결된 모터 개수: %ld", total_motors);
    S2C_pub_->publish(msg);
  }

  // -------------------------------------------------------------- Motor Status Setter --------------------------------------------------------------

  bool MotorStatusNode::setMinPositionLimit()
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

  bool MotorStatusNode::setMaxPositionLimit()
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

  bool MotorStatusNode::setMaxVelocityLimit()
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

  bool MotorStatusNode::setMaxAccelerationLimit()
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

  bool MotorStatusNode::setCurrentLimit()
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

  bool MotorStatusNode::setTemperatureLimit()
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

  bool MotorStatusNode::setPwmLimit()
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

  bool MotorStatusNode::setShutdown()
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

  bool MotorStatusNode::DefaultSettingChange(uint8_t change_set_mode, uint8_t change_set_value_arr[])
  {
    // change_set_mode : 설정할 모드
    // change_set_value : 설정 값
    // change_set_mode가 -1일 경우 모두 기본값으로 설정

    // for (auto id : motor_ids_)
    // {
    //   if (!setupMotor(id, change_set_mode, change_set_value))
    //   {
    //     return false;
    //   }
    // }

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

    return true;
  }

  // bool MotorStatusNode::errorInterface(uint8_t id)
  // {
  //   u_int8_t temperature = 0;
  //   u_int16_t torque = 0;
  //   u_int32_t position = 0;
  //   u_int16_t voltage = 0;
  //   u_int8_t error_status = 0;
  //   u_int8_t moving_status = 0;
  //   u_int32_t goal_position = 0;

  //   HardwareErrorStatus(id, error_status);

  //   if (error_status & 0b00000001)  // 비트 0: 입력 전압 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Input voltage error detected!", id);
  //     getInputVoltage(id, voltage);
  //   }

  //   if (error_status & 0b00000100)  // 비트 2: 과열 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Overheating error detected!", id);
  //     getCurrentTemperature(id, temperature);
  //   }

  //   if (error_status & 0b00001000)  // 비트 3: 엔코더 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Encoder error detected!", id);
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Please check the cable connection", id);
  //     // getCurrentPosition(id, position);
  //     getGoalPosition(id, goal_position);
  //     getMovingStatus(id, moving_status);
  //   }

  //   if (error_status & 0b00010000)  // 비트 4: 전기적 충격 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Electrical shock error detected!", id);
  //   }

  //   if (error_status & 0b00100000)  // 비트 5: 과부하 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Overload error detected!", id);
  //     getCurrentTorque(id, torque);
  //   }
  //   return true;
  // }

} // namespace dynamixel_rdk_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::MotorStatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
