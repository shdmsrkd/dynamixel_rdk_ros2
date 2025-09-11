#include "dynamixel_rdk_ros2/motor_setting.hpp"

namespace dynamixel_rdk_ros2
{
  MotorSetting::MotorSetting(dynamixel::PortHandler *port_handler,
                             dynamixel::PacketHandler *packet_handler,
                             rclcpp::Logger logger)
      : port_handler_(port_handler), packet_handler_(packet_handler), logger_(logger)
  { dxl_variable_init(); }

  MotorSetting::~MotorSetting() {}

  void MotorSetting::dxl_variable_init()
  {
    dxl_error_ = 0;
    dxl_comm_result_ = COMM_TX_FAIL;
  }

  /*=================================================== MOTOR CONTROL SETTER(Individual) ===================================================*/
  bool MotorSetting::setTorque(uint8_t id, bool enable)
  { return TxRx(id, MXRAM::TORQUE_ENABLE, enable, "Torque Enable", WRITE); }

  bool MotorSetting::setGoalPosition(uint8_t id, uint32_t position)
  { return TxRx(id, MXRAM::GOAL_POSITION, position, "Goal Position", WRITE); }

  bool MotorSetting::setGoalCurrent(uint8_t id, uint16_t current)
  { return TxRx(id, MXRAM::GOAL_CURRENT, current, "Goal Current", WRITE); }


  /*=================================================== MOTOR CONTROL SETTER(Sync) ===================================================*/
  bool MotorSetting::setTorqueSync(const std::vector<uint8_t>& motor_ids, bool enable)
  {
    RCLCPP_INFO(logger_, "[SyncWrite] 토크 설정 시작 - 모터 개수: %zu, 토크: %s", 
                motor_ids.size(), enable ? "ON" : "OFF");

    dynamixel::GroupSyncWrite syncWrite(port_handler_, packet_handler_, 
                                        MXRAM::TORQUE_ENABLE.first, MXRAM::TORQUE_ENABLE.second);

    uint8_t torque_value = enable ? 1 : 0;
    
    for (size_t i = 0; i < motor_ids.size(); i++)
    {
      std::vector<uint8_t> data(1);
      data[0] = torque_value;
      
      bool dxl_addparam_result = syncWrite.addParam(motor_ids[i], data.data());
      if (!dxl_addparam_result)
      {
        RCLCPP_ERROR(logger_, "[SyncWrite] 토크 파라미터 추가 실패 - ID: %d", motor_ids[i]);
        return false;
      }
      RCLCPP_INFO(logger_, "[SyncWrite] 토크 파라미터 추가 - ID: %d, 값: %d", motor_ids[i], torque_value);
    }

    int dxl_comm_result = syncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "[SyncWrite] 토크 설정 실패: %s", packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    RCLCPP_INFO(logger_, "[SyncWrite] 토크 설정 성공 - 모든 모터 토크 %s", enable ? "ON" : "OFF");
    return true;
  }


  /*=================================================== MOTOR OPERATING MODE SETTER(Individual) ===================================================*/
    bool MotorSetting::setOperatingMode(uint8_t id, uint8_t mode)
  { return TxRx(id, EEPROM::OPERATING_MODE, mode, "Operating Mode", WRITE); }

    bool MotorSetting::switchToCurrentMode(uint8_t id, uint8_t mode)
  { return setOperatingMode(id, mode); }


  /*=================================================== MOTOR LIMIT SETTER(Individual) ===================================================*/
  bool MotorSetting::setMinPositionLimit(uint8_t id, uint32_t limit)
  { return TxRx(id, EEPROM::MIN_POSITION_LIMIT, limit, "Min Position Limit", WRITE); }

  bool MotorSetting::setMaxPositionLimit(uint8_t id, uint32_t limit)
  { return TxRx(id, EEPROM::MAX_POSITION_LIMIT, limit, "Max Position Limit", WRITE); }

  bool MotorSetting::setMaxVelocityLimit(uint8_t id, uint32_t limit)
  { return TxRx(id, EEPROM::VELOCITY_LIMIT, limit, "Max Velocity Limit", WRITE); }

  bool MotorSetting::setMaxAccelerationLimit(uint8_t id, uint32_t limit)
  { return TxRx(id, EEPROM::ACCELERATION_LIMIT, limit, "Max Acceleration Limit", WRITE); }

  bool MotorSetting::setTemperatureLimit(uint8_t id, uint8_t limit)
  { return TxRx(id, EEPROM::TEMPERATURE_LIMIT, limit, "Temperature Limit", WRITE); }

  bool MotorSetting::setCurrentLimit(uint8_t id, uint16_t limit)
  { return TxRx(id, EEPROM::CURRENT_LIMIT, limit, "Current Limit", WRITE); }

  bool MotorSetting::setPwmLimit(uint8_t id, uint16_t limit)
  { return TxRx(id, EEPROM::PWM_LIMIT, limit, "PWM Limit", WRITE); }

  bool MotorSetting::setShutdown(uint8_t id, uint8_t shutdown)
  { return TxRx(id, EEPROM::SHUTDOWN, shutdown, "Shutdown", WRITE); }


  /*=================================================== MOTOR LIMIT SETTER(Bulk) ===================================================*/
  bool MotorSetting::setMinPositionLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::MIN_POSITION_LIMIT, motor_ids, settings, "min position limit"); }

  bool MotorSetting::setMaxPositionLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::MAX_POSITION_LIMIT, motor_ids, settings, "max position limit"); }

  bool MotorSetting::setMaxVelocityLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::VELOCITY_LIMIT, motor_ids, settings, "max velocity limit"); }

  bool MotorSetting::setMaxAccelerationLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::ACCELERATION_LIMIT, motor_ids, settings, "max acceleration limit"); }

  bool MotorSetting::setTemperatureLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::TEMPERATURE_LIMIT, motor_ids, settings, "temperature limit"); }

  bool MotorSetting::setCurrentLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::CURRENT_LIMIT, motor_ids, settings, "current limit"); }

  bool MotorSetting::setPwmLimitBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::PWM_LIMIT, motor_ids, settings, "pwm limit"); }

  bool MotorSetting::setShutdownBulk(const std::vector<uint8_t> &motor_ids, const std::vector<MotorSettingConfig> &settings)
  { return BulkWrite(EEPROM::SHUTDOWN, motor_ids, settings, "shutdown"); }

  bool MotorSetting::setGoalPositionBulk(const std::vector<uint8_t>& motor_ids, const std::vector<MotorSettingConfig>& settings)
  { return BulkWrite(MXRAM::GOAL_POSITION, motor_ids, settings, "goal position"); }

  template <typename C>
  bool MotorSetting::DefaultSettingChange(uint8_t change_set_mode, const std::vector<C> &change_set_value_vec,
                                         const std::vector<uint8_t> &motor_ids, std::vector<MotorSettingConfig> &motor_settings)
  {
    // change_set_mode : 설정할 모드
    // change_set_value : 설정 값

    for (auto id : motor_ids)
    {
      // 토크 해제
      if (!setTorque(id, TORQUEOFF))
      {
        return false;
      }
    }

    switch (change_set_mode)
    {
    case MIN_POSITION_LIMIT_CASE:
      // 최소 위치 한계 설정 (기본값: 0)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].min_position_limit = radianToTick(change_set_value_vec[i]);
      }
      if (!setMinPositionLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case MAX_POSITION_LIMIT_CASE:
      // 최대 위치 한계 설정 (기본값 : 4095)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].max_position_limit = radianToTick(change_set_value_vec[i]);
      }
      if (!setMaxPositionLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case VELOCITY_LIMIT_CASE:
      // 최대 속도 설정 (기본값 : 210)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].max_velocity_limit = static_cast<uint32_t>(change_set_value_vec[i]);
      }
      if (!setMaxVelocityLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case ACCELERATION_LIMIT_CASE:
      // 최대 가속도 설정 (기본값 : 50)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].max_acceleration_limit = static_cast<uint32_t>(change_set_value_vec[i]);
      }
      if (!setMaxAccelerationLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case TEMPERATURE_LIMIT_CASE:
      // 온도 제한 설정 (기본값: 80)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].temperature_limit = static_cast<uint8_t>(change_set_value_vec[i]);
      }
      if (!setTemperatureLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case CURRENT_LIMIT_CASE:
      // 전류 제한 설정 (기본값: 2047)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].current_limit = static_cast<uint16_t>(change_set_value_vec[i]);
      }
      if (!setCurrentLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case PWM_LIMIT_CASE:
      // PWM 제한 설정 (기본값: 885)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].pwm_limit = static_cast<uint16_t>(change_set_value_vec[i]);
      }
      if (!setPwmLimitBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    case SHUTDOWN_CASE:
      // 셧다운 설정 (기본값: 52)
      for (size_t i = 0; i < motor_ids.size(); i++)
      {
        motor_settings[i].shutdown = static_cast<uint8_t>(change_set_value_vec[i]);
      }
      if (!setShutdownBulk(motor_ids, motor_settings))
      {
        return false;
      }
      break;

    default:
      // 모든 모터를 기본값으로 설정
      RCLCPP_INFO(logger_, "exceeded the size of the mode you can select");
      RCLCPP_INFO(logger_, "Setting all motors to default values");
      break;
    }

    // 토크 설정
    for (auto id : motor_ids)
    {
      if (!setTorque(id, TORQUEON))
      {
        return false;
      }
    }

    return true;
  }


  /*=================================================== PACKET AND COMMUNICATION SETTER ===================================================*/

  void MotorSetting::divide_byte(std::vector<uint8_t> &data, int address, int byte_size)
  {
    switch (byte_size)
    {
    case 1:
      data.push_back(address & 0xFF);
      break;
    case 2:
      data.push_back(DXL_LOBYTE(address));
      data.push_back(DXL_HIBYTE(address));
      break;
    case 4:
      data.push_back(DXL_LOBYTE(DXL_LOWORD(address)));
      data.push_back(DXL_HIBYTE(DXL_LOWORD(address)));
      data.push_back(DXL_LOBYTE(DXL_HIWORD(address)));
      data.push_back(DXL_HIBYTE(DXL_HIWORD(address)));
      break;
    }
  }

  int32_t radianToTick(double rad)
  { return static_cast<uint32_t>((rad + M_PI) * (4095.0 / (2 * M_PI))); }

  bool MotorSetting::sendMotorPacket(const std::vector<uint8_t> &motor_ids, const std::vector<double> &position,
                                     const std::vector<double> &velocity, const std::vector<double> &acceleration)
  {
    dynamixel::GroupSyncWrite SyncWrite(port_handler_, packet_handler_, MXRAM::PROFILE_ACCELERATION.first, 12);

    for (size_t i = 0; i < motor_ids.size(); i++)
    {
      std::vector<uint8_t> control_data_vector;

      uint32_t goal_position = static_cast<uint32_t>((position[i] + M_PI) * (4095.0 / (2 * M_PI)));
      uint32_t goal_velocity = static_cast<uint32_t>(velocity[i] <= 0.0 ? 0 : velocity[i]);
      uint32_t goal_acceleration = static_cast<uint32_t>(acceleration[i] <= 0.0 ? 0 : acceleration[i]);

      divide_byte(control_data_vector, goal_acceleration, 4);
      divide_byte(control_data_vector, goal_velocity, 4);
      divide_byte(control_data_vector, goal_position, 4);

      if (!SyncWrite.addParam(motor_ids[i], control_data_vector.data()))
      {
        RCLCPP_ERROR(logger_, "Failed to add parameter for ID %d", motor_ids[i]);
        return false;
      }
    }

    int dxl_comm_result = SyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Failed to send motor packet: %s", packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    SyncWrite.clearParam();
    return true;
  }


  template <typename T>
  bool MotorSetting::TxRx(uint8_t id, const std::pair<int, int> &control_table_address, T &value, const std::string &status_name, int mode)
  {
    dxl_variable_init();

    RCLCPP_INFO(logger_, "[TxRx] 시작 - ID: %d, 주소: %d, 크기: %d, 값: %d, 작업: %s", 
                id, control_table_address.first, control_table_address.second, static_cast<int>(value), status_name.c_str());

    if (mode == WRITE)
    {
      if constexpr (sizeof(T) == 1)
      {
        RCLCPP_INFO(logger_, "[TxRx] 1바이트 쓰기 시도 - ID: %d, 주소: %d, 값: %d", id, control_table_address.first, static_cast<uint8_t>(value));
        dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, control_table_address.first, static_cast<uint8_t>(value), &dxl_error_);
      }
      else if constexpr (sizeof(T) == 2)
      {
        RCLCPP_INFO(logger_, "[TxRx] 2바이트 쓰기 시도 - ID: %d, 주소: %d, 값: %d", id, control_table_address.first, static_cast<uint16_t>(value));
        dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, control_table_address.first, static_cast<uint16_t>(value), &dxl_error_);
      }
      else if constexpr (sizeof(T) == 4)
      {
        RCLCPP_INFO(logger_, "[TxRx] 4바이트 쓰기 시도 - ID: %d, 주소: %d, 값: %d", id, control_table_address.first, static_cast<uint32_t>(value));
        dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, id, control_table_address.first, static_cast<uint32_t>(value), &dxl_error_);
      }
      else
      {
        RCLCPP_ERROR(logger_, "[TxRx] 지원하지 않는 데이터 크기 - %s, 크기: %zu", status_name.c_str(), sizeof(T));
        return false;
      }

      RCLCPP_INFO(logger_, "[TxRx] 통신 결과 - ID: %d, 결과: %d, 에러: %d", id, dxl_comm_result_, dxl_error_);

      if (dxl_comm_result_ != COMM_SUCCESS)
      {
        RCLCPP_ERROR(logger_, "[TxRx] 통신 실패 - %s for ID %d: %s (코드: %d)", status_name.c_str(), id, packet_handler_->getTxRxResult(dxl_comm_result_), dxl_comm_result_);
        return false;
      }

      if (dxl_error_ != 0)
      {
        RCLCPP_ERROR(logger_, "[TxRx] 다이나믹셀 에러 - %s for ID %d: %s (코드: %d)", status_name.c_str(), id, packet_handler_->getRxPacketError(dxl_error_), dxl_error_);
        return false;
      }

      RCLCPP_INFO(logger_, "[TxRx] 성공 - %s for ID %d", status_name.c_str(), id);
      return true;
    }

    RCLCPP_ERROR(logger_, "Unsupported mode for motor setting: %d", mode);
    return false;
  }

  bool MotorSetting::BulkWrite(const std::pair<int, int> &control_table_address, const std::vector<uint8_t> &motor_ids,
                               const std::vector<MotorSettingConfig> &settings, const std::string &status_name)
  {
    dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

    for (size_t i = 0; i < motor_ids.size(); i++)
    {
      std::vector<uint8_t> control_data;

      if (status_name == "min position limit")
        divide_byte(control_data, settings[i].min_position_limit, control_table_address.second);
      else if (status_name == "max position limit")
        divide_byte(control_data, settings[i].max_position_limit, control_table_address.second);
      else if (status_name == "max velocity limit")
        divide_byte(control_data, settings[i].max_velocity_limit, control_table_address.second);
      else if (status_name == "max acceleration limit")
        divide_byte(control_data, settings[i].max_acceleration_limit, control_table_address.second);
      else if (status_name == "current limit")
        divide_byte(control_data, settings[i].current_limit, control_table_address.second);
      else if (status_name == "temperature limit")
        divide_byte(control_data, settings[i].temperature_limit, control_table_address.second);
      else if (status_name == "pwm limit")
        divide_byte(control_data, settings[i].pwm_limit, control_table_address.second);
      else if (status_name == "shutdown")
        divide_byte(control_data, settings[i].shutdown, control_table_address.second);
      else
      {
        RCLCPP_ERROR(logger_, "Unknown status name: %s", status_name.c_str());
        return false;
      }

      if (!bulkWrite.addParam(motor_ids[i], control_table_address.first, control_table_address.second, control_data.data()))
      {
        RCLCPP_ERROR(logger_, "Failed to add parameter to bulk write for %s, ID %d", status_name.c_str(), motor_ids[i]);
        return false;
      }
    }

    int dxl_comm_result = bulkWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Bulk write failed when setting %s: %s", status_name.c_str(), packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    return true;
  }

}

