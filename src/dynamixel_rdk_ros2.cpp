#include "dynamixel_rdk_ros2/dynamixel_rdk_ros2.hpp"

namespace dynamixel_rdk_ros2
{
  dynamixel_rdk_ros2::dynamixel_rdk_ros2() : Node("dynamixel_rdk_ros2")
  {
    // 외부 Publisher 초기화
    motor_status_pubisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>("motor_status", 10);
    warning_status_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::WarningStatus>("motor_warning", 10);

    // 외부 Subscription 초기화
    ik2rdk_sub = create_subscription<dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs>(
        "dynamixel_control", 10,
        std::bind(&dynamixel_rdk_ros2::dynamixel_control_callback, this, std::placeholders::_1));

    // 파라미터 초기화
    initParameters();

    // Dynamixel 초기화
    initDynamixel();

    dxl_current_ratio = MX_CURRENT_PROFILE;
    dxl_rps_ratio = MX_RPS_PROFILE * M_PI / 30;
    dxl_acc_ratio = MX_ACC_PROFILE * M_PI / 1800;
    std::vector<MotorStatus> motor_status(TOTAL_MOTOR + 2);

    // 타이머
    getting_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&dynamixel_rdk_ros2::timer_callback, this));
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
    this->declare_parameter("dynamixels.max_position_limits", std::vector<double>{M_PI});
    this->declare_parameter("dynamixels.min_position_limits", std::vector<double>{-M_PI});
    // this->declare_parameter("dynamixels.max_velocity_limits", std::vector<int64_t>{210});
    // this->declare_parameter("dynamixels.temperature_limits", std::vector<int64_t>{80});

    // 파라미터 가져오기
    device_port_ = this->get_parameter("device_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    protocol_version_ = this->get_parameter("protocol_version").as_double();
    std::vector<int64_t> ids = this->get_parameter("ids").as_integer_array();
    std::vector<double> max_position_limits_ = get_parameter("dynamixels.max_position_limits").as_double_array();
    std::vector<double> min_position_limits_ = get_parameter("dynamixels.min_position_limits").as_double_array();
    // std::vector<int64_t> max_velocity_limits = get_parameter("dynamixels.max_velocity_limits").as_integer_array();
    // std::vector<int64_t> temperature_limits = get_parameter("dynamixels.temperature_limits").as_integer_array();

    for (size_t i = 0; i < ids.size(); ++i)
    {
      motor_ids_.push_back(static_cast<uint8_t>(ids[i]));
    }

    TOTAL_MOTOR = motor_ids_.size();

    // 모터 기본값 설정
    DefaultSettingChange(MAX_POSITION_LIMIT_CASE, max_position_limits_);
    DefaultSettingChange(MIN_POSITION_LIMIT_CASE, min_position_limits_);
    // DefaultSettingChange(VELOCITY_LIMIT_CASE, max_velocity_limits);
    // DefaultSettingChange(TEMPERATURE_LIMIT_CASE, temperature_limits);


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
    return TxRx(id, MXRAM::TORQUE_ENABLE, enable, "Torque Enable", WRITE);
  }

  // 0 - Current Control Mode, 1 - Velocity Control Mode, 3(Default) - Position Control Mode, 4 - Extended Position Control Mode (Multi-turn), 5 - Current-based Position Control Mode, 16 - PWM Control Mode
  bool dynamixel_rdk_ros2::setOperatingMode(uint8_t id, uint8_t mode)
  {
    return TxRx(id, EEPROM::OPERATING_MODE, mode, "Operating Mode", WRITE);
  }

  // -------------------------------------------------------------- Motor Status Getter --------------------------------------------------------------

  bool dynamixel_rdk_ros2::getCurrentPosition(uint8_t id, uint32_t &position)
  {
    return TxRx(id, MXRAM::PRESENT_POSITION, position, "Current Position", READ);
  }

  bool dynamixel_rdk_ros2::getGoalPosition(uint8_t id, uint32_t &goal_position)
  {
    return TxRx(id, MXRAM::GOAL_POSITION, goal_position, "Goal Position", READ);
  }

  bool dynamixel_rdk_ros2::getCurrentVelocity(uint8_t id, uint8_t &velocity)
  {
    return TxRx(id, MXRAM::PRESENT_VELOCITY, velocity, "Current Velocity", READ);
  }

  bool dynamixel_rdk_ros2::getInputVoltage(uint8_t id, uint16_t &voltage)
  {
    return TxRx(id, MXRAM::PRESENT_INPUT_VOLTAGE, voltage, "Input Voltage", READ);
  }

  bool dynamixel_rdk_ros2::getCurrentTemperature(uint8_t id, uint8_t &temperature)
  {
    return TxRx(id, MXRAM::PRESENT_TEMPERATURE, temperature, "Current Temperature", READ);
  }

  bool dynamixel_rdk_ros2::getCurrentTorque(uint8_t id, uint16_t &torque)
  {
    return TxRx(id, MXRAM::PRESENT_CURRENT, torque, "Current Torque", READ);
  }

  bool dynamixel_rdk_ros2::getMovingStatus(uint8_t id, uint8_t &moving_status)
  {
    return TxRx(id, MXRAM::MOVING_STATUS, moving_status, "Moving Status", READ);
  }

  bool dynamixel_rdk_ros2::HardwareErrorStatus(uint8_t id, uint8_t &error_status)
  {
    return TxRx(id, MXRAM::HARDWARE_ERROR_STATUS, error_status, "Hardware Error Status", READ);
  }

  // -------------------------------------------------------------- Motor Status Getter (Sync) --------------------------------------------------------------

  bool dynamixel_rdk_ros2::getCurrentPositionSync()
  {
    return SyncRead(MXRAM::PRESENT_POSITION, motor_status, "Current Position");
  }

  bool dynamixel_rdk_ros2::getGoalPositionSync()
  {
    return SyncRead(MXRAM::GOAL_POSITION, motor_status, "Goal Position");
  }

  bool dynamixel_rdk_ros2::getCurrentVelocitySync()
  {
    return SyncRead(MXRAM::PRESENT_VELOCITY, motor_status, "Current Velocity");
  }

  bool dynamixel_rdk_ros2::getInputVoltageSync()
  {
    return SyncRead(MXRAM::PRESENT_INPUT_VOLTAGE, motor_status, "Input Voltage");
  }

  bool dynamixel_rdk_ros2::getCurrentTemperatureSync()
  {
    return SyncRead(MXRAM::PRESENT_TEMPERATURE, motor_status, "Current Temperature");
  }

  bool dynamixel_rdk_ros2::getCurrentTorqueSync()
  {
    return SyncRead(MXRAM::PRESENT_CURRENT, motor_status, "Current Torque");
  }

  bool dynamixel_rdk_ros2::getMovingStatusSync()
  {
    return SyncRead(MXRAM::MOVING_STATUS, motor_status, "Moving Status");
  }

  bool dynamixel_rdk_ros2::HardwareErrorStatusSync()
  {
    return SyncRead(MXRAM::HARDWARE_ERROR_STATUS, motor_status, "Hardware Error Status");
  }

  // -------------------------------------------------------------- Motor Status Setter --------------------------------------------------------------

  bool dynamixel_rdk_ros2::setMinPositionLimit()
  {
    return BulkWrite(EEPROM::MIN_POSITION_LIMIT, motor_settings, "min position limit");
  }

  bool dynamixel_rdk_ros2::setMaxPositionLimit()
  {
    return BulkWrite(EEPROM::MAX_POSITION_LIMIT, motor_settings, "max position limit");
  }

  bool dynamixel_rdk_ros2::setMaxVelocityLimit()
  {
    return BulkWrite(EEPROM::VELOCITY_LIMIT, motor_settings, "max velocity limit");
  }

  bool dynamixel_rdk_ros2::setMaxAccelerationLimit()
  {
    return BulkWrite(EEPROM::ACCELERATION_LIMIT, motor_settings, "max acceleration limit");
  }

  bool dynamixel_rdk_ros2::setCurrentLimit()
  {
    return BulkWrite(EEPROM::CURRENT_LIMIT, motor_settings, "current limit");
  }

  bool dynamixel_rdk_ros2::setTemperatureLimit()
  {
    return BulkWrite(EEPROM::TEMPERATURE_LIMIT, motor_settings, "temperature limit");
  }

  bool dynamixel_rdk_ros2::setPwmLimit()
  {
    return BulkWrite(EEPROM::PWM_LIMIT, motor_settings, "pwm limit");
  }

  bool dynamixel_rdk_ros2::setShutdown()
  {
    return BulkWrite(EEPROM::SHUTDOWN, motor_settings, "shutdown");
  }

// change_set_mode : 0 - Operating Mode, 1 - Min Position Limit, 2 - Max Position Limit, 3 - Max Velocity Limit, 4 - Acceleration Limit, 5 - Temperature Limit, 6 - Current Limit, 7 - PWM Limit, 8 - Shutdown
template<typename C>
bool dynamixel_rdk_ros2::DefaultSettingChange(uint8_t change_set_mode, const std::vector<C>& change_set_value_vec)
{
  // change_set_mode : 설정할 모드
  // change_set_value : 설정 값
  // change_set_mode가 -1일 경우 모두 기본값으로 설정

  for (auto id : motor_ids_)
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
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].min_position_limit = radianToTick(change_set_value_vec[i]);
    }
    if (!setMinPositionLimit())
    {
        return false;
    }
    break;

  case MAX_POSITION_LIMIT_CASE:
    // 최대 위치 한계 설정 (기본값 : 4095)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].max_position_limit = radianToTick(change_set_value_vec[i]);
    }
    if (!setMaxPositionLimit())
    {
      return false;
    }
    break;

  case VELOCITY_LIMIT_CASE:
    // 최대 속도 설정 (기본값 : 210)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].max_velocity_limit = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if (!setMaxVelocityLimit())
    {
      return false;
    }
    break;

  case ACCELERATION_LIMIT_CASE:
    // 최대 가속도 설정 (기본값 : 50)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].max_acceleration_limit = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if (!setMaxAccelerationLimit())
    {
      return false;
    }
    break;

  case TEMPERATURE_LIMIT_CASE:
    // 온도 제한 설정 (기본값: 80)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].temperature_limit = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if( !setTemperatureLimit())
    {
      return false;
    }
    break;

  case CURRENT_LIMIT_CASE:
    // 전류 제한 설정 (기본값: 2047)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].current_limit = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if(!setCurrentLimit())
    {
      return false;
    }
    break;

  case PWM_LIMIT_CASE:
    // PWM 제한 설정 (기본값: 885)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].pwm_limit = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if (!setPwmLimit())
    {
      return false;
    }

    break;

  case SHUTDOWN_CASE:
    // 셧다운 설정 (기본값: 52)
    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      motor_settings[TOTAL_MOTOR].shutdown = static_cast<int32_t>(change_set_value_vec[i]);
    }
    if (!setShutdown())
    {
      return false;
    }
    break;

  default:
    // 모든 모터를 기본값으로 설정
    RCLCPP_INFO(this->get_logger(), "exceeded the size of the mode you can select");
    RCLCPP_INFO(this->get_logger(), "Setting all motors to default values");

    break;
  }

  // 토크 설정
  for (auto id : motor_ids_)
  {
    if (!setTorque(id, TORQUEON))
    {
      return false;
    }
  }

  return true;
}

  // -------------------------------------------------------------- Motor Control --------------------------------------------------------------

  bool dynamixel_rdk_ros2::setGoalPosition(uint8_t id, uint32_t position)
  {
    return TxRx(id, MXRAM::GOAL_POSITION, position, "Set Goal Position", WRITE);
  }

  bool dynamixel_rdk_ros2::setGoalPositionBulk()
  {
    return BulkWrite(MXRAM::GOAL_POSITION, motor_settings, "Set Goal Position Bulk");
  }

  bool dynamixel_rdk_ros2::switchTocurrentMode(uint8_t id, uint8_t mode)
  {
    return TxRx(id, EEPROM::OPERATING_MODE, mode, "Switch to Current Mode", WRITE);
  }

  bool dynamixel_rdk_ros2::setGoalCurrent(uint8_t id, uint16_t current)
  {
    return TxRx(id, MXRAM::GOAL_CURRENT, current, "Set Goal Current", WRITE);
  }

  bool dynamixel_rdk_ros2::sendMotorPacket(std::vector<double> position, std::vector<double> velocity, std::vector<double> acceleration)
  {
    dynamixel::GroupSyncWrite SyncWrite(port_handler_, packet_handler_, MXRAM::PROFILE_ACCELERATION.first, 12);

    std::vector<uint8_t> control_data_vector;

    for (int i = 0; i < TOTAL_MOTOR; i++)
    {
      uint32_t goal_position = radianToTick(position[i]);
      uint32_t goal_velocity = velToRadian(velocity[i]);
      uint32_t goal_acceleration = accToRadian(acceleration[i]);

      divide_byte(control_data_vector, goal_acceleration, 4);
      divide_byte(control_data_vector, goal_velocity, 4);
      divide_byte(control_data_vector, goal_position, 4);

      if (!SyncWrite.addParam(motor_ids_[i], control_data_vector.data()))
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

  // -------------------------------------------------------------- Timer Callback --------------------------------------------------------------
  // getting and publishing motor status
  void dynamixel_rdk_ros2::timer_callback()
  {
    motorCheck(); // 연결 상태 확인

    MotorStatus status;
    dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus msg;
    const size_t TOTAL_CONNECT_MOTORS = connected_motor_ids_.size();

    ResizeMsg(msg, TOTAL_CONNECT_MOTORS);

    bool sync_success = false;
        getCurrentPositionSync() &&
        getGoalPositionSync() &&
        getCurrentVelocitySync() &&
        getInputVoltageSync() &&
        getCurrentTemperatureSync() &&
        getCurrentTorqueSync() &&
        getMovingStatusSync() &&
        HardwareErrorStatusSync();

    if (!sync_success)
    {
      RCLCPP_WARN(this->get_logger(), "Sync status read failed, switching to individual motor status read");

      for (size_t index = 0; index < TOTAL_CONNECT_MOTORS; index++)
      {
        uint8_t id = connected_motor_ids_[index];

        if (!getCurrentPosition(id, status.position)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Current Position", id);
        else if (!getGoalPosition(id, status.goal_position)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Goal Position", id);
        else if (!getCurrentVelocity(id, status.velocity)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Current Velocity", id);
        else if (!getInputVoltage(id, status.voltage)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Input Voltage", id);
        else if (!getCurrentTemperature(id, status.temperature)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Current Temperature", id);
        else if (!getCurrentTorque(id, status.torque)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Current Torque", id);
        else if (!getMovingStatus(id, status.moving_status)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Moving Status", id);
        else if (!HardwareErrorStatus(id, status.error_status)) RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to get Hardware Error Status", id);

        msgUpdate(msg, index, status.position, status.velocity, status.voltage, status.temperature, status.torque, status.moving_status, status.error_status);
      }
    }

    RCLCPP_INFO(this->get_logger(), "모터 총 개수 : %zu", motor_ids_.size());
    RCLCPP_INFO(this->get_logger(), "총 연결된 모터 개수 : %zu", TOTAL_CONNECT_MOTORS);

    motor_status_pubisher_->publish(msg);
  }

  void dynamixel_rdk_ros2::dynamixel_control_callback(const dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs &msg)
  {
    std::vector<double> goal_positions;
    std::vector<double> goal_velocities;
    std::vector<double> goal_accelerations;

    for (auto &motor_control : msg.motor_control)
    {
      goal_positions.push_back(motor_control.goal_position);
      goal_velocities.push_back(motor_control.profile_velocity);
      goal_accelerations.push_back(motor_control.profile_acceleration);
    }

    sendMotorPacket(goal_positions, goal_velocities, goal_accelerations);
  }

  // -------------------------------------------------------------- Utility --------------------------------------------------------------

  bool dynamixel_rdk_ros2::errorInterface(uint8_t id)
  {
    uint8_t error_status = 0;

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

  void dynamixel_rdk_ros2::dxl_variable_init()
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
  }

  void dynamixel_rdk_ros2::ResizeMsg(dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus &msg, size_t size)
  {
    msg.position.resize(size);
    msg.velocity.resize(size);
    msg.input_voltage.resize(size);
    msg.temperature.resize(size);
    msg.torque.resize(size);
    msg.moving_status.resize(size);
    msg.error_status.resize(size);
  }

  void dynamixel_rdk_ros2::msgUpdate(dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus &msg,
                                     size_t index,
                                     uint32_t position, uint8_t velocity, uint16_t voltage,
                                     uint8_t temperature, uint16_t torque, uint8_t moving_status, uint8_t error_status)
  {
    msg.position[index] = position;
    msg.velocity[index] = velocity;
    msg.input_voltage[index] = voltage;
    msg.temperature[index] = temperature;
    msg.torque[index] = torque;
    msg.moving_status[index] = moving_status;
    msg.error_status[index] = error_status;
  }


  void dynamixel_rdk_ros2::divide_byte(std::vector<uint8_t> &data, int address, int byte_size)
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

  template <typename T>
  bool dynamixel_rdk_ros2::TxRx(uint8_t id, const std::pair<int, int> &control_table_adress, T &value, const std::string &status_name, int mode)
  { // mode: 0 - Read, 1 - Write
    dxl_variable_init();

    // Read - 모터에서 읽은 raw 바이트 데이터를 채워넣을 포인터 주소를 전달해야 하므로 reinterpret_cast
    if (mode == READ)
    {
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
    }

    // value의 실제 값을 보내야 하므로 static_cast
    if (mode == WRITE)
    {
      if constexpr (sizeof(T) == 1)
      {
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint8_t>(value), &dxl_error);
      }
      else if constexpr (sizeof(T) == 2)
      {
        dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint16_t>(value), &dxl_error);
      }
      else if constexpr (sizeof(T) == 4)
      {
        dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, control_table_adress.first, static_cast<uint32_t>(value), &dxl_error);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unsupported data size for %s", status_name.c_str());
        return false;
      }

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to write %s for ID %d: %s",
                     status_name.c_str(), id, packet_handler_->getTxRxResult(dxl_comm_result));
        return false;
      }

      if (dxl_error != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Dynamixel error while writing %s for ID %d: %s",
                     status_name.c_str(), id, packet_handler_->getRxPacketError(dxl_error));
        return false;
      }

      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Unknown mode value: %d (READ = %d, WRITE = %d)", mode, READ, WRITE);
    return false;
  }

bool dynamixel_rdk_ros2::SyncRead(const std::pair<int, int> &control_table_address, std::vector<MotorStatus> &status_values, const std::string &status_name)
{
  dynamixel::GroupSyncRead syncRead(port_handler_, packet_handler_, control_table_address.first, control_table_address.second);

  for (int i = 0; i < TOTAL_MOTOR; i++)
  {
    bool dxl_addparam_result = syncRead.addParam(motor_ids_[i]);

    if (!dxl_addparam_result)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to add parameter to SyncRead for %s, ID %d",
                   status_name.c_str(), motor_ids_[i]);
      return false;
    }
  }

  int dxl_comm_result = syncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "SyncRead failed when getting %s: %s",
                 status_name.c_str(), packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  for (int i = 0; i < TOTAL_MOTOR; i++)
  {
    if (syncRead.isAvailable(motor_ids_[i], control_table_address.first, control_table_address.second))
    {
      if(status_name == "Current Position") status_values[i].position = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Goal Position") status_values[i].goal_position = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Current Velocity") status_values[i].velocity = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Input Voltage") status_values[i].voltage = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Current Temperature") status_values[i].temperature = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Current Torque") status_values[i].torque = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Moving Status") status_values[i].moving_status = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
      else if(status_name == "Hardware Error Status") status_values[i].error_status = syncRead.getData(motor_ids_[i], control_table_address.first, control_table_address.second);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get %s for ID %d",
                   status_name.c_str(), motor_ids_[i]);
      return false;
    }
  }

  return true;
}

bool dynamixel_rdk_ros2::BulkWrite(const std::pair<int, int> &control_table_address, const std::vector<MotorSettings> &values, const std::string &status_name)
{
  dynamixel::GroupBulkWrite bulkWrite(port_handler_, packet_handler_);

  for (int i = 0; i < TOTAL_MOTOR; i++)
  {
    std::vector<uint8_t> control_data;

    if(status_name == "min position limit") divide_byte(control_data, values[i].min_position_limit, control_table_address.second);
    else if(status_name == "max position limit") divide_byte(control_data, values[i].max_position_limit, control_table_address.second);
    else if(status_name == "max velocity limit") divide_byte(control_data, values[i].max_velocity_limit, control_table_address.second);
    else if(status_name == "max acceleration limit") divide_byte(control_data, values[i].max_acceleration_limit, control_table_address.second);
    else if(status_name == "current limit") divide_byte(control_data, values[i].current_limit, control_table_address.second);
    else if(status_name == "temperature limit") divide_byte(control_data, values[i].temperature_limit, control_table_address.second);
    else if(status_name == "pwm limit") divide_byte(control_data, values[i].pwm_limit, control_table_address.second);
    else if(status_name == "shutdown") divide_byte(control_data, values[i].shutdown, control_table_address.second);

    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown status name: %s", status_name.c_str());
      return false;
    }

    if (!bulkWrite.addParam(motor_ids_[i], control_table_address.first, control_table_address.second, control_data.data()))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to add parameter to bulk write for %s, ID %d",
                   status_name.c_str(), motor_ids_[i]);
      return false;
    }
  }

  int dxl_comm_result = bulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Bulk write failed when setting %s: %s",
                 status_name.c_str(), packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  return true;
}


}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::dynamixel_rdk_ros2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
