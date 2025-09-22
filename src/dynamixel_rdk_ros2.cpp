#include "dynamixel_rdk_ros2/dynamixel_rdk_ros2.hpp"
#include "dynamixel_rdk_ros2/motor_status.hpp"
#include <thread>
#include <chrono>
#include <cmath>

namespace dynamixel_rdk_ros2
{
dynamixel_rdk_ros2::dynamixel_rdk_ros2() : Node("dynamixel_rdk_ros2")
{
  // Publisher
  motor_status_pubisher_ = this->create_publisher<dynamixel_rdk_msgs::msg::CurrentMotorStatus>("motor_status", 2000);
  warning_status_publisher_ = this->create_publisher<dynamixel_rdk_msgs::msg::WarningStatus>("motor_warning", 2000);

  // Subscription
  ik2rdk_sub = create_subscription<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>("dynamixel_control", 10, std::bind(&dynamixel_rdk_ros2::dynamixel_control_callback, this, std::placeholders::_1));
  pantilt_sub_ = create_subscription<dynamixel_rdk_msgs::msg::DynamixelMsgs>("pan_dxl", 10, std::bind(&dynamixel_rdk_ros2::dynamixel_callback, this, std::placeholders::_1));
  if(start())
  {
    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!! 초기 설정 성공 !!!!!!!!!!!");
    // 토크 검사 타이머
    // torque_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    //                 std::bind(&dynamixel_rdk_ros2::checkAndSetTorque, this));
    // getting_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    //                 std::bind(&dynamixel_rdk_ros2::timer_callback, this));
  }
  else
  { RCLCPP_ERROR(this->get_logger(), "!!!!!!!!!!! 초기 설정 실패 !!!!!!!!!!!"); }
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

bool dynamixel_rdk_ros2::start()
{
  initParameters();

  if (!initDynamixel())
  {
    RCLCPP_ERROR(this->get_logger(), "Dynamixel 초기화 실패!");
    return false;
  }

  std::vector<uint8_t> motor_ids_uint8;
  for (auto id : motor_ids_) {
    motor_ids_uint8.push_back(static_cast<uint8_t>(id));
  }

  if (!motor_setting_handler_->setTorqueSync(motor_ids_uint8, true))
  {
    RCLCPP_ERROR(this->get_logger(), "SyncWrite 토크 설정 실패!");
    return false;
  }

  // 모터 기본값 설정
  // DefaultSettingChange(MAX_POSITION_LIMIT_CASE, max_position_limits_);
  // DefaultSettingChange(MIN_POSITION_LIMIT_CASE, min_position_limits_);
  // DefaultSettingChange(VELOCITY_LIMIT_CASE, max_velocity_limits)s;
  // DefaultSettingChange(TEMPERATURE_LIMIT_CASE, temperature_limits);

  // motor_status 초기화
  motor_status.clear();
  motor_status.resize(std::max(TOTAL_MOTOR + 2, static_cast<int>(motor_ids_.size())));

  return true;
}
  void dynamixel_rdk_ros2::initParameters()
  {
    // 파라미터 선언
    this->declare_parameter("device_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("protocol_version", 2.0);
    this->declare_parameter("ids", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23});
    this->declare_parameter("dynamixels.max_position_limits", std::vector<double>{M_PI});
    this->declare_parameter("dynamixels.min_position_limits", std::vector<double>{-M_PI});
    // this->declare_parameter("dynamixels.max_velocity_limits", std::vector<int64_t>{210});
    // this->declare_parameter("dynamixels.temperature_limits", std::vector<int64_t>{80});

    // 파라미터 가져오기
    device_port_ = this->get_parameter("device_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    protocol_version_ = this->get_parameter("protocol_version").as_double();
    std::vector<int64_t> ids = this->get_parameter("ids").as_integer_array();
    max_position_limits_ = get_parameter("dynamixels.max_position_limits").as_double_array();
    min_position_limits_ = get_parameter("dynamixels.min_position_limits").as_double_array();
    // std::vector<int64_t> max_velocity_limits = get_parameter("dynamixels.max_velocity_limits").as_integer_array();
    // std::vector<int64_t> temperature_limits = get_parameter("dynamixels.temperature_limits").as_integer_array();

    for (size_t i = 0; i < ids.size(); ++i)
    {
      motor_ids_.push_back(static_cast<uint8_t>(ids[i]));
    }

    TOTAL_MOTOR = motor_ids_.size();

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

    motor_status_handler_ = std::make_unique<MotorStatus>(
        port_handler_, packet_handler_, this->get_logger());

    motor_setting_handler_ = std::make_unique<MotorSetting>(
        port_handler_, packet_handler_, this->get_logger());

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
    dynamixel_rdk_msgs::msg::WarningStatus msg;

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

    warning_status_publisher_->publish(msg);


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
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Total motors: %zu", connected_motor_ids_.size());
    return true;
  }

  bool dynamixel_rdk_ros2::setTorque(uint8_t id, bool enable)
  {
    return motor_setting_handler_->setTorque(id, enable);
  }

  // -------------------------------------------------------------- Timer Callback --------------------------------------------------------------
  // getting and publishing motor status
void dynamixel_rdk_ros2::timer_callback()
{
  // 핸들러 초기화 확인
  if (!motor_status_handler_ || !port_handler_ || !packet_handler_) {
    RCLCPP_ERROR(this->get_logger(), "모터 핸들러가 초기화되지 않았습니다!");
    return;
  }

  static int read_cycle = 0;

  MotorStatus::MotorStatusConfig status;
  dynamixel_rdk_msgs::msg::CurrentMotorStatus msg;
  const size_t TOTAL_CONNECT_MOTORS = connected_motor_ids_.size();

  // 연결된 모터가 없으면 리턴
  if (TOTAL_CONNECT_MOTORS == 0) {
    RCLCPP_WARN(this->get_logger(), "연결된 모터가 없습니다");
    return;
  }

  // motor_status 벡터 크기 확인 및 조정
  if (motor_status.size() < TOTAL_CONNECT_MOTORS) {
    motor_status.resize(TOTAL_CONNECT_MOTORS);
  }

  ResizeMsg(msg, TOTAL_CONNECT_MOTORS);

  std::vector<uint8_t> connected_motor_uint8_ids;
  for (auto id : connected_motor_ids_)
  {
    connected_motor_uint8_ids.push_back(static_cast<uint8_t>(id));
  }

  //매번 모든 상태를 읽지 않고 번갈아가며 읽기
  bool sync_success = false;
  try {
    if (read_cycle % 3 == 0) {
      // 가장 중요한 위치와 목표위치만 읽기
      sync_success =
          motor_status_handler_->getCurrentPositionSync(connected_motor_uint8_ids, motor_status) &&
          motor_status_handler_->getGoalPositionSync(connected_motor_uint8_ids, motor_status);
      RCLCPP_DEBUG(this->get_logger(), "읽기 사이클 0: 위치 데이터");
    } else if (read_cycle % 3 == 1) {
      // 속도와 이동 상태 읽기
      sync_success =
          motor_status_handler_->getCurrentVelocitySync(connected_motor_uint8_ids, motor_status) &&
          motor_status_handler_->getMovingStatusSync(connected_motor_uint8_ids, motor_status);
      RCLCPP_DEBUG(this->get_logger(), "읽기 사이클 1: 속도/이동 데이터");
    } else {
      // 전압, 온도, 토크, 에러 상태 읽기
      sync_success =
          motor_status_handler_->getInputVoltageSync(connected_motor_uint8_ids, motor_status) &&
          motor_status_handler_->getCurrentTemperatureSync(connected_motor_uint8_ids, motor_status) &&
          motor_status_handler_->getCurrentTorqueSync(connected_motor_uint8_ids, motor_status) &&
          motor_status_handler_->HardwareErrorStatusSync(connected_motor_uint8_ids, motor_status);
      RCLCPP_DEBUG(this->get_logger(), "읽기 사이클 2: 전압/온도/토크/에러 데이터");
    }
    read_cycle++;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "동기 읽기 중 예외 발생: %s", e.what());
    sync_success = false;
  }

  if (!sync_success)
  {
    RCLCPP_WARN(this->get_logger(), "동기 상태 읽기 실패, 개별 모터 상태 읽기로 전환");

    for (size_t index = 0; index < TOTAL_CONNECT_MOTORS; index++)
    {
      uint8_t id = connected_motor_uint8_ids[index];

      if (motor_status_handler_->getCurrentPosition(id, status.position) &&
          motor_status_handler_->getGoalPosition(id, status.goal_position) &&
          motor_status_handler_->getCurrentVelocity(id, status.velocity) &&
          motor_status_handler_->getInputVoltage(id, status.voltage) &&
          motor_status_handler_->getCurrentTemperature(id, status.temperature) &&
          motor_status_handler_->getCurrentTorque(id, status.torque) &&
          motor_status_handler_->getMovingStatus(id, status.moving_status) &&
          motor_status_handler_->HardwareErrorStatus(id, status.error_status))
      {
        msgUpdate(msg, index, status.position, status.goal_position, status.velocity, status.voltage,
                  status.temperature, status.torque, status.moving_status, status.error_status);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "[ID:%d] 개별 상태 읽기 실패", id);
      }
    }
  }
  else
  {
    for (size_t index = 0; index < TOTAL_CONNECT_MOTORS; index++)
    {
      msgUpdate(msg, index, motor_status[index].position, motor_status[index].goal_position, motor_status[index].velocity,
                motor_status[index].voltage, motor_status[index].temperature, motor_status[index].torque,
                motor_status[index].moving_status, motor_status[index].error_status);
    }
  }

  static int log_counter = 0;
  if (log_counter++ % 20 == 0) {
    RCLCPP_INFO(this->get_logger(), "모터 총 개수 : %zu", motor_ids_.size());
    RCLCPP_INFO(this->get_logger(), "총 연결된 모터 개수 : %zu", TOTAL_CONNECT_MOTORS);
  }

  motor_status_pubisher_->publish(msg);
}
  void dynamixel_rdk_ros2::dynamixel_control_callback(const dynamixel_rdk_msgs::msg::DynamixelControlMsgs &msg)
  {
    RCLCPP_INFO(this->get_logger(), "==========================================================");

    // Validate input sizes
    if (msg.motor_control.size() < motor_ids_.size() - 1)
    {
      RCLCPP_ERROR(this->get_logger(), "motor_control size(%zu) < motor_ids size(%zu). Skip sending.",
                   msg.motor_control.size(), motor_ids_.size());
      return;
    }
    if (!motor_setting_handler_)
    {
      RCLCPP_ERROR(this->get_logger(), "motor_setting_handler_ is not initialized.");
      return;
    }

    // Build one combined packet for all motors
    const size_t N = motor_ids_.size() - 1;
    std::vector<uint8_t> all_ids; all_ids.reserve(N);
    std::vector<double> goal_positions; goal_positions.reserve(N);
    std::vector<double> goal_velocities; goal_velocities.reserve(N);
    std::vector<double> goal_accelerations; goal_accelerations.reserve(N);

    for (size_t i = 0; i < N; ++i)
    {
      all_ids.push_back(static_cast<uint8_t>(motor_ids_[i]));
      const auto &mc = msg.motor_control[i];
      goal_positions.push_back(mc.goal_position);
      goal_velocities.push_back(mc.profile_velocity);
      goal_accelerations.push_back(mc.profile_acceleration);
    }

    RCLCPP_INFO(this->get_logger(), "Sending packet with %zu motors", all_ids.size());
    motor_setting_handler_->sendMotorPacket(all_ids, goal_positions, goal_velocities, goal_accelerations);
  }

  void dynamixel_rdk_ros2::dynamixel_callback(const dynamixel_rdk_msgs::msg::DynamixelMsgs &msg)
  {
    RCLCPP_INFO(this->get_logger(), "=== Pan/Tilt Control ===");

    if (!motor_setting_handler_)
    {
      RCLCPP_ERROR(this->get_logger(), "motor_setting_handler_ is not initialized.");
      return;
    }

    uint8_t pan_motor_id = 23;
    double goal_position = msg.goal_position * M_PI / 180;

    uint32_t position_value = static_cast<uint32_t>((goal_position + M_PI) * (4095.0 / (2 * M_PI)));
    motor_setting_handler_->setGoalPosition(pan_motor_id, position_value);
  }

  // -------------------------------------------------------------- Utility --------------------------------------------------------------
  void dynamixel_rdk_ros2::dxl_variable_init()
  {
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
  }

  void dynamixel_rdk_ros2::ResizeMsg(dynamixel_rdk_msgs::msg::CurrentMotorStatus &msg, size_t size)
  {
    msg.position.resize(size);
    msg.goal_position.resize(size);
    msg.velocity.resize(size);
    msg.temperature.resize(size);
    msg.torque.resize(size);
    msg.input_voltage.resize(size);
    msg.moving_status.resize(size);
    msg.error_status.resize(size);
  }

  void dynamixel_rdk_ros2::msgUpdate(dynamixel_rdk_msgs::msg::CurrentMotorStatus &msg,
                                     size_t index,
                                     uint32_t position, uint32_t goal_position, uint8_t velocity, uint16_t voltage,
                                     uint8_t temperature, uint16_t torque, uint8_t moving_status, uint8_t error_status)
  {
    double position_in_radians = tickToRadian(position);
    double goal_position_in_radians = tickToRadian(goal_position);

    msg.position[index] = position_in_radians;
    msg.goal_position[index] = goal_position_in_radians; // 실제 goal_position 사용
    msg.velocity[index] = velocity;
    msg.input_voltage[index] = voltage;
    msg.temperature[index] = temperature;
    msg.torque[index] = torque;
    msg.moving_status[index] = moving_status;
    msg.error_status[index] = error_status;
  }

  // bool dynamixel_rdk_ros2::errorInterface(uint8_t id)
  // {
  //   uint8_t error_status = 0;

  //   HardwareErrorStatus(id, error_status);

  //   if (error_status & 0b00000001) // 비트 0: 입력 전압 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Input voltage error detected!", id);
  //   }

  //   if (error_status & 0b00000100) // 비트 2: 과열 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Overheating error detected!", id);
  //   }

  //   if (error_status & 0b00001000) // 비트 3: 엔코더 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Encoder error detected!", id);
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Please check the cable connection", id);
  //   }

  //   if (error_status & 0b00010000) // 비트 4: 전기적 충격 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Electrical shock error detected!", id);
  //   }

  //   if (error_status & 0b00100000) // 비트 5: 과부하 오류
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "ID %d: Overload error detected!", id);
  //   }
  //   return true;
  // }

  int32_t dynamixel_rdk_ros2::radianToTick(double rad)
  {
    return static_cast<int32_t>((rad * 4096) / (2 * M_PI));
  }

  double dynamixel_rdk_ros2::tickToRadian(uint32_t position)
  {
    return (static_cast<double>(position) * 2 * M_PI) / 4096.0;
  }

  // -------------------------------------------------------------- 토크 검사 및 재설정 --------------------------------------------------------------
  void dynamixel_rdk_ros2::checkAndSetTorque()
  {
    if (!motor_status_handler_ || !motor_setting_handler_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "토크 상태 검사 중...");

    for (auto id : connected_motor_ids_) {
      uint8_t motor_id = static_cast<uint8_t>(id);
      uint8_t torque_status = 0;
      uint8_t dxl_error = 0;

      // 직접 토크 enable 상태를 읽기
      int dxl_comm_result = packet_handler_->read1ByteTxRx(
        port_handler_, motor_id, 64, &torque_status, &dxl_error); // 64: TORQUE_ENABLE 주소

      if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
        if (torque_status == 0) {
          RCLCPP_WARN(this->get_logger(), "모터 ID %d 토크가 꺼져있음. 재설정 시도...", id);

          // 토크 재설정
          if (motor_setting_handler_->setTorque(motor_id, true)) {
            RCLCPP_INFO(this->get_logger(), "모터 ID %d 토크 재설정 성공", id);
          } else {
            RCLCPP_ERROR(this->get_logger(), "모터 ID %d 토크 재설정 실패", id);
          }
        } else {
          RCLCPP_DEBUG(this->get_logger(), "모터 ID %d 토크 정상", id);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "모터 ID %d 토크 상태 읽기 실패", id);
      }
    }
  }

} // namespace dynamixel_rdk_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::dynamixel_rdk_ros2>();

  // Multi-threaded executor 사용 - 콜백들이 병렬로 실행됨
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4개 스레드
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
