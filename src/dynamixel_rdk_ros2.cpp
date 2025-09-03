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
  motor_status_pubisher_ = this->create_publisher<dynamixel_rdk_msgs::msg::CurrentMotorStatus>("motor_status", 200);
  warning_status_publisher_ = this->create_publisher<dynamixel_rdk_msgs::msg::WarningStatus>("motor_warning", 200);

  // Subscription
  ik2rdk_sub = create_subscription<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>("dynamixel_control", 10, std::bind(&dynamixel_rdk_ros2::dynamixel_control_callback, this, std::placeholders::_1));

  if(start())
  {
    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!! 초기 설정 성공 !!!!!!!!!!!");
    getting_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                    std::bind(&dynamixel_rdk_ros2::timer_callback, this));
  }
  else
  { RCLCPP_ERROR(this->get_logger(), "!!!!!!!!!!! 초기 설정 실패 !!!!!!!!!!!"); }

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

bool dynamixel_rdk_ros2::start()
{
  initParameters();

  if (!initDynamixel())
  {
    RCLCPP_ERROR(this->get_logger(), "Dynamixel 초기화 실패!");
    return false;
  }

  for (auto id : motor_ids_) { setTorque(id, TORQUEON); }

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
    this->declare_parameter("ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
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

  motorCheck(); // 연결 상태 확인

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
    RCLCPP_INFO(this->get_logger(), "motor_status 벡터 크기를 %zu로 조정했습니다", TOTAL_CONNECT_MOTORS);
  }

  ResizeMsg(msg, TOTAL_CONNECT_MOTORS);

  std::vector<uint8_t> connected_motor_uint8_ids;
  for (auto id : connected_motor_ids_)
  {
    connected_motor_uint8_ids.push_back(static_cast<uint8_t>(id));
  }

  // Synchronous read 실패시 개별로 읽음
  bool sync_success = false;
  try {
    sync_success =
        motor_status_handler_->getCurrentPositionSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getGoalPositionSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getCurrentVelocitySync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getInputVoltageSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getCurrentTemperatureSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getCurrentTorqueSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->getMovingStatusSync(connected_motor_uint8_ids, motor_status) &&
        motor_status_handler_->HardwareErrorStatusSync(connected_motor_uint8_ids, motor_status);
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
        msgUpdate(msg, index, status.position, status.velocity, status.voltage,
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
      msgUpdate(msg, index, motor_status[index].position, motor_status[index].velocity,
                motor_status[index].voltage, motor_status[index].temperature, motor_status[index].torque,
                motor_status[index].moving_status, motor_status[index].error_status);
    }
  }

  RCLCPP_INFO(this->get_logger(), "모터 총 개수 : %zu", motor_ids_.size());
  RCLCPP_INFO(this->get_logger(), "총 연결된 모터 개수 : %zu", TOTAL_CONNECT_MOTORS);


  motor_status_pubisher_->publish(msg);
}
  void dynamixel_rdk_ros2::dynamixel_control_callback(const dynamixel_rdk_msgs::msg::DynamixelControlMsgs &msg)
  {
    RCLCPP_INFO(this->get_logger(), "==========================================================");

    std::vector<double> goal_positions;
    std::vector<double> goal_velocities;
    std::vector<double> goal_accelerations;

    for (auto &motor_control : msg.motor_control)
    {
      goal_positions.push_back(motor_control.goal_position);
      goal_velocities.push_back(motor_control.profile_velocity);
      goal_accelerations.push_back(motor_control.profile_acceleration);
    }

    std::vector<uint8_t> lower_body_ids;
    std::vector<double> lower_body_positions;
    std::vector<double> lower_body_velocities;
    std::vector<double> lower_body_accelerations;

    std::vector<uint8_t> upper_body_ids;
    std::vector<double> upper_body_positions;
    std::vector<double> upper_body_velocities;
    std::vector<double> upper_body_accelerations;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
      uint8_t motor_id = motor_ids_[i];

      if (motor_id <= 15)  // 하체
      {
        lower_body_ids.push_back(motor_id);
        lower_body_positions.push_back(goal_positions[i]);
        lower_body_velocities.push_back(goal_velocities[i]);
        lower_body_accelerations.push_back(goal_accelerations[i]);
      }
      else  // 상체
      {
        upper_body_ids.push_back(motor_id);
        upper_body_positions.push_back(goal_positions[i]);
        upper_body_velocities.push_back(goal_velocities[i]);
        upper_body_accelerations.push_back(goal_accelerations[i]);
      }
    }

    if (!lower_body_ids.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Sending lower body packet with %zu motors", lower_body_ids.size());
      motor_setting_handler_->sendMotorPacket(lower_body_ids, lower_body_positions, lower_body_velocities, lower_body_accelerations);
    }

    if (!upper_body_ids.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Sending upper body packet with %zu motors", upper_body_ids.size());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      motor_setting_handler_->sendMotorPacket(upper_body_ids, upper_body_positions, upper_body_velocities, upper_body_accelerations);
    }
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
                                     uint32_t position, uint8_t velocity, uint16_t voltage,
                                     uint8_t temperature, uint16_t torque, uint8_t moving_status, uint8_t error_status)
  {
    double position_in_radians = tickToRadian(position);

    msg.position[index] = position_in_radians;
    msg.goal_position[index] = position_in_radians;
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

} // namespace dynamixel_rdk_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros2::dynamixel_rdk_ros2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
