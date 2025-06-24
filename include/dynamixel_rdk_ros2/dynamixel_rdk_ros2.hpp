#ifndef DYNAMIXEL_RDK_ROS2_HPP_
#define DYNAMIXEL_RDK_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_rdk_ros2/control_table.hpp"
#include "dynamixel_rdk_ros2/base_setting_node.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/current_motor_status.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/warning_status.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/dynamixel_msgs.hpp"


// Dynamixel MX Series Profile
#define MX_RPS_PROFILE 0.229     // [rev/min]
#define MX_ACC_PROFILE 214.577   // [rev/min^2]
#define MX_CURRENT_PROFILE 3.36  // [mA]

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
  class dynamixel_rdk_ros2 : public rclcpp::Node
  {
  public:
    // 구조체 정의
    struct MotorKinematics
    {
      int id;
      uint8_t position;
      double velocity;
    };

    struct MotorSettings
    {
      uint8_t min_position_limit;
      uint8_t max_position_limit;

      uint8_t max_velocity_limit;
      uint8_t max_acceleration_limit;

      uint8_t temperature_limit;
      uint8_t current_limit;
      uint8_t pwm_limit;
      uint8_t shutdown;
    };
    // 모터 설정
    std::vector<MotorSettings> motors_setting;

    bool SendMotorPacket();


    // 생성자 및 소멸자
    dynamixel_rdk_ros2();
    virtual ~dynamixel_rdk_ros2();

    // 초기화 함수들
    void initParameters();
    bool initDynamixel();

    // 모터 제어 함수들
    bool setTorque(uint8_t id, bool enable);
    bool setupMotor(uint8_t id, uint8_t change_set_mode, uint8_t change_set_value);
    void setupTorqueJudgment(bool enable);
    bool setOperatingMode(uint8_t id, uint8_t mode);

    // 모터 상태 확인 함수들
    bool motorCheck();
    bool pingMotor(int id);

    // public 멤버 변수들
    uint8_t dxl_error;
    int dxl_comm_result;
    std::vector<MotorKinematics> motors;
    double dxl_rps_ratio, dxl_acc_ratio, dxl_current_ratio;

  protected:
    // 파라미터들
    std::string device_port_;
    int baud_rate_;
    float protocol_version_;
    std::vector<int> motor_ids_;
    std::vector<int> connected_motor_ids_;
    std::vector<int> disconnected_motor_ids_;

    // 통신 핸들러들
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

  private:
    // private 멤버 변수들
    // std::vector<double> max_position_limits_;
    // std::vector<double> min_position_limits_;

    // 템플릿
    template<typename T>
    bool TxRx(uint8_t id, const std::pair<int, int> &control_table_adress, T &value, const std::string &status_name);

    template<typename D>
    bool SyncRead(const std::pair<int, int> &control_table_adress, D &status_save, const std::string &status_name, int batch_size = 5);



    // Publisher들
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::CurrentMotorStatus>::SharedPtr motor_status_pubisher_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::WarningStatus>::SharedPtr warning_status_publisher_;

    // Subscriber들
    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs>::SharedPtr ik2rdk_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr getting_timer_;

    // 모터 상태 읽기 함수들 (Getter)
    bool getCurrentPosition(uint8_t id, uint32_t &position);
    bool getGoalPosition(uint8_t id, uint32_t &goal_position);
    bool getCurrentVelocity(uint8_t id, uint8_t &velocity);
    bool getInputVoltage(uint8_t id, uint16_t &voltage);
    bool getCurrentTemperature(uint8_t id, uint8_t &temperature);
    bool getCurrentTorque(uint8_t id, uint16_t &torque);
    bool getMovingStatus(uint8_t id, uint8_t &moving_status);
    bool HardwareErrorStatus(uint8_t id, uint8_t &error_status);

    // 모터 상태 읽기 함수들(Getter) - Sync
    bool getCurrentPositionSync(uint32_t &position);
    bool getGoalPositionSync(uint32_t &goal_position);
    bool getCurrentVelocitySync(uint8_t &velocity);
    bool getInputVoltageSync(uint16_t &voltage);
    bool getCurrentTemperatureSync(uint8_t &temperature);
    bool getCurrentTorqueSync(uint16_t &torque);
    bool getMovingStatusSync(uint8_t &moving_status);
    bool HardwareErrorStatusSync(uint8_t &error_status);

    // 모터 기본값 설정 함수들(Setter)
    bool setMinPositionLimit();
    bool setMaxPositionLimit();
    bool setMaxVelocityLimit();
    bool setMaxAccelerationLimit();
    bool setTemperatureLimit();
    bool setCurrentLimit();
    bool setPwmLimit();
    bool setShutdown();
    // double convertPositionToRadian(int position, int min = 0, int max = 4095);

    // 모터 컨트롤 함수들
    bool setGoalPosition(uint8_t id, uint32_t position);
    bool setGoalCurrent(uint8_t id, uint16_t current);
    bool setGoalPositionBulk();
    bool switchTocurrentMode(uint8_t id);

    struct MotorControl
    {
      uint8_t id;
      uint32_t goal_position;
      uint32_t goal_velocity;
      uint32_t goal_acceleration;
    };

    std::vector<MotorControl> motor_controls_;

    // 모터 기본값 변경 함수들
    bool sendMotorPacket(std::vector<double> position, std::vector<double> velocity, std::vector<double> acceleration);

    // 설정 변경 함수
    bool DefaultSettingChange(uint8_t change_set_mode, int8_t change_set_value_arr[]);

    // 콜백 및 유틸리티 함수들
    void timer_callback();
    bool errorInterface(uint8_t id);
    void dynamixel_control_callback(const dynamixel_sdk_custom_interfaces::msg::DynamixelControlMsgs & msg);

    int posToRadian(int position)
    {
      return (static_cast<int>(position)) * (2.0 * M_PI / 4095.0);
    }

    int velToRadian(int velocity)
    {
      return (velocity / dxl_rps_ratio);
    }

    int accToRadian(int acceleration)
    {
      return (acceleration / dxl_acc_ratio);
    }

  };
}

#endif // DYNAMIXEL_RDK_ROS2_HPP_
