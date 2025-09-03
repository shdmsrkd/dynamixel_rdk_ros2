#ifndef DYNAMIXEL_RDK_ROS2_HPP_
#define DYNAMIXEL_RDK_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_rdk_ros2/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_rdk_ros2/motor_status.hpp"
#include "dynamixel_rdk_ros2/motor_setting.hpp"

// 내부 메시지 인클루드
#include "dynamixel_rdk_ros2/msg/current_motor_status.hpp"
#include "dynamixel_rdk_ros2/msg/warning_status.hpp"
#include "dynamixel_rdk_ros2/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_ros2/msg/dynamixel_msgs.hpp"

// dynamixel_rdk_msgs 네임스페이스 별칭 생성
namespace dynamixel_rdk_msgs {
namespace msg {
  using CurrentMotorStatus = dynamixel_rdk_ros2::msg::CurrentMotorStatus;
  using WarningStatus = dynamixel_rdk_ros2::msg::WarningStatus;
  using DynamixelControlMsgs = dynamixel_rdk_ros2::msg::DynamixelControlMsgs;
  using DynamixelMsgs = dynamixel_rdk_ros2::msg::DynamixelMsgs;
}
}


namespace dynamixel_rdk_ros2
{
  class dynamixel_rdk_ros2 : public rclcpp::Node
  {
  public:
    // 생성자 및 소멸자
    dynamixel_rdk_ros2();
    virtual ~dynamixel_rdk_ros2();

    // 초기화 함수들
    void initParameters();
    bool initDynamixel();

    // 모터 제어 함수들
    bool setTorque(uint8_t id, bool enable);
    bool setOperatingMode(uint8_t id, uint8_t mode);

    // 모터 상태 확인 함수들
    bool motorCheck();
    bool pingMotor(int id);

    // public 멤버 변수들
    uint8_t dxl_error;
    int dxl_comm_result;
    double dxl_rps_ratio, dxl_acc_ratio, dxl_current_ratio;
    int TOTAL_MOTOR;

  protected:
    // 파라미터들
    std::string device_port_;
    int baud_rate_;
    float protocol_version_;

    std::vector<double> min_position_limits_;
    std::vector<double> max_position_limits_;
    std::vector<int> motor_ids_;
    std::vector<int> connected_motor_ids_;
    std::vector<int> disconnected_motor_ids_;

    // 통신 핸들러들
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

  private:
    // 다른 클래스 핸들러들
    std::unique_ptr<MotorStatus> motor_status_handler_;
    std::unique_ptr<MotorSetting> motor_setting_handler_;

    // Publisher들
    rclcpp::Publisher<dynamixel_rdk_msgs::msg::CurrentMotorStatus>::SharedPtr motor_status_pubisher_;
    rclcpp::Publisher<dynamixel_rdk_msgs::msg::WarningStatus>::SharedPtr warning_status_publisher_;

    // Subscriber들
    rclcpp::Subscription<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>::SharedPtr ik2rdk_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr getting_timer_;

    std::vector<MotorSetting::MotorSettingConfig> motor_settings_;
    std::vector<MotorStatus::MotorStatusConfig> motor_status;
    // 콜백 및 유틸리티 함수들
    void timer_callback();
    void dynamixel_control_callback(const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & msg);
    void dxl_variable_init();
    void ResizeMsg(dynamixel_rdk_msgs::msg::CurrentMotorStatus &msg, size_t size);
    void msgUpdate(dynamixel_rdk_msgs::msg::CurrentMotorStatus &msg, size_t index,
                   uint32_t position, uint8_t velocity, uint16_t voltage,
                   uint8_t temperature, uint16_t torque, uint8_t moving_status, uint8_t error_status);

    int32_t radianToTick(double rad);
    double tickToRadian(uint32_t position);
    bool start();

    // int velToRadian(double velocity)
    // {
    //   int velocity_data;

    //   if (velocity <= 0.0)
    //   {velocity_data = 0;}

    //   return velocity_data;
    // }

    // int accToRadian(double acceleration)
    // {
    //   int acceleration_data;

    //   if (acceleration <= 0.0)
    //   {acceleration_data = 0;}
    //   return acceleration_data;
    // }

  };
}

#endif // DYNAMIXEL_RDK_ROS2_HPP_