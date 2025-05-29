#ifndef BASE_SETTING_NODE_HPP_
#define BASE_SETTING_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_rdk_ros2/control_table.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/warning_status.hpp"

namespace dynamixel_rdk_ros2
{

  class BaseSettingNode : public rclcpp::Node
  {
  public:
    BaseSettingNode(const std::string &node_name, const rclcpp::NodeOptions &options);
    virtual ~BaseSettingNode();

    // 파라미터 초기화 함수
    void initParameters();

    // Dynamixel 초기화 함수
    bool initDynamixel();

    // 모터 토크 활성화/비활성화 함수
    bool setTorque(uint8_t id, bool enable);

    // 모터 설정 함수
    bool setupMotor(uint8_t id, uint8_t change_set_mode, uint8_t change_set_value);

    void setupTorqueJudgment(bool enable);

    // 모터 작동 모드 설정 함수
    bool setOperatingMode(uint8_t id, uint8_t mode);

    bool motorCheck();

    bool pingMotor(int id);

    uint8_t dxl_error;
    int dxl_comm_result;

    struct MotorKinematics
    {
      int id;
      uint8_t position;
      double velocity;
      double acceleration;
    };
    std::vector<MotorKinematics> motors;

  protected:
    std::vector<uint8_t> scanConnectedMotors();

    // 파라미터
    std::string device_port_;
    int baud_rate_;
    float protocol_version_;
    std::vector<int> motor_ids_;
    std::vector<int> connected_motor_ids_;
    std::vector<int> disconnected_motor_ids_;

    // 통신 핸들러
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

  private:
    std::vector<double> max_position_limits_;
    std::vector<double> min_position_limits_;

    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::WarningStatus>::SharedPtr B2C_pub_;
  };

} // namespace dynamixel_rdk_ros2

#endif // BASE_SETTING_NODE_HPP_
