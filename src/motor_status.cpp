#include "dynamixel_rdk_ros2/motor_status.hpp"

namespace dynamixel_rdk_ros2
{
    MotorStatus::MotorStatus(dynamixel::PortHandler *port_handler,
                             dynamixel::PacketHandler *packet_handler,
                             rclcpp::Logger logger) : port_handler_(port_handler), packet_handler_(packet_handler), logger_(logger)
    { dxl_variable_init(); }

    MotorStatus::~MotorStatus() {}

    void MotorStatus::dxl_variable_init()
    {
        dxl_error_ = 0;
        dxl_comm_result_ = COMM_TX_FAIL;
    }

    /*=================================================== MOTOR STATUS GETTERS(Individual) ===================================================*/
    bool MotorStatus::getCurrentPosition(uint8_t id, uint32_t &position)
    { return TxRx(id, MXRAM::PRESENT_POSITION, position, "Current Position", READ); }

    bool MotorStatus::getGoalPosition(uint8_t id, uint32_t &goal_position)
    { return TxRx(id, MXRAM::GOAL_POSITION, goal_position, "Goal Position", READ); }

    bool MotorStatus::getCurrentVelocity(uint8_t id, uint8_t &velocity)
    { return TxRx(id, MXRAM::PRESENT_VELOCITY, velocity, "Current Velocity", READ); }

    bool MotorStatus::getInputVoltage(uint8_t id, uint16_t &voltage)
    { return TxRx(id, MXRAM::PRESENT_INPUT_VOLTAGE, voltage, "Input Voltage", READ); }

    bool MotorStatus::getCurrentTemperature(uint8_t id, uint8_t &temperature)
    { return TxRx(id, MXRAM::PRESENT_TEMPERATURE, temperature, "Current Temperature", READ); }

    bool MotorStatus::getCurrentTorque(uint8_t id, uint16_t &torque)
    { return TxRx(id, MXRAM::PRESENT_CURRENT, torque, "Current Torque", READ); }

    bool MotorStatus::getMovingStatus(uint8_t id, uint8_t &moving_status)
    { return TxRx(id, MXRAM::MOVING_STATUS, moving_status, "Moving Status", READ); }

    bool MotorStatus::HardwareErrorStatus(uint8_t id, uint8_t &error_status)
    { return TxRx(id, MXRAM::HARDWARE_ERROR_STATUS, error_status, "Hardware Error Status", READ); }


    /*===================================================== MOTOR STATUS GETTERS(Sync) =====================================================*/
    bool MotorStatus::getCurrentPositionSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::PRESENT_POSITION, motor_ids, status_values, "Current Position"); }

    bool MotorStatus::getGoalPositionSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::GOAL_POSITION, motor_ids, status_values, "Goal Position"); }

    bool MotorStatus::getCurrentVelocitySync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::PRESENT_VELOCITY, motor_ids, status_values, "Current Velocity"); }

    bool MotorStatus::getInputVoltageSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::PRESENT_INPUT_VOLTAGE, motor_ids, status_values, "Input Voltage"); }

    bool MotorStatus::getCurrentTemperatureSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::PRESENT_TEMPERATURE, motor_ids, status_values, "Current Temperature"); }

    bool MotorStatus::getCurrentTorqueSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::PRESENT_CURRENT, motor_ids, status_values, "Current Torque"); }

    bool MotorStatus::getMovingStatusSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::MOVING_STATUS, motor_ids, status_values, "Moving Status"); }

    bool MotorStatus::HardwareErrorStatusSync(const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values)
    { return SyncRead(MXRAM::HARDWARE_ERROR_STATUS, motor_ids, status_values, "Hardware Error Status"); }


    /*======================================================= COMMUNICATION GETTER =======================================================*/
    template <typename T>
    bool MotorStatus::TxRx(uint8_t id, const std::pair<int, int> &control_table_address, T &value, const std::string &status_name, int mode)
    {
        dxl_variable_init();

        if (mode == READ)
        {
            if constexpr (sizeof(T) == 1)
            {
                dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, control_table_address.first, reinterpret_cast<uint8_t *>(&value), &dxl_error_);
            }
            else if constexpr (sizeof(T) == 2)
            {
                dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, control_table_address.first, reinterpret_cast<uint16_t *>(&value), &dxl_error_);
            }
            else if constexpr (sizeof(T) == 4)
            {
                dxl_comm_result_ = packet_handler_->read4ByteTxRx(port_handler_, id, control_table_address.first, reinterpret_cast<uint32_t *>(&value), &dxl_error_);
            }
            else
            {
                return false;
            }
            if (dxl_comm_result_ != COMM_SUCCESS)
            {
                return false;
            }

            if (dxl_error_ != 0)
            {
                RCLCPP_ERROR(logger_, "[TxRx READ] 다이나믹셀 에러 - %s for ID %d: %s (코드: %d)", status_name.c_str(), id, packet_handler_->getRxPacketError(dxl_error_), dxl_error_);
                return false;
            }

            RCLCPP_INFO(logger_, "[TxRx READ] 성공 - ID %d %s: %d", id, status_name.c_str(), static_cast<int>(value));
            return true;
        }

        return false;
    }

    bool MotorStatus::SyncRead(const std::pair<int, int> &control_table_address, const std::vector<uint8_t> &motor_ids, std::vector<MotorStatusConfig> &status_values, const std::string &status_name)
    {
        dynamixel::GroupSyncRead syncRead(port_handler_, packet_handler_, control_table_address.first, control_table_address.second);

        status_values.resize(motor_ids.size());

        for (size_t i = 0; i < motor_ids.size(); i++)
        {
            bool dxl_addparam_result = syncRead.addParam(motor_ids[i]);

            if (!dxl_addparam_result)
            {
                RCLCPP_ERROR(logger_, "Failed to add parameter to SyncRead for %s, ID %d", status_name.c_str(), motor_ids[i]);
                return false;
            }
        }

        int dxl_comm_result = syncRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(logger_, "SyncRead failed when getting %s: %s", status_name.c_str(), packet_handler_->getTxRxResult(dxl_comm_result));
            return false;
        }

        for (size_t i = 0; i < motor_ids.size(); i++)
        {
            if (syncRead.isAvailable(motor_ids[i], control_table_address.first, control_table_address.second))
            {
                if (status_name == "Current Position")
                    status_values[i].position = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Goal Position")
                    status_values[i].goal_position = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Current Velocity")
                    status_values[i].velocity = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Input Voltage")
                    status_values[i].voltage = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Current Temperature")
                    status_values[i].temperature = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Current Torque")
                    status_values[i].torque = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Moving Status")
                    status_values[i].moving_status = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
                else if (status_name == "Hardware Error Status")
                    status_values[i].error_status = syncRead.getData(motor_ids[i], control_table_address.first, control_table_address.second);
            }
            else
            {
                RCLCPP_ERROR(logger_, "Failed to get %s for ID %d", status_name.c_str(), motor_ids[i]);
                return false;
            }
        }

        return true;
    }

}
