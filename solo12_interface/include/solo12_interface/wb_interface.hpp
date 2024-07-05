#ifndef WB_SYSTEM_INTERFACE_HPP
#define WB_SYSTEM_INTERFACE_HPP
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


#define N_JNT   12
#define N_JNT_FOR_LEG 3
#define N_LEG 4
#define N_INT   3
#define IMU_NUM_STT_ 4
#define PI 3.141592


#include "master_board_sdk/master_board_interface.h"

namespace solo12_interface
{

    struct imu_data_str
    {
        float accelerometer[3];
        float gyroscope[3];
        float attitude[3];
        float linear_acceleration[3];
    }; 
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class SOLO12_WB_Interface : public hardware_interface::SystemInterface
    {
        public:
            //manage the allocation of MasterBoardInterface
            SOLO12_WB_Interface();

            ~SOLO12_WB_Interface();
            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& ) override;
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            const std::string leg_jnt_[N_JNT_FOR_LEG] = {"HAA","HFE","KFE"};
            const std::string legs_[N_LEG] = {"LF","RF","LH","RH"};
            const std::string IMU_measures_[IMU_NUM_STT_] = {"body_acceleration", "body_angular_velocity", "nav_acceleration", "RPY_body_nav_frame"};
            const std::string Axis_[3] = {"X","Y","Z"}; 
            const double init_jnt_[N_JNT] = {0, -PI/2, PI, 0, PI/2, -PI, 0, PI/2, -PI, 0, -PI/2, PI};

            const std::string interface_IMU_type_[IMU_NUM_STT_] = {
                hardware_interface::HW_IF_ACCELERATION,
                hardware_interface::HW_IF_VELOCITY,
                hardware_interface::HW_IF_ACCELERATION,
                hardware_interface::HW_IF_POSITION
            };
            
            uint8_t timeout_;
            uint8_t command_type_;
            MasterBoardInterface master_board_interface_;
            double kp_,kv_,i_sat_;

            double state_[N_JNT][N_INT];
            double command_[N_JNT][N_INT];
            imu_data_t IMU_State_;
            double IMU_state_arr_ [3][IMU_NUM_STT_];

            int jnt_motor_[N_JNT];

            
            

            std::string interface_type_[N_INT];
            std::string joint_name_[N_JNT];

            int cont_;

            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> publisher_;
            std::chrono::time_point<std::chrono::system_clock> start_t_;


    };
}
#endif