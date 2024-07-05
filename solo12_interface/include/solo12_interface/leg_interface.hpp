#ifndef LEG_SYSTEM_INTERFACE_HPP
#define LEG_SYSTEM_INTERFACE_HPP
#include <memory>
#include <string>
#include <vector>

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


#define N_JNT   3
#define N_INT   3


#include "master_board_sdk/master_board_interface.h"

namespace solo12_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class SOLO12_Leg_Interface : public hardware_interface::SystemInterface
    {
        public:
            //manage the allocation of MasterBoardInterface
            SOLO12_Leg_Interface();

            ~SOLO12_Leg_Interface();
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

            uint8_t timeout_;
            uint8_t command_type_;
            MasterBoardInterface master_board_interface_;
            double kp_,kv_,i_sat_;

            double state_[3][3];
            int jnt_motor_[3];
            std::vector<double> command_;

            std::string interface_type_[N_INT];
            std::string joint_name_[N_JNT];

            int cont_;

            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> publisher_;



    };
}
#endif