#include "solo12_interface/leg_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#define CONNECTION_MODE "enx34298f70270f"
#define ERROR_MAX 20
#define TIMEOUT_PARAM "timeout"
#define COMMAND_TYPE_PARAM "command_type"
#define KP_PARAM "Kp"
#define KV_PARAM "Kv"
#define CUR_SAT_PARAM "I_saturation"

#define LOGGER_NAME "SOLO12_Leg_System_Interface"
#define MOTOR_NUM 1
// define to map real driver with software structure
/*#define JNT_MOTOR_HAA 7
#define JNT_MOTOR_HFE 9
#define JNT_MOTOR_HKE 8*/
#define JNT_MOTOR_HAA 9
#define JNT_MOTOR_HFE 7
#define JNT_MOTOR_HKE 11




#define TRASMISSION_RATEO  9
#define EFFORT_CURR_GAIN 0.025

//diver 1 and 0 has bees exchanged


namespace solo12_interface
{
    SOLO12_Leg_Interface::SOLO12_Leg_Interface():
    master_board_interface_(CONNECTION_MODE)
    {
        
        jnt_motor_[0] = JNT_MOTOR_HAA;
        jnt_motor_[1] = JNT_MOTOR_HFE;
        jnt_motor_[2] = JNT_MOTOR_HKE;

        for( int i = 0; i< 3 ; i++)
        {
            if(jnt_motor_[i] < 0 || jnt_motor_[i] > 11)
             throw std::invalid_argument("assigned invalid argument to MOTOR variable");
        }


    }

    SOLO12_Leg_Interface::~SOLO12_Leg_Interface()
    {
        master_board_interface_.~MasterBoardInterface();
    }

    /*
    In this first test the joint name and state interaface are fixed, 
    user can choose with parameter the timeout value in [0,255] and the
    type of all command interface 

    */

    CallbackReturn SOLO12_Leg_Interface::on_init(const hardware_interface::HardwareInfo & info)
    {
        int timeout,command_type;
        cont_ = 0;
        if(hardware_interface::SystemInterface::on_init(info)!= CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        interface_type_[0] = hardware_interface::HW_IF_POSITION;
        interface_type_[1] = hardware_interface::HW_IF_VELOCITY;
        interface_type_[2] = hardware_interface::HW_IF_EFFORT;

        joint_name_[0] = "HAA";
        joint_name_[1] = "HFE";
        joint_name_[2] = "KFE";

        // Timeout param check
        auto it = info_.hardware_parameters.find(TIMEOUT_PARAM);
        if( it == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "'timeout' parameter is not found"
            );
            return CallbackReturn::ERROR;
        }
        
        timeout = std::stoi( info_.hardware_parameters[TIMEOUT_PARAM]);

        if(timeout >255 || timeout < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "not correct value of 'timeout' it should be in range(0,255) but it's %d"
                ,timeout
            );
            return CallbackReturn::ERROR;
        }
        timeout_ = (uint8_t) timeout;

        //Command Type check
        auto it_1 = info_.hardware_parameters.find(COMMAND_TYPE_PARAM);
        if(it_1 ==info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "'command type' parameter is not found"
            );
            return CallbackReturn::ERROR;       
        }

        command_type = std::stoi(info_.hardware_parameters[COMMAND_TYPE_PARAM]);

        if(command_type > 2 || command_type < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "not correct value of 'command type' it should be in range(0,2) but it's %d",
                command_type
            );
            return CallbackReturn::ERROR;
        }
        
        else if(command_type == 0)
            command_.resize(MOTOR_NUM*2);
        else
            command_.resize(MOTOR_NUM);

        command_type_ = (uint8_t) command_type;
        //Kp param Check
        auto it_2 = info_.hardware_parameters.find(KP_PARAM);
        if(it_2 ==info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "'Kp' parameter is not found"
            );
            return CallbackReturn::ERROR;       
        }

        kp_ = std::stod(info_.hardware_parameters[KP_PARAM]); 
        
        if(kp_ <0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "position error gain shouldn't be negative"
            );
            return CallbackReturn::ERROR;
        }


        //Kv param Check
        auto it_3 = info_.hardware_parameters.find(KV_PARAM);
        if(it_3 ==info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "'Kv' parameter is not found"
            );
            return CallbackReturn::ERROR;       
        }

        kv_ = std::stod(info_.hardware_parameters[KV_PARAM]); 

        if(kv_ < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "velocity error gain shouldn't be negative"
            );
            return CallbackReturn::ERROR;
        }

        //Current_Saturation param Check
        auto it_4 = info_.hardware_parameters.find(CUR_SAT_PARAM);
        if(it_4 ==info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "'Current Saturation' parameter is not found"
            );
            return CallbackReturn::ERROR;       
        }
        i_sat_ = std::stod(info_.hardware_parameters[CUR_SAT_PARAM]); 

        if(i_sat_ > 12.0 || i_sat_ < 0.0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "not correct value of 'current saturation' it should be in range(0,12) but it's %f"
                ,i_sat_
            );
            return CallbackReturn::ERROR;
        }

        master_board_interface_.Init();


        // TODO understand if  put this in on_configure it's better than there
        for(int i=0; i < MOTOR_NUM ;i++)
        {
            master_board_interface_.motor_drivers[jnt_motor_[i]/2].EnablePositionRolloverError();
            master_board_interface_.motor_drivers[jnt_motor_[i]/2].SetTimeout(timeout_);
            master_board_interface_.motor_drivers[jnt_motor_[i]/2].Enable();
        
        }
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Initalization Succsesfully complete"
        );
        //Init variable for all motor driver TODO it must done here ?!?!
        /*
    */
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SOLO12_Leg_Interface::on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start Configuration Procedure"
        );
        while(!master_board_interface_.IsTimeout() && !master_board_interface_.IsAckMsgReceived())
        {
            master_board_interface_.SendInit();
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        if(master_board_interface_.IsTimeout())   
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "Timeout Occur"
            );
            return CallbackReturn::ERROR;
        }
        else
        {
            //search why interface cant create a publisher
            //publisher_ = this->create
            RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Configuration Complete"
            );
            return CallbackReturn::SUCCESS;
        }
    }

    CallbackReturn SOLO12_Leg_Interface::on_activate(const rclcpp_lifecycle::State&)
    {
        bool all_motor_ready = false;
        // change so that use jnt_driver and jnt_motor 
        for(int i=0; i<MOTOR_NUM; i++)
        {
           /* if(i%2 == 0)
            {*/
            if(!master_board_interface_.motor_drivers[jnt_motor_[i]/2].IsConnected())
            {
                RCLCPP_ERROR(
                        rclcpp::get_logger(LOGGER_NAME),
                        "Driver %d is not connected, system will shotdown",
                        jnt_motor_[i]/2
                );
                return CallbackReturn::ERROR;
            }
                
            //}
            if(!master_board_interface_.motors[jnt_motor_[i]].IsEnabled())
            {
                master_board_interface_.motors[jnt_motor_[i]].SetCurrentReference(0);
                master_board_interface_.motors[jnt_motor_[i]].Enable();
                master_board_interface_.motors[jnt_motor_[i]].SetKp(kp_);
                master_board_interface_.motors[jnt_motor_[i]].SetKd(kv_);
                master_board_interface_.motors[jnt_motor_[i]].SetSaturationCurrent(i_sat_);
                RCLCPP_INFO(
                    rclcpp::get_logger(LOGGER_NAME),
                    "Motor %d is enabled",
                    i
                );
            }
        }
        while(all_motor_ready)
        {
            all_motor_ready = true;
            for(int i=0; i<MOTOR_NUM;i++)
            {
                if(!master_board_interface_.motors[jnt_motor_[i]].IsReady())
                    all_motor_ready = false;
               
            }
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            if(master_board_interface_.IsTimeout())
            {
                RCLCPP_ERROR(
                rclcpp::get_logger(LOGGER_NAME),
                "Timeout Occur"
                );
                return CallbackReturn::ERROR;
            }
        }
            //  TODO possibly execute a procedure to initilize the joint variable
        for(uint i=0; i< N_JNT; i++ )
        {
            for(uint j=0; j< N_INT; j++)
            {
                state_[i][j] = 0.0;
            }
        }
        for(uint i = 0; i < command_.size(); i++ )
        {
            command_[i] = 0.0;
                RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "counter : %d", i
          ); 
        }   
        
        
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Complete Activation Procedure"
        );

        return CallbackReturn::SUCCESS;
        
    }


    CallbackReturn SOLO12_Leg_Interface::on_cleanup(const rclcpp_lifecycle::State&)
    {
        master_board_interface_.Stop();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SOLO12_Leg_Interface::on_deactivate(const rclcpp_lifecycle::State&)
    {
        for(int i =0; i< MOTOR_NUM; i++)
            master_board_interface_.motors[jnt_motor_[i]].Disable();
        return CallbackReturn::SUCCESS;

    }
    //TODO manage the case of calling 
    CallbackReturn SOLO12_Leg_Interface::on_shutdown(const rclcpp_lifecycle::State&)
    {
        for(int i =0; i < MOTOR_NUM; i++ )
            master_board_interface_.motor_drivers[jnt_motor_[i]/2].Disable();

        return CallbackReturn::SUCCESS;
    }
    std::vector<hardware_interface::StateInterface> SOLO12_Leg_Interface::export_state_interfaces()
    {
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start export of State interface"
        );
        std::vector<hardware_interface::StateInterface> state_interface;
        int n_jnt = joint_name_->size();
        int n_int = interface_type_->size();
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "number of interface is %d and of joint name is %d",
            n_int,n_jnt
        );
        for(int i=0; i<N_JNT; i++)
        {
            for(int j=0; j<N_INT;j++)
            {
                RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "setup %d joint and %d interface",
            i,j
        );
                state_interface.emplace_back(
                    hardware_interface::StateInterface(
                        joint_name_[i],interface_type_[j],&state_[i][j]
                    )
                );
            }
        }
        RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_NAME),
        "Export of State interface Completed"
    );
        return state_interface;
    }

    std::vector<hardware_interface::CommandInterface> SOLO12_Leg_Interface::export_command_interfaces()
    {
        
        std::vector<hardware_interface::CommandInterface> command_interface;
       RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start export of Command interface"
        );
        int n_jnt = joint_name_->size();
        for(int i=0; i<n_jnt;i++)
        {
            if(command_type_ != 0)
            {
                command_interface.emplace_back(
                    hardware_interface::CommandInterface(
                        joint_name_[i],interface_type_[command_type_],&command_[i]
                    )
                );  
            }
            else
            {
                for(int j=0;j<2;j++)
                {
                    command_interface.emplace_back(
                        hardware_interface::CommandInterface(
                            joint_name_[i],interface_type_[j],&command_[i + n_jnt*j]
                        )
                    ); 
                }
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Export of Command interface Completed"
        );
        return command_interface;
    }

    hardware_interface::return_type SOLO12_Leg_Interface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start read"
        );*/
        uint n_jnt = joint_name_->size();
        int error_code;
        master_board_interface_.ParseSensorData();

        if(master_board_interface_.IsTimeout())
            return hardware_interface::return_type::ERROR;
        for(uint i=0; i<MOTOR_NUM; i++)
        {
            state_[i][0] = master_board_interface_.motors[jnt_motor_[i]].GetPosition()/ (TRASMISSION_RATEO);
            state_[i][1] = master_board_interface_.motors[jnt_motor_[i]].GetVelocity()/ (TRASMISSION_RATEO);
            state_[i][2] = master_board_interface_.motors[jnt_motor_[i]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
            error_code = master_board_interface_.motor_drivers[jnt_motor_[i]/2].error_code;
            if(error_code != 0)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger(LOGGER_NAME),
                    "dirver %d error detect number %d",jnt_motor_[i]/2, error_code
                );
            }
        }
        cont_ ++; 
        if(cont_%1000==0)
        {
            
                RCLCPP_INFO(
                rclcpp::get_logger(LOGGER_NAME),
                "readed value of joints is [%f]",//,%f,%f]",
                state_[0][0]//,state_[1][0],state_[2][0]
            );  

                RCLCPP_INFO(
                rclcpp::get_logger(LOGGER_NAME),
                "readed torque value of joints is [%f,%f]",
                state_[0][2],state_[1][2]
            );  
            cont_ = 0;
        }
         /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Read Complete"
        );*/
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SOLO12_Leg_Interface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        int n_jnt = joint_name_->size();
        /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start write"
        );*/
        if(master_board_interface_.IsTimeout())
            return hardware_interface::return_type::ERROR;
        if(cont_%1000==0)
        {
            
                RCLCPP_INFO(
                rclcpp::get_logger(LOGGER_NAME),
                "write value of joints is [%f,%f,%f]",
                command_[0],command_[1],command_[2]
            );  
        }
        for(int i=0; i<MOTOR_NUM; i++)
        {
            if(command_type_ == 0)
            {
               /*RCLCPP_INFO(
                            rclcpp::get_logger(LOGGER_NAME),
                            "Command variable for joint %d is %f",
                            i,command_[i]
                        );*/
                master_board_interface_.motors[jnt_motor_[i]].SetPositionReference(command_[i] * (TRASMISSION_RATEO));

                     /*   RCLCPP_INFO(
                            rclcpp::get_logger(LOGGER_NAME),
                            "write case 0"
                        );*/
            }
            else if(command_type_ == 1)
                master_board_interface_.motors[jnt_motor_[i]].SetVelocityReference(command_[i] * (TRASMISSION_RATEO));
            else
                master_board_interface_.motors[jnt_motor_[i]].SetCurrentReference(command_[i] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));

        }
        
            master_board_interface_.SendCommand();
           /* RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Write Complete"
        );*/
        return hardware_interface::return_type::OK;
    }
}
PLUGINLIB_EXPORT_CLASS(
  solo12_interface::SOLO12_Leg_Interface, hardware_interface::SystemInterface)