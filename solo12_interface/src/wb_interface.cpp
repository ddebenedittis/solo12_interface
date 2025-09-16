#include "solo12_interface/wb_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "../include/solo12_interface/wb_interface.hpp"

#define CONNECTION_MODE "enp2s0"
#define ERROR_MAX 20
#define TIMEOUT_PARAM "timeout"
#define COMMAND_TYPE_PARAM "command_type"
#define KP_PARAM "Kp"
#define KV_PARAM "Kv"
#define CUR_SAT_PARAM "I_saturation"

#define LOGGER_NAME "SOLO12_WB_System_Interface"
#define MOTOR_NUM 3
// define to map real driver with software structure
// ordine plottato 
#define JNT_MOTOR_LF_HAA 1 // 2 -- 2
#define JNT_MOTOR_LF_HFE 5 // 3 -- 1
#define JNT_MOTOR_LF_KFE 2 // 0 -- 0
#define JNT_MOTOR_RF_HAA 0 // 1 -- 3
#define JNT_MOTOR_RF_HFE 4 // 4 -- 5
#define JNT_MOTOR_RF_KFE 3 // 5 -- 4
#define JNT_MOTOR_LH_HAA 7 // 9 -- 7
#define JNT_MOTOR_LH_HFE 11 // 7 -- 9
#define JNT_MOTOR_LH_KFE 9 // 8 -- 8
#define JNT_MOTOR_RH_HAA 6 // 6 -- 6
#define JNT_MOTOR_RH_HFE 8 // 10 -- 11
#define JNT_MOTOR_RH_KFE 10 // 11 -- 10

#define TRASMISSION_RATEO  9
#define EFFORT_CURR_GAIN 0.025

//diver 1 and 0 has bees exchanged


namespace solo12_interface
{
    SOLO12_WB_Interface::SOLO12_WB_Interface():
    master_board_interface_(CONNECTION_MODE)
    {
        
        jnt_motor_[0] = JNT_MOTOR_LF_HAA;
        jnt_motor_[1] = JNT_MOTOR_LF_HFE;
        jnt_motor_[2] = JNT_MOTOR_LF_KFE;

        jnt_motor_[3] = JNT_MOTOR_RF_HAA;
        jnt_motor_[4] = JNT_MOTOR_RF_HFE;
        jnt_motor_[5] = JNT_MOTOR_RF_KFE;
        jnt_motor_[6] = JNT_MOTOR_LH_HAA;
        jnt_motor_[7] = JNT_MOTOR_LH_HFE;
        jnt_motor_[8] = JNT_MOTOR_LH_KFE;
        jnt_motor_[9] = JNT_MOTOR_RH_HAA;
        jnt_motor_[10] = JNT_MOTOR_RH_HFE;
        jnt_motor_[11] = JNT_MOTOR_RH_KFE;

        for( int i = 0; i< N_JNT ; i++)
        {
            if(jnt_motor_[i] < 0 || jnt_motor_[i] > 11)
             throw std::invalid_argument("assigned invalid argument to MOTOR variable");
        }


    }

    SOLO12_WB_Interface::~SOLO12_WB_Interface()
    {
        master_board_interface_.~MasterBoardInterface();
    }

    /*
    In this first test the joint name and state interaface are fixed, 
    user can choose with parameter the timeout value in [0,255] and the
    type of all command interface 

    the interface could be commanded in 3 mode: 
        0: enable only Drivers PD control
        1: enable only External current-torque Control
        2: use both external and driver control

    */

    CallbackReturn SOLO12_WB_Interface::on_init(const hardware_interface::HardwareInfo & info)
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

        for(int i = 0; i< N_LEG; i++)
        {
            for(int j = 0; j < N_JNT_FOR_LEG; j++)
            {
                joint_name_[j + i * N_JNT_FOR_LEG] = legs_[i] + "_" + leg_jnt_[j];
            } 
        }

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
  std::cerr<<" the KP is "<<kp_<< " and the KD is "<<kv_<<std::endl;
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
        else if( command_type == 1)
        {
            kv_ = 0.0;
            kp_ = 0.0;
        }

        command_type_ = (uint8_t) command_type;

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
        //all driver should be configured
        for(int i=0; i < N_JNT/2 ;i++)
        {
            master_board_interface_.motor_drivers[i].EnablePositionRolloverError();
            master_board_interface_.motor_drivers[i].SetTimeout(timeout_);
            master_board_interface_.motor_drivers[i].Enable();
        
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

    CallbackReturn SOLO12_WB_Interface::on_configure(const rclcpp_lifecycle::State&)
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

    CallbackReturn SOLO12_WB_Interface::on_activate(const rclcpp_lifecycle::State&)
    {
        bool all_motor_ready = false;
        // change so that use jnt_driver and jnt_motor 
        for(int i=0; i<N_JNT; i++)
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
                master_board_interface_.motors[jnt_motor_[i]].SetPositionReference(0.0);//init_jnt_[i]);
                master_board_interface_.motors[jnt_motor_[i]].SetVelocityReference(0.0);
                master_board_interface_.motors[jnt_motor_[i]].SetKp(kp_);
                master_board_interface_.motors[jnt_motor_[i]].SetKd(kv_);
                master_board_interface_.motors[jnt_motor_[i]].SetSaturationCurrent(i_sat_);
                master_board_interface_.motors[jnt_motor_[i]].Enable();
                RCLCPP_INFO(
                    rclcpp::get_logger(LOGGER_NAME),
                    "Motor %d is enabled",
                    i
                );
            }
            start_t_ = std::chrono::system_clock::now();
        }
        
        all_motor_ready = true;
        while(all_motor_ready)
        {
            master_board_interface_.ParseSensorData();
            all_motor_ready = false;
            for(int i=0; i<N_JNT;i++)
            {
                if(!(master_board_interface_.motors[jnt_motor_[i]].IsReady() && master_board_interface_.motors[jnt_motor_[i]].IsEnabled()))
                    all_motor_ready = true;
               /* RCLCPP_INFO(
                    rclcpp::get_logger(LOGGER_NAME),
                    "motor %d is %d",jnt_motor_[i],master_board_interface_.motors[jnt_motor_[i]].IsReady()
                );*/
            }
            master_board_interface_.SendCommand();
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
                command_[i][j] = 0.0;
            }
        }
        
        
        
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Complete Activation Procedure"
        );

        return CallbackReturn::SUCCESS;
        
    }


    CallbackReturn SOLO12_WB_Interface::on_cleanup(const rclcpp_lifecycle::State&)
    {
        master_board_interface_.Stop();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SOLO12_WB_Interface::on_deactivate(const rclcpp_lifecycle::State&)
    {
        for(int i =0; i< N_JNT; i++)
            master_board_interface_.motors[jnt_motor_[i]].Disable();
        return CallbackReturn::SUCCESS;

    }
    //TODO manage the case of calling 
    CallbackReturn SOLO12_WB_Interface::on_shutdown(const rclcpp_lifecycle::State&)
    {
        for(int i =0; i < N_JNT; i++ )
            master_board_interface_.motor_drivers[jnt_motor_[i]/2].Disable();

        return CallbackReturn::SUCCESS;
    }
    std::vector<hardware_interface::StateInterface> SOLO12_WB_Interface::export_state_interfaces()
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
            "number of state interface is %d and of joint name is %d",
            n_int,n_jnt
        );
        for(int i=0; i<N_JNT; i++)
        {
            for(int j=0; j<N_INT;j++)
            {
             /*   RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "setup %d joint and %d interface",
            i,j
        );*/
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
        std::string imu_name;
        
        std::string name = "IMU";
            state_interface.emplace_back(name ,"linear_acceleration.x", &IMU_lin_acc_[0]);
            state_interface.emplace_back(name ,"linear_acceleration.y", &IMU_lin_acc_[1]);
            state_interface.emplace_back(name ,"linear_acceleration.z", &IMU_lin_acc_[2]);

            state_interface.emplace_back(name ,"angular_velocity.x", &IMU_ang_vel_[0]);
            state_interface.emplace_back(name ,"angular_velocity.y", &IMU_ang_vel_[1]);
            state_interface.emplace_back(name ,"angular_velocity.z", &IMU_ang_vel_[2]);

            state_interface.emplace_back(name ,"orientation.x", &IMU_ori_[0]);
            state_interface.emplace_back(name ,"orientation.y", &IMU_ori_[1]);
            state_interface.emplace_back(name ,"orientation.z", &IMU_ori_[2]);
            state_interface.emplace_back(name ,"orientation.w", &IMU_ori_[3]);
        return state_interface;
    }

    std::vector<hardware_interface::CommandInterface> SOLO12_WB_Interface::export_command_interfaces()
    {
        
        std::vector<hardware_interface::CommandInterface> command_interface;
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start export of Command interface"
        );

        int n_jnt = joint_name_->size();
        int n_int = interface_type_->size();
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "number of command interface is %d and of joint name is %d",
            n_int,n_jnt
        );
        for(int i=0; i<N_JNT; i++)
        {
            for(int j=0; j<N_INT;j++)
            {
              /*  RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "setup %d joint and %d interface",
            i,j
        );*/
                command_interface.emplace_back(
                    hardware_interface::CommandInterface(
                        joint_name_[i],interface_type_[j],&command_[i][j]
                    )
                );
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Export of Command interface Completed"
        );
        return command_interface;
    }

    hardware_interface::return_type SOLO12_WB_Interface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start read"
        );*/
       // uint n_jnt = joint_name_->size();
        imu_data_t imu_meas;
        master_board_interface_.ParseSensorData();
        imu_meas = master_board_interface_.get_imu_data();

        if(master_board_interface_.IsTimeout())
            return hardware_interface::return_type::ERROR;

        for(int i = 0; i < 3 ; i++)
        {
            IMU_lin_acc_[i] = imu_meas.accelerometer[i];
            IMU_ang_vel_[i] = imu_meas.gyroscope[i];
            
        }
        orient_.setRPY( imu_meas.attitude[0], imu_meas.attitude[1], imu_meas.attitude[2] );
        IMU_ori_[0]=orient_.getX(); 
        IMU_ori_[1]=orient_.getY();
        IMU_ori_[2]=orient_.getZ();
        IMU_ori_[3]=orient_.getW();
        // for(uint i=0; i<N_JNT; i++)
        // {
        //     state_[i][0] = master_board_interface_.motors[jnt_motor_[i]].GetPosition()/ (TRASMISSION_RATEO);
        //     state_[i][1] = master_board_interface_.motors[jnt_motor_[i]].GetVelocity()/ (TRASMISSION_RATEO);
        //     state_[i][2] = master_board_interface_.motors[jnt_motor_[i]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
             
        // }
        for(uint i = 0;i<N_LEG;i++)
        {
            for(uint j=0; j<N_JNT_FOR_LEG; j++)
            {
                if(i%2 == 0)
                {
                    if(j==1 || j == 2)
                    {
                        state_[i*N_JNT_FOR_LEG+j][0] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetPosition()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][1] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetVelocity()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][2] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
                    }
                    else
                    {
                        state_[i*N_JNT_FOR_LEG+j][0] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetPosition()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][1] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetVelocity()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][2] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
                    }
                }
                if(i%2 == 1)
                {
                    if(j==1 || j == 2)
                    {
                        state_[i*N_JNT_FOR_LEG+j][0] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetPosition()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][1] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetVelocity()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][2] = master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
                    }
                    else
                    {
                        state_[i*N_JNT_FOR_LEG+j][0] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetPosition()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][1] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetVelocity()/ (TRASMISSION_RATEO);
                        state_[i*N_JNT_FOR_LEG+j][2] = -master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].GetCurrent()* EFFORT_CURR_GAIN * TRASMISSION_RATEO;
                    }
                }
            }
        }
        cont_ ++; 
        // RCLCPP_INFO(
        //     rclcpp::get_logger(LOGGER_NAME),
        //     "readed value of joints is::\n [%f,%f,%f]\n [%f,%f,%f]\n [%f,%f,%f]\n [%f,%f,%f]\n",
        //     state_[0][0],state_[1][0],state_[2][0],state_[3][0],state_[4][0],state_[5][0],
        //     state_[6][0],state_[7][0],state_[8][0],state_[9][0],state_[10][0],state_[11][0]
        // );  
        // if(cont_%1==0)
        // {
            
             
            
        //     /*   RCLCPP_INFO(
        //         rclcpp::get_logger(LOGGER_NAME),
        //         "readed value of joints is:: [%f]\n",state_[8][0]
        //     ); */
        //     /*
        //         RCLCPP_INFO(
        //             rclcpp::get_logger(LOGGER_NAME),
        //             "sensor packet loss\n value::%f",
        //             ((double)master_board_interface_.GetSensorsLost()/(double)master_board_interface_.GetSensorsSent())*100
        //         );
        //          RCLCPP_INFO(
        //             rclcpp::get_logger(LOGGER_NAME),
        //             "\nSIMU-acc::[%f,%f,%f]\nIMU-gyro::[%f,%f,%f]\nIMU-lin-acc::[%f,%f,%f]\n IMU-rpy::[%f,%f,%f]",
        //             IMU_state_arr_[0][0],IMU_state_arr_[1][0],IMU_state_arr_[2][0],
        //             IMU_state_arr_[0][1],IMU_state_arr_[1][1],IMU_state_arr_[2][1],
        //             IMU_state_arr_[0][2],IMU_state_arr_[1][2],IMU_state_arr_[2][2],
        //             IMU_state_arr_[0][3],IMU_state_arr_[1][3],IMU_state_arr_[2][3]
                    

        //             Motor Orientation 
        //             FL_LEG:
        //                 -HAA: ok 
        //                 -HFE: not ok
        //                 -KFE: not ok
        //             FR_LEG:
        //                 -HAA: not ok 
        //                 -HFE:  ok
        //                 -KFE:  ok
        //             HL_LEG:
        //                 -HAA: ok 
        //                 -HFE: not ok
        //                 -KFE: not ok
        //             HR_LEG:
        //                 -HAA: not ok 
        //                 -HFE:  ok
        //                 -KFE:  ok
        //         );*/
            
        
        //     cont_ = 0;
        // }
         /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Read Complete"
        );*/
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SOLO12_WB_Interface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        //int n_jnt = joint_name_->size();
        /*RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_NAME),
            "Start write"
        );*/
        if(master_board_interface_.IsTimeout())
            return hardware_interface::return_type::ERROR;

        
            
            //     RCLCPP_INFO(
            //     rclcpp::get_logger(LOGGER_NAME),
            //     "write value of joints is [%f,%f,%f]",
            //     command_[3][2],command_[4][2],command_[5][2]/(TRASMISSION_RATEO*EFFORT_CURR_GAIN)
            // );  
         
        // for(int i=0; i<N_JNT; i++)
        // {
            
        //     master_board_interface_.motors[jnt_motor_[i]].SetPositionReference(command_[i][0] * (TRASMISSION_RATEO));
        //     master_board_interface_.motors[jnt_motor_[i]].SetVelocityReference(command_[i][1] * (TRASMISSION_RATEO));
        //     if(command_type_ != 0 )
        //         master_board_interface_.motors[jnt_motor_[i]].SetCurrentReference(command_[i][2] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));
        //     else 
        //         master_board_interface_.motors[jnt_motor_[i]].SetCurrentReference(0.0);
        // }
        // RCLCPP_INFO(
        //     rclcpp::get_logger(LOGGER_NAME),
        //     "pass time out error "
        // );
        for(uint i = 0;i<N_LEG;i++)
        {
            for(uint j=0; j<N_JNT_FOR_LEG; j++)
            {
                if(i%2 == 0)
                {
                    if(j==1 || j == 2)
                    {
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetPositionReference(-command_[i*N_JNT_FOR_LEG+j][0] * (TRASMISSION_RATEO));
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetVelocityReference(-command_[i*N_JNT_FOR_LEG+j][1] * (TRASMISSION_RATEO));
                        if(command_type_ != 0 )
                        {
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(-command_[i*N_JNT_FOR_LEG+j][2] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));
                            // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            // "pass here"
                            // );
                        }
                        else 
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(0.0);
                    }
                    else
                    {
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetPositionReference(command_[i*N_JNT_FOR_LEG+j][0] * (TRASMISSION_RATEO));
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetVelocityReference(command_[i*N_JNT_FOR_LEG+j][1] * (TRASMISSION_RATEO));
                        if(command_type_ != 0 )
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(command_[i*N_JNT_FOR_LEG+j][2] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));
                        else 
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(0.0);
                    }
                }
                if(i%2 == 1)
                {
                    if(j==1 || j == 2)
                    {
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetPositionReference(command_[i*N_JNT_FOR_LEG+j][0] * (TRASMISSION_RATEO));
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetVelocityReference(command_[i*N_JNT_FOR_LEG+j][1] * (TRASMISSION_RATEO));
                        if(command_type_ != 0 )
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(command_[i*N_JNT_FOR_LEG+j][2] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));
                        else 
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(0.0);
                    }
                    else
                    {
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetPositionReference(-command_[i*N_JNT_FOR_LEG+j][0] * (TRASMISSION_RATEO));
                        master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetVelocityReference(-command_[i*N_JNT_FOR_LEG+j][1] * (TRASMISSION_RATEO));
                        if(command_type_ != 0 )
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(-command_[i*N_JNT_FOR_LEG+j][2] /(TRASMISSION_RATEO*EFFORT_CURR_GAIN));
                        else 
                            master_board_interface_.motors[jnt_motor_[i*N_JNT_FOR_LEG+j]].SetCurrentReference(0.0);
                    }
                }
            }
        }

        //  RCLCPP_INFO(
        //     rclcpp::get_logger(LOGGER_NAME),
        //     "pass interface write "
        // );
        // auto now = std::chrono::system_clock::now();
        // auto rel_time = std::chrono::duration_cast<std::chrono::milliseconds> (now-start_t_).count();
        //       RCLCPP_INFO(
        //     rclcpp::get_logger(LOGGER_NAME),
        //     "Update time in seconds %f and error %f\n",(float)rel_time/1000,command_[1][0]-  state_[1][0]
        // );
        
            master_board_interface_.SendCommand();
        //   RCLCPP_INFO(
        //     rclcpp::get_logger(LOGGER_NAME),
        //     "pass interface send command"
        // );
        return hardware_interface::return_type::OK;
    }
}
PLUGINLIB_EXPORT_CLASS(
  solo12_interface::SOLO12_WB_Interface, hardware_interface::SystemInterface)
