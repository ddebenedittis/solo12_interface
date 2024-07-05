import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():
    
    pd_control =  ExecuteProcess( 
        cmd=['ros2 control load_controller PD_control --set-state active'],
    )
    
    joint_state =  ExecuteProcess( 
        cmd=['ros2 control load_controller joint_state_broadcaster --set-state active'],
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('solo12_sim'),
            'launch/real_leg_interface.launch.py',
        ))),
        
        TimerAction(
            period=1.0,
            actions=[pd_control, joint_state]
        )
    ])
