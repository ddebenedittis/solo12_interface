import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    
    pd_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['PD_control', '--controller-manager', '/controller_manager'],
        emulate_tty=True,
        output='screen',
    )

    joint_state_broadcast_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('solo12_sim'), 'launch', 'interface.launch.py')
            ),
        ),
        
        pd_spawner,
        
        joint_state_broadcast_spawner,
    ])
