import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():

    solo_desc_path = get_package_share_path("solo_robot_description")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(solo_desc_path,"launch","gazebo_display_solo.launch.py")
        )
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'solo',
                                   '-x','0.0',
                                   '-y','0.0',
                                   '-z','0.26'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_cont_lh = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'lh_forward_position_controller'],
        output='screen'
    )
    load_cont_lf = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'lf_forward_position_controller'],
        output='screen'
    )
    load_cont_rh = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'rh_forward_position_controller'],
        output='screen'
    )
    load_cont_rf = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'rf_forward_position_controller'],
        output='screen'
    )
    return LaunchDescription(
        [
            gazebo_launch,
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_lh],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_rh],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_rf],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_lf],
            )
        ),
        spawn_entity
        ]
    )
