import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
def generate_launch_description():

    broadcaster_path = get_package_share_path("quadruped_info_broadcaster")
    solo_desc_path = get_package_share_path("solo_robot_description")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(solo_desc_path,"launch","gazebo_display_solo.launch.py")
        )
    )
    xacro_file = os.path.join(solo_desc_path,"urdf/again/solo12.urdf.xacro") 
    # gazebo = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #             get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #         )
   
    doc = xacro.parse(open(xacro_file))
    
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'solo',
                                   '-z','0.33', '-x', '0.225'
                                   ],
                        output='screen')
            

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
  
    load_cont_pd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'PD_control'],
        output='screen'
    )
    
    load_cont_sc_plan = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active',
             'SC_Planner'],
        output='screen'
    )

    broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(broadcaster_path,"launch","gazebo_quadruped_info_broadcaster.launch.py")
        ),
        launch_arguments={
            "period":"1000"
        }.items()
    )

    return LaunchDescription(
        [
            gazebo,
            node_robot_state_publisher,
            
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
       
                RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_pd],
            )
        ), RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[broadcaster_launch],
            )
        ), RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_cont_sc_plan],
            )
        ),
        spawn_entity
        ]
    )
