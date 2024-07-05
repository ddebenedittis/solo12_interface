import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.events.lifecycle import  *
from launch.substitutions import Command,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #get solo12 urdf with system HW interface 
    solo_robot_path = get_package_share_path("solo_robot_description")
    ref_gen_path = get_package_share_path("reference_generator")
    ref_gen_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(ref_gen_path,"launch","ref_generator.launch.py")
        )
    )
    solo_robot_path = os.path.join(solo_robot_path,"urdf/again/solo12.urdf.xacro") 
    solo12_model = DeclareLaunchArgument(
        name="solo12_urdf",
        default_value=str(solo_robot_path)
    )
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("solo12_urdf")]),
        value_type=str
    )

    #get controller config file
    controller_path = get_package_share_path("solo12_sim")

    controller_path = os.path.join(controller_path,'config','leg_control.yaml')
    controller_param = ParameterValue(controller_path,value_type=str)
    
    print(controller_path)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
        controller_path],
        
    )

    load_cont_lh = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'leg_forward_position_controller'],
        output='screen'
    )


    return LaunchDescription(
        [
           solo12_model,
           #robot_state_pub_node,
            control_node
        ]
    )