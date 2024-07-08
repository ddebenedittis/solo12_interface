import os
 
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    solo_robot_path = get_package_share_path("solo_description")
    solo_robot_path = os.path.join(solo_robot_path,"xacro/solo12.urdf.xacro")
    
    solo12_model = DeclareLaunchArgument(
        name="solo12_urdf", 
        default_value=str(solo_robot_path)
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("solo12_urdf"), ' sim:=False']),
        value_type=str,
    )

    controller_path = get_package_share_path("solo12_sim")
    controller_path = os.path.join(controller_path,'config','base_config.yaml')
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
        controller_path],
    )

    return LaunchDescription([
        solo12_model,
        
        control_node,
    ])