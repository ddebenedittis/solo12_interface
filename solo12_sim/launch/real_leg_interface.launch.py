import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #get solo12 urdf with system HW interface 
    solo_robot_path = get_package_share_path("solo_description")
    solo_robot_path = os.path.join(solo_robot_path,"xacro/solo12.urdf.xacro")
    solo12_model = DeclareLaunchArgument(
        name="solo12_urdf", 
        default_value=str(solo_robot_path)
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("solo12_urdf"), ' sim:=False']),
        value_type=str
    )

    #get controller config file
    controller_path = get_package_share_path("solo12_sim")

    controller_path = os.path.join(controller_path,'config','base_config.yaml')
    controller_param = ParameterValue(controller_path,value_type=str)
    
    print(controller_path)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
        controller_path],
    )

    return LaunchDescription(
        [
            solo12_model,
            #robot_state_pub_node,
            control_node, 
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #     target_action=control_node,
            #     on_exit=[load_cont_sc_plan],)
            # ),
        ]
    )