import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    solo_controller_path = get_package_share_path("solo12_sim")

    broadcaster_path = get_package_share_path("quadruped_info_broadcaster")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(solo_controller_path,"launch","control_step_0.launch.py")
        )
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
            gazebo_launch,
            broadcaster_launch,


            
        ]
    )