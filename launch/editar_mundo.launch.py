from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.world',
        description='Nome do arquivo .world do mundo a ser carregado'
    )

    world_file = LaunchConfiguration('world')

    pkg_share = FindPackageShare("capote").find("capote")

    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=os.path.join(pkg_share, "models")
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": world_path}.items(),
    )
    
    return LaunchDescription([
        declare_world_arg,
        set_gazebo_model_path,
        gazebo
    ])
