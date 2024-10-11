

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    # Specify the name of the package
    package_name = 'AMR-Sweeper_description'
    

    # Launch AMR-Sweeper
    AMRSweeper_sim_launch = IncludeLaunchDescription(           #note AMR-Sweeper name without "-" due to python
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'AMR-Sweeper_description.launch.py')]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )


    # Spawn Sweepe in Gazebo
    spawn_AMRSweeper_sim = Node(                                #note AMR-Sweeper name without "-" due to python
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'AMR-Sweeper_description'],
        output='screen'
    )


    # Configure the differential driver controller manager spawner node
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=["diff_cont"]
    )


    # Configure the joint broadcaster spawner node
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=["joint_broad"]
    )


    # Run all
    return LaunchDescription([
        AMRSweeper_sim_launch,                      #note AMR-Sweeper name without "-"
        gazebo,
        spawn_AMRSweeper_sim,                       #note AMR-Sweeper name without "-"
        diff_drive_spawner,
        joint_broad_spawner
    ])
