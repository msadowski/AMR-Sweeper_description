

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    # Specify the name of the package
    package_name = 'AMR-Sweeper_description'
    

    # Launch AMR-Sweeper
    AMRSweeper_sim_launch = IncludeLaunchDescription(           #note AMR-Sweeper name without "-" due to python
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'AMR-Sweeper_description.launch.py')]), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    #Path to world file[spawner_joint_broad]: waiting for service /controller_manager/list_controllers to become available...
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )


    # Include Gazebo launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args':['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
   )


    # Spawn Sweepe in Gazebo
    spawn_AMRSweeper_sim = Node(                                #note AMR-Sweeper name without "-" due to python
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'AMR-Sweeper_description', '-z', '0.13'],
        output='screen'
    )


    # Configure the differential driver controller manager spawner node
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"]
    )


    # Configure the joint broadcaster spawner node
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"]
    )


    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    # Run all
    return LaunchDescription([
        AMRSweeper_sim_launch,                      #note AMR-Sweeper name without "-"
        twist_mux,
        world_arg,
        gz_sim,
        spawn_AMRSweeper_sim,                       #note AMR-Sweeper name without "-"
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
