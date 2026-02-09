
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # PARAMS
    gazebo_pkg_name = "gazebo_ros"
    package_name = "robot_description"
    models_pkg = "env_models"
    robot_name = "rosbot_xl"
    world_file = "generated_world.world"

    # Pose where we want to spawn the robot
    spawn_x_val = "0"
    spawn_y_val = "0"
    spawn_z_val = "0"
    spawn_yaw_val = "0.0"

    # Paths
    pkg_description = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    pkg_models = FindPackageShare(package=models_pkg).find(models_pkg)
    world_path = os.path.join(pkg_models, "worlds", world_file)
    xacro_file_path = os.path.join(pkg_description, "urdf/" + robot_name + "/" + robot_name + ".urdf.xacro")
    gazebo_models_path = os.path.join(pkg_models, "models")
    yaml_path = os.path.join(pkg_description, 'config', 'rosbot_xl.yaml')


    # GAZEBO CLASSIC
    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments=[("world", world_path), ("verbose", "true"), ("paused", "false")]
    )

    # Spawn Robot
    spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', robot_name,
                '-topic', 'robot_description',
                '-timeout', '120.0',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file_path]), 'use_sim_time': use_sim_time}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Set environment variables for Gazebo paths
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(gazebo_plugins_name)
    install_dir = get_package_prefix(package_name)

    os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + install_dir + "/share" + ':' + gazebo_models_path
    os.environ['GAZEBO_PLUGIN_PATH'] = os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + '/lib' + ':'

    return LaunchDescription([
        start_sim,
        robot_state_publisher,
        spawn_robot,
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='If true, use simulated clock'),
    ])
           