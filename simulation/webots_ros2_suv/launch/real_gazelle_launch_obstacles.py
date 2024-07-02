#!/usr/bin/env python
import os
import pathlib
import launch
import yaml
from launch_ros.actions import LifecycleNode
import xacro
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

PACKAGE_NAME = 'webots_ros2_suv'
USE_SIM_TIME = True
CONFIG_DIRECTORY = "obstacles"

def get_ros2_nodes(*args):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    node_sensors_gazelle = Node(
        package=PACKAGE_NAME,
        executable='node_sensors_gazelle',
        name='node_sensors_gazelle',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_ego_controller = Node(
        package=PACKAGE_NAME,
        executable='node_ego_controller',
        name='node_ego_controller',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}],
        # remappings=[
        #     ('/ch64w/lslidar_point_cloud', '/lidar'),
        # ]
    )

    node_visual = Node(
        package=PACKAGE_NAME,
        executable='node_visual',
        name='node_visual',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )
    node_drive_gazelle = Node(
        package=PACKAGE_NAME,
        executable='node_drive_gazelle',
        name='node_drive_gazelle',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )
    node_point_obstacles = Node(
        package=PACKAGE_NAME,
        executable='node_point_obstacles',
        name='node_point_obstacles',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_bev_builder = Node(
        package=PACKAGE_NAME,
        executable='node_bev_builder',
        name='node_bev_builder',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )
    package = get_package_share_directory("webots_ros2_suv")
    pcl_map_node = Node(
        package="pcl_maps",
        executable='pcl_map_node',
        name='pcl_map_node',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME, "config_dir": os.path.join(package, "config", CONFIG_DIRECTORY)}]
    )
    
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_ch64w.yaml')
    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar_ch64w.rviz')

    lidar_driver_node = LifecycleNode(package='lslidar_driver',
                                namespace='ch64w',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir],
                                )
    rviz_node = Node(
        package='rviz2',
        namespace='ch64w',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf = os.path.join(
        package_dir,
        os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf'))))
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': USE_SIM_TIME, 'robot_description': robot_desc}],
            arguments=[urdf])

    static_transforms = [
        ["base_link", "imu_link"],
        ["base_link", "lidar"],
        ["base_link", "laser_link"],
        # ["base_link", "zed_camera_link"],
        ["map", "odom"],
        ["base_link", "range_finder"]
    ]
    static_transform_nodes = []
    for s in static_transforms:
        static_transform_nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0"] + s,
            parameters=[{'use_sim_time': USE_SIM_TIME}]
        ))
    # static_transform_nodes.append(Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen" ,
    #         arguments=["0", "0", "0", "0", "0", "0", "laser_data_frame", "odom"],
    #         parameters=[{'use_sim_time': USE_SIM_TIME}]
    # ))
    # static_transform_nodes.append(Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen" ,
    #         arguments=["0", "0", "0", "0", "0", "0", "laser_data_frame", "odom"],
    #         parameters=[{'use_sim_time': USE_SIM_TIME}]
    # ))

    return [
        state_publisher_node,
        node_sensors_gazelle,
        node_ego_controller,
        lidar_driver_node,
        node_drive_gazelle,
        # node_visual,
        # rviz_node,
        # node_bev_builder,
        node_point_obstacles,
        pcl_map_node
    ] + static_transform_nodes

def generate_launch_description():
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2'
        }.items()
    )
    ouster_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ros2_ouster'),
            '/launch/driver_launch.py'
        ])
    )
    os.environ['CONFIG_DIRECTORY'] = CONFIG_DIRECTORY
    return LaunchDescription([
        zed_wrapper_launch,
        ouster_wrapper_launch,
    ] + get_ros2_nodes())
