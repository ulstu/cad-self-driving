#!/usr/bin/env python
import os
import pathlib
import launch
import yaml
import xacro
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from nav2_common.launch import RewrittenYaml

PACKAGE_NAME = 'webots_ros2_suv'
USE_SIM_TIME = True

def get_ros2_nodes(*args):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    lane_follower = Node(
        package=PACKAGE_NAME,
        executable='lane_follower',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_sensors_webots = Node(
        package=PACKAGE_NAME,
        executable='node_sensors_webots',
        name='node_sensors_webots',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_localmap = Node(
        package=PACKAGE_NAME,
        executable='node_localmap',
        name='node_localmap',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    pcl_map_node = Node(
        package="pcl_maps",
        executable='pcl_map_node',
        name='pcl_map_node',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_globalmap = Node(
        package=PACKAGE_NAME,
        executable='node_globalmap',
        name='node_globalmap',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_path_controller = Node(
        package=PACKAGE_NAME,
        executable='node_path_controller',
        name='node_path_controller',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

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

    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(pkg_share, 'config/config.rviz')]]
        )

    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('/depth_camera_info', '/vehicle/range_finder/camera_info'),
            ('/depth', '/vehicle/range_finder'),
        ],
        parameters=[{
            #'use_sim_time': use_sim_time,
            'range_min': 1.0,
            'range_max': 40.0,
            'scan_height': 10, # если h - высота изображения, то этот параметр задает, область видимости по горизонтали высотой scan_height
            'output_frame': 'base_link',
            'scan_time': 0.033
            }]
    )

    return [
        state_publisher_node,
        node_sensors_webots,
        node_localmap,
        node_globalmap,
        node_path_controller,
        #depth_to_laserscan,
        # pcl_map_node,
        #rviz2_node,
        #lane_follower,
    ] + static_transform_nodes


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]), 
                            ros2_supervisor=False,
                            host_ip="host.docker.internal",)
    robot_description_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf')))
    vehicle_driver = WebotsController(
        robot_name='vehicle',
        ip_address="host.docker.internal",
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True}
        ],
        respawn=True
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='robocross.wbt',
            description='Robocross simulation world'
        ),
        webots,
        webots._supervisor,
        vehicle_driver,
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
             target_action=webots,
             on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ] + get_ros2_nodes())
