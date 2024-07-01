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
CONFIG_DIRECTORY = "simulator"

local_project_settings_config_path = "config/project_settings.yaml"
package_dir = get_package_share_directory(PACKAGE_NAME)
project_settings_config_path = os.path.join(package_dir, local_project_settings_config_path)
with open(project_settings_config_path, "r") as file:
    project_settings_config = yaml.safe_load(file)

if project_settings_config['use_gpu'] == False:
    os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

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
    package = get_package_share_directory("webots_ros2_suv")
    pcl_map_node = Node(
        package="pcl_maps",
        executable='pcl_map_node',
        name='pcl_map_node',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME, "config_dir": os.path.join(package, "config", CONFIG_DIRECTORY)}]
    )

    node_globalmap = Node(
        package=PACKAGE_NAME,
        executable='node_globalmap',
        name='node_globalmap',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_ego_controller = Node(
        package=PACKAGE_NAME,
        executable='node_ego_controller',
        name='node_ego_controller',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_point_obstacles = Node(
        package=PACKAGE_NAME,
        executable='node_point_obstacles',
        name='node_point_obstacles',
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
        #node_localmap,
        #node_globalmap,
        node_ego_controller,
        node_point_obstacles,
        #depth_to_laserscan,
        pcl_map_node,
        #rviz2_node,
        #lane_follower,
    ] + static_transform_nodes


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]), ros2_supervisor=True, stream=True)
    robot_description_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf')))
    vehicle_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description':     os.environ['CONFIG_DIRECTORY'] = CONFIG_DIRECTORY
robot_description_path}
        ],
        respawn=True
    )
    # package = get_package_share_directory("webots_ros2_suv")
    # with open(f'{package}/config/main.yaml', 'w') as file:
    #     yaml.dump({"current": CONFIG_DIRECTORY}, file, default_flow_style=False)
    os.environ['CONFIG_DIRECTORY'] = CONFIG_DIRECTORY
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
