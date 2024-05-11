#!/usr/bin/env python
import os
import pathlib
import launch
import yaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
#from .log_server import LogServer

def get_ros2_nodes(*args):
#    LogServer.set_state('starting')
    package_dir = get_package_share_directory('webots_ros2_suv')
    urdf_filename = pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf'))
    robot_description = urdf_filename.read_text()

    use_sim_time = True#LaunchConfiguration('use_sim_time', default='true') 
    with open(urdf_filename, 'r') as infp:
        robot_desc = infp.read()

    suv_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'vehicle'},
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time,},
        ]
    )

    field_follower = Node(
        package='webots_ros2_suv',
        executable='field_follower',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[robot_description])
    
    pkg_share = FindPackageShare(package='webots_ros2_suv').find('webots_ros2_suv')

    ekf_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}])

    node_sensors_webots = Node(
        package='webots_ros2_suv',
        executable='node_sensors_webots',
        name='node_sensors_webots',
        output='screen' ,
        parameters=[{'use_sim_time': use_sim_time}]
    )


    static_imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        parameters=[{'use_sim_time': use_sim_time}]

    )
    static_lidar_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar"],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    static_map_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

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

    slam_node = Node(
        parameters=[
            os.path.join(pkg_share, 'config/slam_toolbox_mapping.yaml'),
            {"use_sim_time": False}
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_localization",
        output="screen"
    )

    with open(os.path.join(pkg_share, 'config/nav2_mapping.yaml'), 'r') as file:
        configParams = yaml.safe_load(file)#['my_node_name']['ros__parameters']
        print(configParams)
    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config/nav2_mapping.yaml'),
        root_key='',
        param_rewrites={},
        convert_types=True)
    map_file_name = os.path.join(pkg_share, 'maps/suv_world.yaml')
    return [
        field_follower,
        suv_driver,
        robot_state_publisher,
        node_sensors_webots,
        #ekf_robot_localization_cmd,
        static_imu_transform,
        static_lidar_transform,
        static_map_transform,
        rviz2_node,
        depth_to_laserscan,
        #slam_node
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py']),
        #     launch_arguments=configParams.items(),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py']),
        #     launch_arguments={
        #                        'use_sim_time' : use_sim_time,
        #                        'map' : map_file_name,
        #                        'autostart' : 'true',
        #                        'params_file' : [configured_params]}.items(),
        # ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_suv')
    world = LaunchConfiguration('world')
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )
    ros2_supervisor = Ros2SupervisorLauncher()
    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            #default_value='suv_world.wbt',
            default_value='ulstu_field.wbt',
            description='Choose one of the world files from `/webots_ros2_tesla/worlds` directory'
        ),
        webots,
        ros2_supervisor,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        reset_handler
    ] + get_ros2_nodes())
