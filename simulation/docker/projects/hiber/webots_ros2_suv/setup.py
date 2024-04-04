"""webots_ros2 package setup file."""

from setuptools import setup
import os


def generate_data_files(share_path, dir):
    data_files = []
    for path, _, files in os.walk(dir):
        list_entry = (os.path.dirname(os.path.dirname(share_path)) + '/' + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

package_name = 'webots_ros2_suv'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files += generate_data_files('share/' + package_name + '/static/', 'map-server/dist/')
data_files += generate_data_files('share/' + package_name + '/launch/', 'launch/')
data_files += generate_data_files('share/' + package_name + '/worlds/', 'worlds/')
data_files += generate_data_files('share/' + package_name + '/resource/', 'resource/')
data_files += generate_data_files('share/' + package_name + '/config/', 'config/')
data_files += generate_data_files('share/' + package_name + '/maps/', 'maps/')
data_files += generate_data_files('share/' + package_name + '/map-server/', 'map-server/')

setup(
    name=package_name,
    version='2023.0.1',
    packages=[package_name, "webots_ros2_suv/lib", "webots_ros2_suv/states", "webots_ros2_suv/states/robocross", "webots_ros2_suv/workers"],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='SUV ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = webots_ros2_suv.lane_follower:main',
            'field_follower = webots_ros2_suv.field_follower:main',
            'node_sensors_webots = webots_ros2_suv.node_sensors_webots:main',
            'node_localmap = webots_ros2_suv.node_localmap:main',
            'node_ego_controller = webots_ros2_suv.node_ego_controller:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
