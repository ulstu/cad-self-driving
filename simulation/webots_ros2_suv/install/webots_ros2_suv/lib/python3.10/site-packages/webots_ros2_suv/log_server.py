import rclpy
from rclpy.node import Node
from prometheus_client import Enum
from prometheus_client import Gauge
from prometheus_client import Info
from enum import Enum
import random

class LogServerStatus(Enum):
    STARTED = 1.0
    RUNNING = 2.0
    STOPPED = 3.0


e = Gauge('robot_control_state', 'Current control state')
speed_gauge = Gauge('speed', 'Speed of the vehicle')
lat_gauge = Gauge('lat', 'Latitude of the vehicle')
lon_gauge = Gauge('lon', 'Longitude of the vehicle')
alt_gauge = Gauge('alt', 'Alt of the vehicle')
angle_gauge = Gauge('angle', 'Angle of the vehicle')
lidar_hz_gauge = Gauge('lidar_hz', 'Lidar publishing frequency')
camera_hz_gauge = Gauge('camera_hz', 'Camera publishing frequency')
gnss_hz_gauge = Gauge('gnss_hz', 'GNSS publishing frequency')
transmission_gauge = Gauge('transmission', 'Transmission number')
steering_gauge = Gauge('steering', 'Steering angle')


def set_state(state):
    if state == LogServerStatus.STOPPED:
        set_location(0.0, 0.0, 0.0, 0.0, 0.0)
        set_lidar_hz(0)
        set_gnss_hz(0)
        set_camera_hz(0.0)
        set_transmission(0.0)
    print(state)
    e.set(state.value)

def set_location(speed, lat, lon, alt, angle):
        #print(f'sending info {speed} {lat} {lon} {alt} {angle}')
    speed_gauge.set(speed)
    lat_gauge.set(lat)
    lon_gauge.set(lon)
    alt_gauge.set(alt)
    angle_gauge.set(angle)

def set_lidar_hz(lidar_hz):
    lidar_hz_gauge.set(lidar_hz)

def set_transmission(transmission):
    transmission_gauge.set(transmission)

def set_steering(steering):
    steering_gauge.set(steering)

def set_camera_hz(camera_hz):
    camera_hz_gauge.set(camera_hz)

def set_gnss_hz(gnss_hz):
    gnss_hz_gauge.set(gnss_hz)


