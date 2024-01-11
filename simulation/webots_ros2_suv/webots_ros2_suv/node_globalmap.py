import rclpy
import cherrypy
import traceback
import yaml
import os
import pathlib
import json
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from os import listdir
from os.path import isfile, join
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose
import time

BASE_RESOURCE_PATH = get_package_share_directory('webots_ros2_suv') + '/'
# для отладки в режиме редактирования fronend части прописать абсолютный путь, например:

BASE_PATH = '/home/hiber/ros2_ws/src/webots_ros2_suv/'
#BASE_PATH = BASE_RESOURCE_PATH
STATIC_PATH = BASE_PATH + 'map-server/dist/'
YAML_PATH = BASE_PATH + 'config/map-config/robocross.yaml'
MAPS_PATH = BASE_PATH + 'config/global_maps/'
ASSETS_PATH = STATIC_PATH + 'assets/'

class MapServer(Node):
    def __init__(self):
        try:
            super().__init__('node_globalmap')
            self.get_logger().info('map server started')

            self.__cli_pos = self.create_client(PoseService, 'get_current_global_pos')
            while not self.__cli_pos.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('pos service not available, waiting again...')
        
        except  Exception as err:
            print(''.join(traceback.TracebackException.from_exception(err).format()))


    @cherrypy.expose
    def index(self):
        raise cherrypy.HTTPRedirect("static/index.html")

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_position(self):
        try:
            req = PoseService.Request()
            req.request.data = "Request"
            future = self.__cli_pos.call_async(req)
            self.get_logger().info(f"REQUEST STARTED")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f"REQUEST FINISHED");
            return {'status' : 'ok', 'lat': response.response.lat, 'lon': response.response.lon, 'orientation': response.response.orientation}
        except Exception as e:
            self.get_logger().error('Position service call failed %r' % (e,))
            return {'status' : 'error', 'message': ''.join(traceback.TracebackException.from_exception(e).format())}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_point_types(self):
        try:
            with open(YAML_PATH, 'r') as file:
                point_types = yaml.safe_load(file)
                print(point_types)
                return {'status': 'ok', 'pointtypes': point_types}
        except Exception as err:
            return {'status': ''.join(traceback.TracebackException.from_exception(err).format())}
        
    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_maps(self):
        try:
            map_files = [f for f in listdir(MAPS_PATH) if isfile(join(MAPS_PATH, f))]
            return {'status': 'ok', 'maps': map_files}
        except Exception as err:
            return {'status': ''.join(traceback.TracebackException.from_exception(err).format())}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def save_map(self, filename, map_data):
        try:
            with open(f'{MAPS_PATH}/{filename}.geojson', 'w') as f:
                json.dump(json.loads(map_data), f)
            return {'status': 'ok'} 
        except Exception as err:        
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
            return {'status': 'error', 'message': ''.join(traceback.TracebackException.from_exception(err).format())}

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def load_map(self, filename):
        try:
            with open(f'{MAPS_PATH}/{filename}') as f:
                j = json.load(f)
                return {'status': 'ok', 'features' : j} 
        except Exception as err:        
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
            return {'status': 'error', 'message': ''.join(traceback.TracebackException.from_exception(err).format())}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_init_point(self):
        config_path = os.path.join(BASE_RESOURCE_PATH,
                                    pathlib.Path(os.path.join(BASE_RESOURCE_PATH, 'config', 'map_config.yaml')))
        if not os.path.exists(config_path):
            print('Global map coords config file file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        return {'lat': config['lat'], 'lon': config['lon'], 'mapfile': config['mapfile']}

def main(args=None):
    try:
        rclpy.init(args=args)

        cherrypy.quickstart(MapServer(), '/', {'global':
                                                   {'server.socket_host': '127.0.0.1',
                                                    'server.socket_port': 8008,
                                                    'tools.staticdir.root': STATIC_PATH,
                                                    'log.error_file': 'site.log'
                                                    },
                                               '/static': {
                                                   'tools.staticdir.on': True,
                                                   'tools.staticdir.dir': STATIC_PATH,
                                                   'tools.staticdir.index': 'index.html'
                                               },
                                               '/assets': {
                                                   'tools.staticdir.on': True,
                                                   'tools.staticdir.dir': ASSETS_PATH,
                                                   'tools.staticdir.index': 'index.html'
                                               }
                                               })
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()