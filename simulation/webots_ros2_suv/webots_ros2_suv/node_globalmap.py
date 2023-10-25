import rclpy
import cherrypy
import traceback
import yaml
import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from .lib.world_model import WorldModel

BASE_RESOURCE_PATH = get_package_share_directory('webots_ros2_suv') + '/'
# для отладки в режиме редактирования fronend части прописать абсолютный путь, например:

BASE_PATH = '/home/hiber/ros2_ws/src/webots_ros2_suv/'
#BASE_PATH = BASE_RESOURCE_PATH
STATIC_PATH = BASE_PATH + 'map-server/dist/'
YAML_PATH = BASE_PATH + 'resource/map-config/robocross.yaml'
ASSETS_PATH = STATIC_PATH + 'assets/'

class MapServer(Node):
    def __init__(self):
        try:
            super().__init__('node_globalmap')
            self.get_logger().info('map server started')
            self.__world_model = WorldModel()
        except  Exception as err:
            print(''.join(traceback.TracebackException.from_exception(err).format()))

    @cherrypy.expose
    def index(self):
        raise cherrypy.HTTPRedirect("static/index.html")
    
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
    def get_init_point(self):
        config_path = os.path.join(BASE_RESOURCE_PATH,
                                    pathlib.Path(os.path.join(BASE_RESOURCE_PATH, 'config', 'map_config.yaml')))
        if not os.path.exists(config_path):
            print('Global map coords config file file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        return {'lat': config['lat'], 'lon': config['lon']}

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
        rclpy.shutdown()
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()