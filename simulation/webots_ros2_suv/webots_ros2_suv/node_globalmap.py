import rclpy
import cherrypy
import traceback
import yaml
from ament_index_python.packages import get_package_share_directory
from .lib.world_model import WorldModel

BASE_PATH = get_package_share_directory('webots_ros2_suv') + '/'
# для отладки в режиме редактирования fronend части прописать абсолютный путь, например:
#BASE_PATH = '/home/hiber/ros2_ws/src/webots_ros2_suv/'
STATIC_PATH = BASE_PATH + 'map-server/dist/'
YAML_PATH = BASE_PATH + 'resource/map-config/robocross.yaml'
ASSETS_PATH = STATIC_PATH + 'assets/'

class MapServer(object):
    def __init__(self):
        try:
            print('Map server started')
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