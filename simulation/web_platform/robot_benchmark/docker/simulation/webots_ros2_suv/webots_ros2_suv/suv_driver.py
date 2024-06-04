import rclpy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry

class SUVDriver:
    def init(self, webots_node, properties):
        try:
            self.__robot = webots_node.robot

            # ROS interface
            rclpy.init(args=None)
            self.__node = rclpy.create_node('suv_node')
            self.__node.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)

            self.__timestep = int(self.__robot.getBasicTimeStep())

            # Sensors
            self.__robot.getDevice('gps').enable(16)
            self.__gps = self.__robot.getDevice('gps')
            self.__gyro = self.__robot.getDevice('gyro')
            self.__imu = self.__robot.getDevice('inertial_unit')
            
            self.step(16)
            self.__node._logger.info(f'type: {type(self.__robot)}')
        except  Exception as err:
            print(f'{str(err)}')
        
    def __cmd_ackermann_callback(self, message):
        self.__robot.setCruisingSpeed(message.speed)
        self.__robot.setSteeringAngle(message.steering_angle)

    def step(self, d=0):
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except  Exception as err:
            print(f'{str(err)}')

        
        #print(self.__gps.getValues())
        #print(self.__gps.getSpeed())
        #print(self.__gps.getSpeedVector())
        #self.__publish_odometry()
        
