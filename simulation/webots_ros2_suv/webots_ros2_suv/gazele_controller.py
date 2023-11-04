import rclpy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import serial
import time

use_serial = True
serial_name = "/dev/tnt0"


class GazeleController:
    __channels = [1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0]

    __last_rc_frame = 0

    stop_flag = 0

    __ser = serial.Serial(
        port=serial_name,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        writeTimeout=0.005,
        timeout=0.005)

    __counter = 0
    __first = 0x00
    __second = 0x00
    __third = 0x00
    __p_counter = 0
    __p_first = 0x00
    __p_second = 0x00
    __p_third = 0x00

    __return_state = 0
    __return_timer = 0
    __lidarDist = 250
    __cluthWork = 100
    __action = 0
    __autoFlag = 0
    __turnDir = 0
    __last_angle = 0
    __new_angle = 0

    __gear_num = 0

    def rc_available(self):
        return int(round(time.time() * 1000)) < self.__last_rc_frame + 1000

    def init(self, webots_node, properties):
        try:
            self.__robot = webots_node.robot

            # ROS interface
            rclpy.init(args=None)
            self.__node = rclpy.create_node('suv_node')

            self.__node._logger.info('Gazele controller preparing')

            self.__node.create_subscription(Int32MultiArray, 'cmd_rc', self.__cmd_rc, 1)

            self.__timestep = int(self.__robot.getBasicTimeStep())

            # Sensors
            self.__robot.getDevice('gps').enable(16)
            self.__gps = self.__robot.getDevice('gps')
            self.__gyro = self.__robot.getDevice('gyro')
            self.__imu = self.__robot.getDevice('inertial_unit')

            self.step(16)

            self.__timer = self.__node.create_timer(0.1, self.__timer_callback)

            self.__node._logger.info('Gazele controller initialized')

        except  Exception as err:
            print(f'{str(err)}')

    def millis(self):
        return int(round(time.time() * 1000))
    def __timer_callback(self):
        if (self.stop_flag == 0 and self.rc_available()):
            if self.__channels[8] < 1900:  # Manual control from RC
                wheel_position = (self.__channels[0] - 1500) * 0.001
                self.__robot.setSteeringAngle(wheel_position)

                clutch_position = (self.__channels[1] - 1500) * 0.002
                if clutch_position < 0:
                    clutch_position = 0
                self.__robot.setThrottle(clutch_position)

                brake_position = (1500 - self.__channels[1]) * 0.002
                if brake_position < 0:
                    brake_position = 0
                self.__robot.setBrakeIntensity(brake_position)

                gear_value = self.__channels[4]
                self.__gear_num = 0
                if gear_value > 1750:
                    self.__gear_num = -1
                elif gear_value > 1250:
                    self.__gear_num = 0
                elif gear_value > 999:
                    self.__gear_num = 1

                engine_control = self.__channels[5]
                if engine_control < 1500:
                    self.__robot.setGear(0)
                else:
                    self.__robot.setGear(self.__gear_num)

                signals_control = self.__channels[9]
                self.__robot.setHazardFlashers(signals_control > 1500)

                self.__robot.step()

                self.__node._logger.info("Manual throt:%f brake:%f gear:%d steer:%f" % (
                    clutch_position, brake_position, self.__gear_num, wheel_position))

                self.__ser.flushInput()
            else:  # Control from serial port
                while self.__ser.in_waiting > 0:
                    t = self.__ser.read(1)
                    if t // 128:
                        self.__first = t
                        self.__second = 0x00
                        self.__third = 0x00
                        self.__counter = 0
                    elif self.__counter == 0:
                        self.__second = t
                        self.__counter += 1
                    elif self.__counter == 1:
                        self.__third = t
                        cmd = self.__first << 1
                        cmd = cmd >> 3
                        result = self.__third
                        result += self.__second << 7
                        result += self.__first << 14

                        if cmd == 16:
                            self.__new_angle = result
                        elif cmd == 6:
                            self.__lidarDist = result
                        elif cmd == 8:
                            self.__autoFlag = 1
                        elif cmd == 5:
                            if result == 2:
                                self.__turnDir = 1
                            else:
                                self.__turnDir = 0
                            self.__return_state = 1
                            self.__return_timer = self.millis()
                        elif cmd == 7:
                            self.__autoFlag = 0
                        self.__counter += 1

                if self.__return_state > 0 and self.__autoFlag == 1:
                    if self.__return_state == 1:
                        self.__gear_num = 1
                        self.__robot.setSteeringAngle(0)
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 2000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 2:
                        self.__gear_num = 1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(-0.5)
                        else:
                            self.__robot.setSteeringAngle(0.5)
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 5000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 3:
                        self.__gear_num = 1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(-0.5)
                        else:
                            self.__robot.setSteeringAngle(0.5)
                        self.__robot.setBrakeIntensity(0)
                        self.__robot.setThrottle(0.5)
                        if self.__lidarDist > 200:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 4:
                        self.__gear_num = 1
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 2000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 5:
                        self.__gear_num = -1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(0.5)
                        else:
                            self.__robot.setSteeringAngle(-0.5)
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 10000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 6:
                        self.__gear_num = -1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(0.5)
                        else:
                            self.__robot.setSteeringAngle(-0.5)
                        self.__robot.setBrakeIntensity(0)
                        self.__robot.setThrottle(0.5)
                        if self.__lidarDist < 192:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 7:
                        self.__gear_num = -1
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 2000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 8:
                        self.__gear_num = 1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(-0.5)
                        else:
                            self.__robot.setSteeringAngle(0.5)
                        self.__robot.setBrakeIntensity(1)
                        self.__robot.setThrottle(0)
                        if self.millis() - self.__return_timer >= 10000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    elif self.__return_state == 2:
                        self.__gear_num = 1
                        if self.__turnDir == 0:
                            self.__robot.setSteeringAngle(-0.5)
                        else:
                            self.__robot.setSteeringAngle(0.5)
                        self.__robot.setBrakeIntensity(0)
                        self.__robot.setThrottle(0.5)
                        if self.millis() - self.__return_timer >= 6000:
                            self.__return_state += 1
                            self.__return_timer = self.millis()

                    engine_control = self.__channels[5]
                    if engine_control < 1500:
                        self.__robot.setGear(0)
                    else:
                        self.__robot.setGear(self.__gear_num)

                    self.__node._logger.info('Auto reversal mode')

                else:
                    clutch_position = self.__channels[1]
                    if self.__autoFlag == 0:
                        if clutch_position > 1850:
                            traffic_light_detect = self.__channels[2]
                            if traffic_light_detect < 1850:
                                self.__autoFlag = 1
                            self.__return_timer = self.millis()

                    if self.__autoFlag == 1:
                        self.__robot.setThrottle(0.5)
                        self.__robot.setBrakeIntensity(0)
                    else:
                        self.__robot.setThrottle(0)
                        self.__robot.setBrakeIntensity(1)

                    gear_value = self.__channels[4]
                    self.__gear_num = 0
                    if gear_value > 1750:
                        self.__gear_num = -1
                    elif gear_value > 1250:
                        self.__gear_num = 0
                    elif gear_value > 999:
                        self.__gear_num = 1

                    engine_control = self.__channels[5]
                    if engine_control < 1500:
                        self.__robot.setGear(0)
                    else:
                        self.__robot.setGear(self.__gear_num)

                    signals_control = self.__channels[9]
                    self.__robot.setHazardFlashers(signals_control > 1500)

                    self.__robot.setSteeringAngle(self.__new_angle * 0.001)

                    self.__node._logger.info('Auto mode')


        else:
            self.__node._logger.info('E-stop')
            self.__ser.flushInput()
            self.__robot.setBrakeIntensity(1)
            self.__robot.setThrottle(0)
            self.__robot.setSteeringAngle(0)
            self.__robot.setGear(0)
            self.stop_flag = 1
            self.__autoFlag = 0
            if self.__channels[1] < 1350:
                self.stop_flag = 0
        # if self.rc_available():
        #     self.__node._logger.info('RC available')
        # else:
        #     self.__node._logger.info('RC connection lost')

    def __cmd_rc(self, message):
        self.__channels = message.data
        self.__last_rc_frame = int(round(time.time() * 1000))

    def step(self, d=0):
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except  Exception as err:
            print(f'{str(err)}')

        # print(self.__gps.getValues())
        # print(self.__gps.getSpeed())
        # print(self.__gps.getSpeedVector())
        # self.__publish_odometry()
