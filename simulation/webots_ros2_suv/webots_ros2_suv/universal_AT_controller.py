import rclpy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from jsonrpc import JSONRPCResponseManager, Dispatcher
from vehicle import Driver
import socket
import time

UDP_SERVER_IP = "127.0.0.1"
UDP_PORT = 5005

COMMAND_TIMEOUT = 5000

INJECTORS_COUNT = 10


class ATController:
    robot = None
    emergencyStop = True
    commandTimeout = False
    last_cmd_frame = 0

    __sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    __sock.settimeout(0.001)

    injectors_state = []

    __rpc_dispatcher = Dispatcher()

    @staticmethod
    @__rpc_dispatcher.add_method
    def setControl(wheel=None, brake=None, targetSpeed=None, turnIndicator=None):
        if (ATController.emergencyStop or ATController.commandTimeout):
            return '"status":"failed,"reason":"controllerHasErrors"'

        if wheel is not None:
            ATController.robot.setSteeringAngle(wheel)
        if brake is not None:
            ATController.robot.setBrakeIntensity(brake)
        if targetSpeed is not None:
            ATController.robot.setCruisingSpeed(targetSpeed)
        if turnIndicator is not None:
            if turnIndicator == "left":
                ATController.robot.setIndicator(Driver.INDICATOR_LEFT)
            elif turnIndicator == "right":
                ATController.robot.setIndicator(Driver.INDICATOR_RIGHT)
            elif turnIndicator == "off":
                ATController.robot.setIndicator(Driver.INDICATOR_OFF)

        return '"status":"ok"'

    @staticmethod
    @__rpc_dispatcher.add_method
    def resetErrors(errors):
        if "emergencyStop" in errors:
            ATController.emergencyStop = False
        if "commandTimeout" in errors:
            ATController.commandTimeout = False
        return '"status":"ok"'

    @staticmethod
    @__rpc_dispatcher.add_method
    def getErrors():
        errors = []
        if ATController.emergencyStop:
            errors.append("emergencyStop")
        if ATController.commandTimeout:
            errors.append("commandTimeout")
        return errors

    @staticmethod
    @__rpc_dispatcher.add_method
    def setInjectors(byMask=True, num=None, state=None, mask=None):
        if byMask == True:
            if mask == None:
                return '"status":"failed,"reason":"invalidMask"'
            for i in range(INJECTORS_COUNT):
                ATController.injectors_state[i] = mask & 1
                mask = mask >> 1
            return '"status":"ok"'
        elif num == None:
            return '"status":"failed,"reason":"invalidNum"'
        elif state == None:
            return '"status":"failed,"reason":"invalidState"'
        else:
            ATController.injectors_state[num] = state

    @staticmethod
    @__rpc_dispatcher.add_method
    def getInjectors(byMask=True, num=None):
        if byMask:
            mask = 0
            for i in range(INJECTORS_COUNT):
                mask = mask << 1
                if ATController.injectors_state[INJECTORS_COUNT - i - 1]:
                    mask |= 1
            return '"mask":"' + mask + '"'
        elif num == None:
            return '"status":"failed,"reason":"invalidNum"'
        else:
            return '"state":"' + ATController.injectors_state[num] + '"'

    def __check_rpc_msgs(self):
        try:
            data, addr = self.__sock.recvfrom(1024)
        except socket.timeout:
            return

        if not data:
            return

        ATController.last_cmd_frame = int(round(time.time() * 1000))

        response = JSONRPCResponseManager.handle(data.decode("utf-8"), self.__rpc_dispatcher).json

        self.__sock.sendto(response, addr)
        # self.__node._logger.info(response)

    def init(self, webots_node, properties):
        try:
            ATController.robot = webots_node.robot

            # ROS interface
            rclpy.init(args=None)
            self.__node = rclpy.create_node('suv_node')

            self.__node._logger.info('AT controller preparing')

            self.__sock.bind((UDP_SERVER_IP, UDP_PORT))

            for i in range(INJECTORS_COUNT):
                ATController.injectors_state.append(False)

            self.__timer = self.__node.create_timer(0.1, self.__timer_callback)
            self.__node.create_timer(0.05, self.__check_rpc_msgs)

            self.__node._logger.info('AT controller initialized')



        except  Exception as err:
            print(f'{str(err)}')

    def __timer_callback(self):
        if int(round(
                time.time() * 1000)) > ATController.last_cmd_frame + COMMAND_TIMEOUT and not ATController.commandTimeout:
            ATController.commandTimeout = True
            self.__node._logger.info('AT controller command timeout')

        if ATController.commandTimeout or ATController.emergencyStop:
            ATController.robot.setCruisingSpeed(0)
            ATController.robot.setHazardFlashers(True)
        else:
            ATController.robot.setHazardFlashers(False)
        ATController.robot.step()

    def step(self, d=0):
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except  Exception as err:
            print(f'{str(err)}')
