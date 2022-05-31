#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from utils import *
import traceback
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class StereoRecognizer(object):
    control_pub = None
    cmd_pub = None

    def publish_control(self, velocity, wheel, transmission):
        msg = ""
        if velocity != None:
            msg = "velocity: {};".format(velocity)
        if wheel != None:
            msg += "wheel: {};".format(wheel)
        if transmission != None:
            msg += "transmission: {}".format(transmission)
        self.control_pub.publish(msg)

    def start_car(self):
        '''
        Начало движения
        '''
        log(self, "visual start")
        self.publish_control(5, 0, 1)
        # запуск распознавания
    
    def stop_car(self):
        '''
        Остановка алгоритма распознаания
        '''
        log(self, "visual stop")
        self.publish_control(0, 0, 0)

    def turn_car(self):
        '''
        Разворот автомобиля
        '''
        log(self, "turn")
        self.publish_control(5, 600, -1)
        rospy.sleep(2)
        self.publish_control(5, -600, 1)
        rospy.sleep(2)
        self.publish_control(5, 600, -1)
        rospy.sleep(2)
        self.publish_control(5, -600, 1)
        rospy.sleep(2)
        self.publish_control(5, 0, 1)
        rospy.sleep(2)
        self.cmd_pub.publish("finishturn")

    def finish_car(self):
        '''
        Завершение выполнения задания
        '''
        log(self, "finish")
        self.publish_control(0, 0, 0)

    def cmd_callback(self, data):
        '''
        Callback для получения сигнала управления автомобилем
        '''
        log(self, "data received: {}".format(data))
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        cmd = {}
        for v in vars:
            keyval = v.split(':')
            cmd[clear_str(keyval[0])] = clear_str(keyval[1])
        self.cur_cmd = {}
        if "cmd" in cmd:
            if cmd["cmd"] == "start":
                self.start_car()
            elif cmd["cmd"] == "stop":
                self.stop_car()
            elif cmd["cmd"] == "turn":
                self.turn_car()

    def image_map_callback(self, ros_img):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(ros_img, "rgb8")
      except CvBridgeError as e:
        rospy.logerr('Ошибка конвертирования изображения: {}'.format(traceback.format_exc()))

      (rows,cols,channels) = cv_image.shape
      if cols > 60 and rows > 60 :
        cv2.circle(cv_image, (50,50), 10, 255)
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)


    def start(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("carcmd", String, self.cmd_callback)
        rospy.Subscriber("img_cloud_points", Image, self.image_map_callback)
        self.control_pub = rospy.Publisher('carcontrol', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('completetask', String, queue_size=10)

if __name__ == '__main__':
    try:
        rospy.init_node('stereorecognizer', anonymous=True)
        ctrl = StereoRecognizer()
        ctrl.start()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))


#roslaunch ouster_ros ouster.launch sensor_hostname:=10.3.23.82 \
#                                   udp_dest:=10.3.23.48 \
#                                   lidar_mode:=512x10 viz:=true