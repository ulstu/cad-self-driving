#!/usr/bin/env python3

import rospy
import time
import roslib
import sys
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError


class ObstacleDetector:
    def __init__(self):
        self.angle_pub = rospy.Publisher("angle", String, queue_size=1)
#        self.angle_pub = rospy.Publisher("angle", std_msgs.msg.String, queue_size=5)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        self.blockpub = rospy.Publisher('blockdetect', String, queue_size=10)

        #self.bridge = CvBridge()
        time.sleep(10)
        self.image_sub = rospy.Subscriber("/img_node/range_image", Image, self.image_callback)
        self.pos_history = []

    def image_callback(self, data):
        try:
            cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #rospy.loginfo(f'image received {cv_image.shape}')
        except Exception as e:
            print(e)

        #rospy.loginfo("image shape {} {} {}".format(rows, cols, channels))
        scale_percent = 30  # percent of original size
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        #cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        window_size = 120
        imin = 0
        stride = 1
        padding = 500
        #cv_image[cv_image < 80] = 0
        np_shift_img = cv_image.copy()
        np_img = np_shift_img[1:, :, :] - cv_image[:-1, :, :]
        
        np_img = np_img[:,padding:-padding,:]
        np_img[np_img >= 3] = 255
        np_img[np_img < 3] = 0
        np_img = 255 - np_img
        img_height = 120
        new_img = np.empty((img_height, np_img.shape[1], np_img.shape[2]), dtype=np.uint8)
        for i in range(img_height):
            r = int(i / int(img_height / np_img.shape[0]))
            #print(new_img[i, :, :].shape, np_img[r, :, :].shape)
            new_img[i, :, :] = np_img[r, :, :]
        np_img = new_img
        (rows, cols, channels) = np_img.shape
        minw = (255 * img_height * window_size)
        for i in range(0, np_img.shape[1] - window_size, stride):
            window = np_img[:, i : i + window_size,:]
            s = np.sum(window)
            if s <= minw:
                minw = s
                imin = i
        #rospy.loginfo(f'window size: {minw}')

        #if len(self.pos_history) > 0 and abs(imin - self.pos_history[-1]) > 20:
        #    imin = self.pos_history[-1]
        #filter_size = 3
        ## if len(self.pos_history) > filter_size:
        ##     imin = int((np.mean(self.pos_history[:-filter_size:-1]) + imin) / 2)
        #self.pos_history.append(imin)
        #np_img = (np_img -235) * 12
        np_img = cv2.cvtColor(np_img.astype(np.uint8),cv2.COLOR_GRAY2RGB)
        
        cv2.rectangle(np_img, (imin, 0), (imin + window_size, rows), (0, 255, 0), 2)
        cv2.line(np_img, (imin + int(window_size / 2), 0), (int(cols / 2), img_height), (255, 0, 0), 3)
        angle = int(np.arctan(((imin + int(window_size / 2) - cols / 2) / 16))  * 90)
        if angle > 0:
            angle -= 10
        else:
            angle += 10
        self.angle_pub.publish(str(angle))
        cv2.imshow("Image window", np_img)
        cv2.waitKey(3)

        try:
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            pass
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('image_obstacle_detection', anonymous=True)
        detector = ObstacleDetector()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
