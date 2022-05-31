#!/usr/bin/env python3

import rospy
import time
import roslib
import math
import sys
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError


class PathDetector:
    def __init__(self):
        self.is_init = False
        self.angle_pub = rospy.Publisher("serialcode", String, queue_size=1)
#        self.angle_pub = rospy.Publisher("angle", std_msgs.msg.String, queue_size=5)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)

        time.sleep(10)
        self.image_sub = rospy.Subscriber("/img_node/range_image", Image, self.image_callback)
        self.carstate_sub = rospy.Subscriber("carstate", String, self.carstate_callback)
        self.is_turn = False
        self.is_stop = False
        self.pos_history = []
        #self.cap = cv2.VideoWriter('/home/nvidia/video_lidar/lidar_data.avi', cv2.VideoWriter_fourcc(*'XVID'), 40.0, (2048, 16))
        self.is_init = True
        

        
    def carstate_callback(self, data):
        if 'turn' in data.data:
            self.is_turn = True
        if 'stop' in data.data:
            self.is_stop = True
    
    def detect_blocks(self, img):
        m = np.median(img)
        #print(m)
        #print('-----')
        self.angle_pub.publish("6 " + str(m))
        return m
    
    def new_detect_blocks(self, image):
        image[image<200]=0
        contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_len2center = 90000
        needed_block = []
        range_end = 10000
        center = image.shape[1] // 2
        print(image.shape)
        #viz_image = 
        for contour in contours:
            box = cv2.boundingRect(contour)
            x, y, w, h = box
            len1 = abs(x - center)
            len2 = abs(x + w - center)
            rang = np.median(image[2, x:x+w, :])
            if len1 + len2 < min_len2center and w > 20 and rang > 150:
                needed_block = box
                min_len2center = len1 + len2
                #range = np.median(image[2, x:x+w, :])   
                range_end = rang
        print(f'range = {range_end} box = {box} new = {np.median(image[0, x:x+w, :])}')     
        return range_end
    
    def search_window(self, np_img, img_height, window_size, minval = 0, is_near=False):
        (rows, cols, channels) = np_img.shape
        imin = 0
        stride = 1
        minw = (255 * img_height * window_size)
        np_img[np_img < minval] = 0
        for i in range(0, np_img.shape[1] - window_size, stride):
            window = np_img[:1, i : i + window_size,:]
            s = np.sum(window)
            if (is_near and s < minw) or (not is_near and s <= minw):# or  (abs(imin - i) < 10 and abs(s - minw) < 1000):
                minw = s
                imin = i
        return minw, imin

    def search_nearest(self, np_img, minval = 0):
        np_img[np_img < minval] = 0
        ilongest_empty = int(np_img.shape[1] / 2)
        sum_rows_img = np_img.sum(axis=0)
        boundary_array = []
        left_boundary = 0
        right_boundary = 0
        boundary_state = False
        
        for i in range(0, sum_rows_img.shape[0], 1):
            if boundary_state == False and sum_rows_img[i][0] == 0:
                left_boundary = i
                boundary_state = True
            elif (boundary_state == True and sum_rows_img[i][0] > 0) or (boundary_state == True and i == sum_rows_img.shape[0] - 1):
                right_boundary = i
                boundary_array.append([left_boundary, right_boundary])
                boundary_state = False
                 
        #print(boundary_array)
        space_max = 0
        for space in boundary_array:
            if (space[1] - space[0]) > space_max:
                space_max = space[1] - space[0]
                ilongest_empty = int((space[1] + space[0]) / 2)
                
        height = 60
        new_img = cv2.resize(np_img, (np_img.shape[1], height), interpolation = cv2.INTER_AREA)
        new_img = cv2.cvtColor(new_img.astype(np.uint8),cv2.COLOR_GRAY2RGB)
        cv2.line(new_img, (ilongest_empty, 0), (int(np_img.shape[1] / 2), height), (0, 0, 255), 3)
        cv2.imshow("longest window", new_img)
        return ilongest_empty

    def image_callback(self, data):
        if self.is_init:
            try:
                cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            except Exception as e:
                print(e)
        
            padding = 500
            np_img = cv_image[6:9, padding:-padding,:].copy()
            
            #self.new_detect_blocks(np_img)
            block_distance = self.detect_blocks(np_img)
            #block_distance = np.median(cv_image[6, 1100:1200, :])
            #print(block_distance) #145 140
            left_edge = np.median(cv_image[6:9, 341:682, :])
            right_edge = np.median(cv_image[6:9, 1365:1707, :])
            #print(left_edge, right_edge)
            #print(block_distance)
            if self.is_turn:
                left_edge = np.median(cv_image[6:9, 341:682, :])
                right_edge = np.median(cv_image[6:9, 1365:1707, :])
                #print(left_edge, right_edge)
                #block_distance = self.new_detect_blocks(np_img)
                if right_edge > left_edge and block_distance > 170:
                
                    self.angle_pub.publish("5 1")
                    self.is_turn = False
                elif right_edge < left_edge and block_distance > 190:
                    self.angle_pub.publish("5 2")
                    self.is_turn = False
            if self.is_stop:
                #block_distance = self.new_detect_blocks(np_img)
                if block_distance > 145:
                    self.is_stop = False
                    self.angle_pub.publish("7 0")

            window_size = 140
            window_size_near = 190
            minw, imin = self.search_window(np_img, np_img.shape[0], window_size=window_size, minval=160)
            near_imin = self.search_nearest(np_img, minval=235)

            if (imin + window_size / 2 > int(np_img.shape[1] / 2)):
                imin -= 50


            img_height = 66
            new_img = np.empty((img_height, np_img.shape[1], np_img.shape[2]), dtype=np.uint8)
            for i in range(img_height):
                r = int(i / int(img_height / np_img.shape[0]))
                new_img[i, :, :] = cv_image[6 + r, padding:-padding, :]
            np_img = new_img

            np_img = cv2.cvtColor(np_img.astype(np.uint8),cv2.COLOR_GRAY2RGB)
            (rows, cols, channels) = np_img.shape
            if abs(near_imin - cols / 2) > 50:
                if (imin  - cols / 2) < 0: # left
                    if (near_imin - cols / 2) > 0:
                        imin = near_imin
                    else:
                        imin = int((imin + near_imin) / 2)
                else: #right
                    if (near_imin - cols / 2) < 0: #left
                        imin = near_imin
                    else:
                        imin = int((imin + near_imin) / 2)
            else: 
                near_imin = int(np_img.shape[1] / 2 - window_size_near / 2)
            

            #print(imin, near_imin)
            cv2.rectangle(np_img, (imin, 0), (imin + window_size, rows), (0, 255, 0), 2)
            cv2.line(np_img, (imin + int(window_size / 2), 0), (int(cols / 2), img_height), (255, 0, 0), 3)
            cv2.line(np_img, (near_imin, 0), (int(cols / 2), img_height), (0, 0, 255), 3)

            angle = int(np.arctan(((imin + int(window_size / 2) - cols / 2) / img_height))  * 90)
            
            #angle =  angle ** 2 / 75 * (-1 if angle < 0 else 1)
            #near_angle = int(np.arctan(((near_imin + int(window_size_near / 2) - cols / 2) / img_height))  * 90)
            #near_angle = near_angle ** 2 / 75 * (-1 if near_angle < 0 else 1)
            #print(angle, near_angle)
            self.angle_pub.publish("16 " + str(angle * 12))
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
        detector = PathDetector()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
