import math
import time
import numpy as np
import cv2
import traceback
import os
import pathlib
from scipy.spatial.transform import Rotation
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from PIL import Image
import torch
from datetime import datetime


class BEVCreator(object):
    def __init__(self) -> None:
        pass

    def make_bev(self, labels, range_image):
        cv2.imshow("range", range_image)
        colorized = colorize(labels)
        cv2.imshow("map image", np.asarray(colorized))
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        #seg_image = labels.copy()
        # result_image = np.zeros((MAP_DEPTH + 1, range_image.shape[1]))
        # result_image[result_image == 0] = -1

        # width, height = seg_image.shape[1], seg_image[0]
        # source = np.float32([[0, 0], [100, 0], [100, 100], [0, 100]])
        # dest = np.float32([[0, 0], [-1000, 0], [-1000, -1000], [0, -1000]])
        # # source = np.float32([[0, int(width / 2)], [height, int(width / 2)], [0, int(width / 2) - 100], [height, 0, int(width / 2) - 100]])
        # # dest = np.float32([[0, int(width / 2)], [height, int(width / 2)], [0, int(width / 2) - 200], [height, 0, int(width / 2) - 80]])
        # homography, _ = cv2.findHomography(source, dest)
        # result_image = cv2.perspectiveTransform(seg_image, homography)

#         for i in range(range_image.shape[1]):
#             for j in range(range_image.shape[0]):
#                 #x_coef = (MAP_DEPTH / (0.5 * MAP_DEPTH + j)) / 3
#                 #new_i = int(i * x_coef)
#                 #new_i = new_i if new_i < 840 else 839
#                 new_i = int(i - (i - range_image.shape[1] / 2) * 1.4)
# #                self._logger.info(f'{new_i}')
#                 result_image[MAP_DEPTH - int(range_image[j, i] * MAP_DEPTH), new_i] = seg_image[j, i]

        # colorized = colorize(result_image)
        # cv2.imshow("map image", np.asarray(colorized))
        # if cv2.waitKey(25) & 0xFF == ord('q'):
        #    return

def main():
    run = []
    bv = BEVCreator()
    prefix_path = "./perception/seg_bev/images_seg_range/"
    files = os.listdir(prefix_path)
    files.sort()
    for img in files:
        img_prefix = img.split('_')[0]
        if not img_prefix in run:
            run.append(img_prefix)
            bv.make_bev(np.load(f'{prefix_path}{img_prefix}_seg.npy'), 
                          np.load(f'{prefix_path}{img_prefix}_range.npy'))
            time.sleep(0.5)

if __name__ == '__main__':
    main()
