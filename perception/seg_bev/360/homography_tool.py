import os
from tkinter import *

import cv2
import numpy as np
import yaml
from PIL import Image
from matplotlib import pyplot as plt

from ipm_transformer import IPMTransformer


class HomographyTool:
    def __init__(self):
        self.__img_width = 800
        self.__img_height = 800
        self.__src_image = None
        self.__canvas_image = None
        self.__horizont_line_height = 20
        self.ipm_transformer = IPMTransformer()
        self.RECT_SIZE = 15
        self.__img_filetype = "png"
        self.__img_filename = None

    def load_config(self, filename):
        if not os.path.exists(filename):
            print('Config file not found. Use default values')
            return
        with open(filename) as file:
            config = yaml.full_load(file)
        self.ipm_transformer = IPMTransformer(np.array(config['homography']))
        self.__horizont_line_height = config['horizont']

    def find_homography(self):
        h_img = cv2.resize(self.__src_image, (self.__img_width, self.__img_height))
        self.__img_ipm = self.ipm_transformer.get_ipm(h_img, is_mono=self.__img_filetype == "npy",
                                                      horizont=self.__horizont_line_height)

        return self.__img_ipm

    def load_image(self, filename):
        self.__img_filename = filename
        pil_img = Image.open(self.__img_filename)
        self.__src_image = np.array(pil_img)
        pil_img = pil_img.resize((self.__img_width, int(pil_img.size[1] * self.__img_width / pil_img.size[0])))
        self.__img_height = pil_img.size[1]
        self.__img_width = pil_img.size[0]
