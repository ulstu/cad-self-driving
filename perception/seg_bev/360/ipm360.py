import json
import os

import PIL
import cv2
import numpy as np
from PIL import Image

from ipm_transformer import IPMTransformer


class IPM360:
    def __init__(self):
        self.config = None
        self.IPMconfig = None
        self.cameras_count = 0

    def json_load_config(self, filename):
        if not os.path.exists(filename):
            print('Config file not found. Use default values')
            return
        with open(filename) as file:
            config = json.load(file)
        self.config = dict(config)
        self.cameras_count = len(self.config["data"])
        self.IPMconfiging()

    def load_config(self, config):
        self.config = config
        self.cameras_count = len(self.config["data"])
        self.IPMconfiging()

    def IPMconfiging(self):
        self.IPMconfig = []
        for i in range(self.cameras_count):
            homo_data = self.config["data"][i]["homography"]
            tmp = IPMTransformer(homo_data["homography"])
            tmp.calc_homography(np.array(homo_data["src_points"]), np.array(homo_data["dst_points"]))
            self.IPMconfig.append(tmp)

    # Принимает список PIL img
    def homography360(self, cameras):
        if len(cameras) != self.cameras_count:
            print(f"Length cameras array dont equals cameras count {self.cameras_count}")
            return
        res_image = Image.new("RGBA", size=(800 * 3, 800 * 4))

        for idx in range(self.cameras_count):
            camera_data = self.config["data"][idx]

            globals()["image{idx}"] = self.homography(cameras[idx], idx)
            globals()["image{idx}"] = globals()["image{idx}"].resize(
                (round(800 * camera_data["scale"] / 10),
                 round(800 * camera_data["scale"] / 10)))
            globals()["image{idx}"] = globals()["image{idx}"].rotate(camera_data["angle"], PIL.Image.NEAREST,
                                                                     expand=True).convert("RGBA")

            pixdata = globals()["image{idx}"].load()

            width, height = globals()["image{idx}"].size
            for y in range(height):
                for x in range(width):
                    if pixdata[x, y][0] == 0 and pixdata[x, y][1] == 0 and pixdata[x, y][2] == 0:
                        pixdata[x, y] = (0, 0, 0, 0)
            res_image.alpha_composite(globals()["image{idx}"], (camera_data["pos"][0], camera_data["pos"][1]))
        return res_image

    def homography(self, image, idx):
        if idx < 0 or idx >= self.cameras_count:
            print(f"Index {idx} don`t merge with cameras count")
            return
        homo_data = self.config["data"][idx]["homography"]
        print(homo_data)
        image = np.array(image)
        h_img = cv2.resize(image, (homo_data["width"], homo_data["height"]))
        ipm_transformer = IPMTransformer(homography_matrix=np.array(homo_data["homography"]))
        img_ipm = ipm_transformer.get_ipm(h_img, is_mono=False, horizont=homo_data["horizont"])
        pil_img = Image.fromarray(img_ipm)
        target_width = 800  # 400
        pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))
        return pil_img
