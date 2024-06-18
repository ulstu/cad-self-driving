import json
import os
from tkinter import *
from tkinter import filedialog as fd

import PIL
import cv2
import numpy as np
from PIL import Image, ImageTk
from matplotlib import pyplot as plt

from ipm360 import IPM360
from ipm_calibration_tool import CalibrationTool
from ipm_transformer import IPMTransformer
from untils.rectangle import rectangle

project_root = "data"


def get_rgb(rgb):
    return "#%02x%02x%02x" % rgb


class Camera:
    def __init__(self, tk_object, pos):
        self.tk_object = tk_object
        self.pos = pos


class ManyCameras:
    cameras_count = 6
    preview_image_size = 200
    # Позиционирование камер проходит против часовой стрелки от камеры, смотрящей вперед
    default_cameras_positions = [(800, 0), (0, 800), (0, 1600), (800, 2400), (1600, 1600), (1600, 800)]
    cameras_position = []
    cameras_images = [None for i in range(cameras_count)]
    source_cameras_images = [None for _ in range(cameras_count)]
    rotated_cameras_images = []
    scale_cameras_images = []

    def my_callback(self, var):
        self.place_camera_elements()

    def __init__(self):
        self.fullconfig = {"data": []}
        for i in range(self.cameras_count):
            self.fullconfig["data"].append({"pos": self.default_cameras_positions[i], "angle": 0, "homography": {}})
        self.root = Tk()
        self.start_pos = (450, 100)
        self.root.title("Calibration tool for IPM")  # title of the GUI window
        self.root.maxsize(1500, 1000)  # specify the max size the window can expand to
        self.nav = Canvas(self.root)
        self.nav.config(width=1500, height=100)
        self.nav.grid(row=0, column=0)
        self.cameras = []
        self.canvas = Canvas(self.root)
        self.canvas.config(width=750, height=900)
        self.canvas.grid(row=1, column=0)
        self.canvas.bind('<Button-1>', self.selected)
        self.frame_config = Frame(self.root, borderwidth=1, relief=SOLID)
        self.frame_config.config(width=600, height=600)
        self.frame_config.grid(row=1, column=1, padx=5, pady=5)
        self.configs_frames = []
        for i in range(self.cameras_count):
            card = Frame(self.frame_config, borderwidth=1, relief=SOLID)
            self.configs_frames.append(card)
            card.config(width=560, height=130)
            card.grid(row=int(i / 2), column=i % 2, padx=10, pady=10)

            varx = IntVar()
            vary = IntVar()
            vara = IntVar()
            vars = IntVar()

            varx.set(self.default_cameras_positions[i][0])
            vary.set(self.default_cameras_positions[i][1])
            vars.set(10)

            ll = Label(card, text=f"Camera {i}")
            ll.grid(row=0)

            lx = Label(card, text="X")
            lx.grid(row=1, column=0)
            wx = Scale(card, from_=0, to=2400, orient=HORIZONTAL, variable=varx, command=self.my_callback)
            wx.grid(row=1, column=1)

            wx.bind("<Button-1>", lambda event: event.widget.focus_set())

            ly = Label(card, text="Y")
            ly.grid(row=2, column=0)
            wy = Scale(card, from_=0, to=2400, orient=HORIZONTAL, variable=vary, command=self.my_callback)
            wy.grid(row=2, column=1)
            wy.bind("<Button-1>", lambda event: event.widget.focus_set())

            la = Label(card, text="Angle")
            la.grid(row=3, column=0)
            wa = Scale(card, from_=0, to=359, orient=HORIZONTAL, variable=vara, command=self.my_callback)
            wa.grid(row=3, column=1)
            wa.bind("<Button-1>", lambda event: event.widget.focus_set())

            ls = Label(card, text="Scale x10")
            ls.grid(row=4, column=0)
            ws = Scale(card, from_=1, to=20, orient=HORIZONTAL, variable=vars, command=self.my_callback)
            ws.grid(row=4, column=1)
            ws.bind("<Button-1>", lambda event: event.widget.focus_set())

            self.cameras_position.append((wx, wy))
            self.rotated_cameras_images.append(wa)
            self.scale_cameras_images.append(ws)

        self.images = {}
        self.place_camera_elements()

        self.find_homography_button = Button(
            self.nav,
            text='Load config',
            command=self.load_config
        )
        self.save_config_button = Button(
            self.nav,
            text='Save config',
            command=self.save_config
        )
        self.load_pre_image_button = Button(
            self.nav,
            text='Load preview images',
            command=self.load_pre_images
        )
        self.load_image_button = Button(
            self.nav,
            text='Load images',
            command=self.load_images
        )

        self.find_homography_button.grid(row=0, column=1, padx=5, pady=5)
        self.load_pre_image_button.grid(row=0, column=2, padx=5, pady=5)
        self.save_config_button.grid(row=0, column=3, padx=5, pady=5)
        self.load_image_button.grid(row=0, column=4, padx=5, pady=5)
        self.root.mainloop()
        self.images = {}

    def place_camera_elements(self):
        self.canvas.delete("all")
        # self.canvas.create_image(0, 0,
        #                          anchor=NW, image=PhotoImage("data/set1/set1_back.png"))
        # Отображение предпросмотра камер
        for i in range(len(self.cameras_position)):
            if not self.cameras_images[i]:
                self.canvas.create_rectangle(
                    *rectangle(self.cameras_position[i][0].get() // 4, self.cameras_position[i][1].get() // 4,
                               round(200 * (self.scale_cameras_images[i].get() / 10)), round(
                                           200 * (self.scale_cameras_images[i].get() / 10))),
                    fill='grey',
                    outline='blue',
                    width=3,
                    activedash=(5, 4), tags=f"camera_{i}")
            else:
                img = self.cameras_images[i].resize(
                    (round(self.cameras_images[i].size[0] * self.scale_cameras_images[i].get() / 10),
                     round(self.cameras_images[i].size[1] * self.scale_cameras_images[i].get() / 10)))
                img = img.rotate(self.rotated_cameras_images[i].get(), PIL.Image.NEAREST,
                                 expand=1).convert('RGBA')
                pixdata = img.load()

                width, height = img.size
                for y in range(height):
                    for x in range(width):
                        if pixdata[x, y] == (0, 0, 0, 255):
                            pixdata[x, y] = (0, 0, 0, 0)
                photo = ImageTk.PhotoImage(img)
                globals()[f"self.canvas.photo{i}"] = photo
                self.canvas.create_image(self.cameras_position[i][0].get() // 4, self.cameras_position[i][1].get() // 4,
                                         anchor=NW, image=photo, tags=f"camera_{i}")
                # Отображение списка конфигов
                # for i in range(len(self.default_cameras_positions)):

    def load_config(self):
        filename = fd.askopenfilename(filetypes=[("YAML config", "*.yaml")])
        if not os.path.exists(filename):
            print('Config file not found. Use default values')
            return
        with open(filename) as file:
            config = yaml.full_load(file)
        self.fullconfig = dict(config)
        self.parce_config()

    def parce_config(self):
        for i in range(len(self.cameras_position)):
            self.cameras_position[i][0].set(self.fullconfig["data"][i]["pos"][0])
            self.cameras_position[i][1].set(self.fullconfig["data"][i]["pos"][1])
            self.rotated_cameras_images[i].set(self.fullconfig["data"][i]["angle"])
            self.scale_cameras_images[i].set(self.fullconfig["data"][i]["scale"])
        self.place_camera_elements()

    def save_config(self):
        filename = fd.asksaveasfile(filetypes=[("YAML config", "*.yaml")]).name
        if filename.split(".")[-1] != "yaml":
            filename += ".yaml"
        self.save_pos_to_fullconfig()
        with open(filename, 'w') as file:
            yaml.dump(self.fullconfig, file)

    def save_pos_to_fullconfig(self):
        for i in range(len(self.cameras_position)):
            self.fullconfig["data"][i]["pos"] = (
                self.cameras_position[i][0].get(), self.cameras_position[i][1].get())
            self.fullconfig["data"][i]["angle"] = self.rotated_cameras_images[i].get()
            self.fullconfig["data"][i]["scale"] = self.scale_cameras_images[i].get()

    def load_pre_images(self):
        self.save_pos_to_fullconfig()
        ipm360 = IPM360()
        ipm360.load_config(self.fullconfig)
        # self.source_cameras_images[1].show()
        ipm360.homography360(self.source_cameras_images).show()
    def homography(self, image, idx):
        if idx < 0 or idx >= self.cameras_count:
            print(f"Index {idx} don`t merge with cameras count")
            return
        homo_data = self.fullconfig["data"][idx]["homography"]
        h_img = cv2.resize(image, (homo_data["width"], homo_data["height"]))
        ipm_transformer = IPMTransformer(homography_matrix=np.array(homo_data["homography"]))
        img_ipm = ipm_transformer.get_ipm(h_img, is_mono=False, horizont=homo_data["horizont"])
        pil_img = Image.fromarray(img_ipm)
        target_width = 400  # 400
        pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))
        return pil_img

    def load_images(self):
        filenames = fd.askopenfilenames(filetypes=[("Images", "*.png")])
        # self.save_pos_to_fullconfig()
        for idx in range(len(filenames)):
            filename = filenames[idx]
            print(filename)
            self.source_cameras_images[idx] = Image.open(filename)
            self.cameras_images[idx] = self.homography(np.array(self.source_cameras_images[idx]), idx).resize(
                (200, 200))
        self.place_camera_elements()

    def load_homografy(self):
        fig = plt.figure()

        res = Image.new("RGB", (1200 * 3, 1200 * 3), color="black")
        res.paste(self.images["forward"], (1200, 0))
        res.paste(self.images["left"], (0, 1200))
        res.paste(self.images["backward"], (1200, 2400))
        res.paste(self.images["right"], (2400, 1200))

        ax1 = fig.add_subplot(3, 3, 5)
        ax1.imshow(res)
        fig.show()

    def selected(self, event):
        nearby_tags = self.canvas.find_closest(event.x, event.y)
        if self.canvas.gettags(nearby_tags[0])[0].split("_")[0] != "camera":
            return
        idx = int(self.canvas.gettags(nearby_tags[0])[0].split("_")[1])
        cb = CalibrationTool(self.root, f"Camera #{idx}", os.path.join(project_root, "config"))
        config, image, src_image = cb.get_data()
        if config and image:
            print(config, image)
            image = image.resize((200, 200))
            self.fullconfig["data"][idx]["homography"] = config
            self.cameras_images[idx] = image
            self.source_cameras_images[idx] = src_image
            self.place_camera_elements()
