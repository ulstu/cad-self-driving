import os
import tkinter as tk
from tkinter import *
from tkinter import filedialog

import cv2
import yaml
from PIL import ImageTk as itk
import numpy as np
from PIL import Image, ImageTk

from perception.seg_bev.ipm_transformer import IPMTransformer


class ImageApp:
    ipm_transformer: IPMTransformer

    def __init__(self):
        self.__src_image = None
        self.__prev_canvas_image = None
        self.__img_prev_height = None
        self.__img_prev_width = 400
        self.pts_src: np.array
        self.pts_dst: np.array
        self.__horizont_line_height = 100
        self.RECT_SIZE = 10
        self.image_scale_coef = 1

        self.__canvas_image = None
        self.__canvas_lidar = None

        self.__img_height = 800
        self.__img_width = 800
        self.ipm_transformer = IPMTransformer()

        self.root = tk.Tk()

        self.root.title("Calibration tool for IPM")  # title of the GUI window
        self.root.geometry("1500x1000")
        self.root.maxsize(1500, 1000)

        self.left_frame = Frame(self.root, width=1200, height=100)
        self.left_frame.grid(row=0, column=0, padx=10, pady=5, columnspan=2)

        self.center_frame = Frame(self.root, width=1000, height=1000)
        self.center_frame.grid(row=1, column=0, padx=10, pady=5)

        self.limits_container = Canvas(self.center_frame)
        self.limits_container.grid(row=0, column=0)

        self.open_image_button = Button(
            self.left_frame,
            text='Open an image',
            command=self.open_image_dialog
        )
        self.load_lidar_button = Button(
            self.left_frame,
            text='Open a lidar',
            command=self.open_image_dialog
        )

        self.varh = IntVar()
        self.varh.set(100)

        self.src_x_list = list()
        self.src_y_list = list()

        self.src_x_sliders = list()
        self.src_y_sliders = list()

        self.dst_x_list = list()
        self.dst_y_list = list()

        self.dst_x_sliders = list()
        self.dst_y_sliders = list()

        for i in range(0, 4):
            varx = IntVar()
            vary = IntVar()
            self.src_x_list.append(varx)
            self.src_y_list.append(vary)

            varx_dst = IntVar()
            vary_dst = IntVar()
            self.dst_x_list.append(varx_dst)
            self.dst_y_list.append(vary_dst)

        self.configs_frames = []
        self.open_image_button.grid(row=0, column=0, padx=5, pady=5)
        self.load_lidar_button.grid(row=0, column=1, padx=5, pady=5)
        self.root.mainloop()

    def parameters(self):
        self.frame_config = Frame(self.root, borderwidth=1, relief=SOLID)
        self.frame_config.config(width=600, height=600)
        self.frame_config.grid(row=1, column=1, padx=5, pady=5)

        self.frame_h = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_h.config(width=600, height=600)
        self.frame_h.grid(row=0, column=0, padx=5, pady=5)
        lx = Label(self.frame_h, text="Horizontal")
        lx.grid(row=1, column=0)

        self.wx = Scale(self.frame_h, from_=0, to=self.__img_prev_height, orient=HORIZONTAL, variable=self.varh,
                        command=self.my_callback)
        self.wx.config(length=200)
        self.wx.grid(row=2, column=0)
        self.wx.bind("<ButtonRelease-1>", self.my_callback)

        self.wx.bind("<Button-1>", lambda event: event.widget.focus_set())

        self.frame_src = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_src.config(width=600, height=600)
        self.frame_src.grid(row=1, column=0, padx=5, pady=5)

        lx = Label(self.frame_src, text="SRC points")
        lx.grid(row=1, column=0, columnspan=2)

        self.frame_dst = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_dst.config(width=600, height=600)
        self.frame_dst.grid(row=2, column=0, padx=5, pady=5)

        lx = Label(self.frame_dst, text="DST points")
        lx.grid(row=1, column=0, columnspan=2)

        for i in range(0, 4):
            self.src_x_sliders.append(Scale(self.frame_src, from_=0, to=self.__img_prev_width, orient=HORIZONTAL,
                                            variable=self.src_x_list[i], command=self.my_callback))
            self.src_y_sliders.append(Scale(self.frame_src, from_=0, to=self.__img_prev_width, orient=HORIZONTAL,
                                            variable=self.src_y_list[i], command=self.my_callback))
            self.src_x_sliders[i].grid(row=i + 2, column=0)
            self.src_y_sliders[i].grid(row=i + 2, column=1)

            self.dst_x_sliders.append(Scale(self.frame_dst, from_=0, to=self.__img_prev_width, orient=HORIZONTAL,
                                            variable=self.dst_x_list[i], command=self.my_callback))
            self.dst_y_sliders.append(Scale(self.frame_dst, from_=0, to=self.__img_prev_width, orient=HORIZONTAL,
                                            variable=self.dst_y_list[i], command=self.my_callback))
            self.dst_x_sliders[i].grid(row=i + 2, column=0)
            self.dst_y_sliders[i].grid(row=i + 2, column=1)

    def update_parameters(self):
        self.wx.config(to=self.__img_prev_height * 2)
        for i in range(0, 4):
            self.src_x_sliders[i].config(to=self.__img_prev_width * 2)
            self.src_y_sliders[i].config(to=self.__img_prev_height * 2)
            self.dst_x_sliders[i].config(to=self.__img_prev_width * 2)
            self.dst_y_sliders[i].config(to=self.__img_prev_height * 2)

    def change_var(self, var):
        self.place_elements()
        self.show_homography()
        self.update_parameters()

    def my_callback(self, var):
        self.place_elements()
        self.update_homography()
        self.update_parameters()

    def place_elements(self):
        print(self.pts_src)
        self.limits_container.delete('all')
        self.limits_container.create_image(0, 0, image=self.__prev_canvas_image, anchor=NW, tags="preview")
        self.limits_container.config(width=1220,
                                     height=1000)
        self.__cut_line = self.limits_container.create_line(0, self.varh.get() // 2, self.__img_prev_width,
                                                            self.varh.get() // 2, tags="__cut_line",
                                                            fill="red",
                                                            width=2)

        self.lt = self.limits_container.create_rectangle(self.src_x_list[0].get() // 2, self.src_y_list[0].get() // 2,
                                                         self.src_x_list[0].get() // 2 + self.RECT_SIZE,
                                                         self.src_y_list[0].get() // 2 + self.RECT_SIZE, tags="lt",
                                                         fill="red")
        self.lb = self.limits_container.create_rectangle(self.src_x_list[1].get() // 2, self.src_y_list[1].get() // 2,
                                                         self.src_x_list[1].get() // 2 + self.RECT_SIZE,
                                                         self.src_y_list[1].get() // 2 + self.RECT_SIZE, tags="lb",
                                                         fill="green")
        self.rt = self.limits_container.create_rectangle(self.src_x_list[2].get() // 2, self.src_y_list[2].get() // 2,
                                                         self.src_x_list[2].get() // 2 + self.RECT_SIZE,
                                                         self.src_y_list[2].get() // 2 + self.RECT_SIZE, tags="rt",
                                                         fill="blue")
        self.rb = self.limits_container.create_rectangle(self.src_x_list[3].get() // 2, self.src_y_list[3].get() // 2,
                                                         self.src_x_list[3].get() // 2 + self.RECT_SIZE,
                                                         self.src_y_list[3].get() // 2 + self.RECT_SIZE, tags="rb",
                                                         fill="yellow")

        self.lt_dst = self.limits_container.create_oval(self.dst_x_list[0].get() // 2, self.dst_y_list[0].get() // 2,
                                                        self.dst_x_list[0].get() // 2 + self.RECT_SIZE,
                                                        self.dst_y_list[0].get() // 2 + self.RECT_SIZE, tags="lt_dst",
                                                        fill="orangered1")
        self.lb_dst = self.limits_container.create_oval(self.dst_x_list[1].get() // 2, self.dst_y_list[1].get() // 2,
                                                        self.dst_x_list[1].get() // 2 + self.RECT_SIZE,
                                                        self.dst_y_list[1].get() // 2 + self.RECT_SIZE, tags="lb_dst",
                                                        fill="lightseagreen")
        self.rt_dst = self.limits_container.create_oval(self.dst_x_list[2].get() // 2, self.dst_y_list[2].get() // 2,
                                                        self.dst_x_list[2].get() // 2 + self.RECT_SIZE,
                                                        self.dst_y_list[2].get() // 2 + self.RECT_SIZE, tags="rt_dst",
                                                        fill="lightblue")
        self.rb_dst = self.limits_container.create_oval(self.dst_x_list[3].get() // 2, self.dst_y_list[3].get() // 2,
                                                        self.dst_x_list[3].get() // 2 + self.RECT_SIZE,
                                                        self.dst_y_list[3].get() // 2 + self.RECT_SIZE, tags="rb_dst",
                                                        fill="lightyellow1")

    def show_homography(self):
        self.limits_container.create_image(self.__img_prev_width + 20, 0, image=self.photo, anchor=NW,
                                           tags="image")

    def update_homography(self):
        photo = self.find_homography()
        self.photo = ImageTk.PhotoImage(photo)
        self.show_homography()

    def fill_pts_from_sliders(self):
        self.__horizont_line_height = self.varh.get()
        # lt = self.limits_container.coords(self.lt) * 2
        # lb = self.limits_container.coords(self.lb) * 2
        # rt = self.limits_container.coords(self.rt) * 2
        # rb = self.limits_container.coords(self.rb) * 2
        # lt_dst = self.limits_container.coords(self.lt_dst) * 2
        # lb_dst = self.limits_container.coords(self.lb_dst) * 2
        # rt_dst = self.limits_container.coords(self.rt_dst) * 2
        # rb_dst = self.limits_container.coords(self.rb_dst) * 2
        #
        # # поправка на отрезанный горизонт
        # lt[1] = lt[1] - self.__horizont_line_height
        # lb[1] = lb[1] - self.__horizont_line_height
        # rt[1] = rt[1] - self.__horizont_line_height
        # rb[1] = rb[1] - self.__horizont_line_height
        # lt_dst[1] = lt_dst[1] - self.__horizont_line_height
        # lb_dst[1] = lb_dst[1] - self.__horizont_line_height
        # rt_dst[1] = rt_dst[1] - self.__horizont_line_height
        # rb_dst[1] = rb_dst[1] - self.__horizont_line_height
        #
        self.pts_src = np.array([
            [self.src_x_list[0].get(), self.src_y_list[0].get()],
            [self.src_x_list[1].get(), self.src_y_list[1].get()],
            [self.src_x_list[2].get(), self.src_y_list[2].get()],
            [self.src_x_list[3].get(), self.src_y_list[3].get()],
        ], dtype=np.float32)

        self.pts_dst = np.array([
            [self.dst_x_list[0].get(), self.dst_y_list[0].get()],
            [self.dst_x_list[1].get(), self.dst_y_list[1].get()],
            [self.dst_x_list[2].get(), self.dst_y_list[2].get()],
            [self.dst_x_list[3].get(), self.dst_y_list[3].get()],
        ], dtype=np.float32)
        # self.pts_dst = np.array([[lt_dst[0] + self.RECT_SIZE / 2, lt_dst[1] + self.RECT_SIZE / 2],
        #                          [lb_dst[0] + self.RECT_SIZE / 2, lb_dst[1] + self.RECT_SIZE / 2],
        #                          [rt_dst[0] + self.RECT_SIZE / 2, rt_dst[1] + self.RECT_SIZE / 2],
        #                          [rb_dst[0] + self.RECT_SIZE / 2, rb_dst[1] + self.RECT_SIZE / 2]], dtype=np.float32)

    def find_homography(self):
        self.fill_pts_from_sliders()
        print(self.pts_src, self.pts_dst)
        self.ipm_transformer.calc_homography(self.pts_src, self.pts_dst)

        h_img = cv2.resize(self.__src_image, (self.__img_width, self.__img_height))

        __img_ipm = self.ipm_transformer.get_ipm(h_img, is_mono=False,
                                                 horizont=self.__horizont_line_height)

        pil_img = Image.fromarray(__img_ipm)
        target_width = self.__img_width  # 400
        pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))
        return pil_img

    def open_image_dialog(self):
        # Позволяет пользователю выбрать файл, поддерживаются форматы изображений
        file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.jpg *.jpeg *.png *.gif *.bmp")])
        if file_path:
            self.load_image(file_path)

    def load_image(self, image_path):
        # Устанавливаем изображение из файла
        pil_img = Image.open(image_path)
        self.__img_height = int(pil_img.size[1] * (self.__img_width / pil_img.size[0]))
        pil_img = pil_img.resize((self.__img_width, self.__img_height))
        self.__src_image = np.array(pil_img)

        # Устанавливаем изображение для preview
        self.image_scale_coef = self.__img_prev_width / pil_img.size[0]
        pil_img = pil_img.resize((self.__img_prev_width, int(pil_img.size[1] * self.image_scale_coef)))
        self.__prev_canvas_image = itk.PhotoImage(pil_img)
        self.__img_prev_height = self.__prev_canvas_image.height()
        self.__img_prev_width = self.__prev_canvas_image.width()
        print(self.__img_width, self.__img_height)
        self.pts_src, self.pts_dst = self.calc_pts(self.__img_width, self.__img_height)

        self.place_elements()
        self.parameters()
        self.update_homography()
        self.wx.config(to=self.__img_height)

    def calc_pts(self, width, height):

        pts_src = np.array([[int(width / 4), int(height / 4)],
                            [int(width / 4), int(3 * height / 4)],
                            [int(3 * width / 4), int(height / 4)],
                            [int(3 * width / 4), int(3 * height / 4)]])

        pts_dst = np.copy(pts_src)
        for i in range(4):
            self.src_x_list[i].set(int(pts_src[i][0]))
            self.src_y_list[i].set(int(pts_src[i][1]))

            self.dst_x_list[i].set(int(pts_dst[i][0]))
            self.dst_y_list[i].set(int(pts_dst[i][1]))
        return pts_src, pts_dst


if __name__ == "__main__":
    app = ImageApp()
