import os
import json
import tkinter as tk
from tkinter import *
from tkinter import filedialog

import cv2
import yaml
from PIL import ImageTk as itk
import numpy as np
from PIL import Image, ImageTk

from ipm_transformer import IPMTransformer

DATACAMERA = "/home/hiber/ros2_ws/data/camera/buffer.png"
DATALIDAR = "/home/hiber/ros2_ws/data/lidar/buffer.json"

class ImageApp:
    ipm_transformer: IPMTransformer

    def __init__(self):
        self.__src_image = None
        self.__obstacle_image = None
        self.__prev_canvas_image = None
        self.__img_prev_height = None
        self.__img_prev_width = 800
        self.pts_src: np.array
        self.pts_dst: np.array
        self.__horizont_line_height = 100
        self.RECT_SIZE = 20
        self.image_scale_coef = 1
        
        self.adj_dst = {
            "lt_dst": ("lb_dst", "rt_dst"),
            "rt_dst": ("rb_dst", "lt_dst"),
            "lb_dst": ("lt_dst", "rb_dst"),
            "rb_dst": ("rt_dst", "lb_dst"),
        }
        self.adj = {
            "lt": "rt",
            "rt": "lt",
            "lb": "rb",
            "rb": "lb",
        }
        self.src_idx = {"lt": 0, "lb": 1, "rt": 2, "rb": 3}
        self.dst_idx = {"lt_dst": 0, "lb_dst": 1, "rt_dst": 2, "rb_dst": 3}

        self.rt, self.lt, self.lb, self.rb = None, None, None, None
        self.rt_dst, self.lt_dst, self.lb_dst, self.rb_dst = None, None, None, None

        self.__canvas_image = None
        self.__canvas_lidar = None

        self.__img_height = 800
        self.__img_width = 800
        self.ipm_transformer = IPMTransformer()
        self.obstacles_data = []

        self.root = tk.Tk()

        self.root.title("Calibration tool for IPM")  # title of the GUI window
        self.root.geometry("1650x1000")
        self.root.maxsize(1650, 1000)

        self.left_frame = Frame(self.root, width=1200, height=100)
        self.left_frame.grid(row=0, column=0, padx=10, pady=5, columnspan=2)

        self.center_frame = Frame(self.root, width=1700, height=1500)
        self.center_frame.grid(row=1, column=0, padx=10, pady=5)

        self.limits_container = Canvas(self.center_frame, width=1700, height=1500)
        self.limits_container.bind("<B1-Motion>", self.drag)
        self.limits_container.bind("<ButtonRelease-1>", self.my_callback)
        self.limits_container.grid(row=0, column=0)
        self.freeze = IntVar()
        self.freeze.set(0)
        self.check_update = Checkbutton(
            self.left_frame,
            text='Freeze',
            variable=self.freeze,
            command=self.update_freeze
        )
        self.load_lidar_button = Button(
            self.left_frame,
            text='Open a obstacles',
            command=self.open_lidar_dialog
        )
        self.load_config_button = Button(
            self.left_frame,
            text='Load config',
            command=self.load_config_file
        )
        self.save_config_button = Button(
            self.left_frame,
            text='Save config',
            command=self.save_config_file
        )
        self.varh = IntVar()
        self.varh.set(100)

        self.varx = IntVar()
        self.varx.set(0)
        self.vary = IntVar()
        self.vary.set(0)
        self.vars = DoubleVar()
        self.vars.set(1)

        self.varx_obs = IntVar()
        self.varx_obs.set(400)
        self.vary_obs = IntVar()
        self.vary_obs.set(400)
        self.vars_obs = DoubleVar()
        self.vars_obs.set(15)

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
        self.check_update.grid(row=0, column=0, padx=5, pady=5)
        self.load_lidar_button.grid(row=0, column=1, padx=5, pady=5)
        self.load_config_button.grid(row=0, column=2, padx=5, pady=5)
        self.save_config_button.grid(row=0, column=3, padx=5, pady=5)

        self.pts_src, self.pts_dst = self.calc_pts(self.__img_width, self.__img_height)


        self.parameters()
        self.root.after(0, self.load_data)
        self.root.mainloop()
    
    def update_freeze(self):
        if self.freeze.get() == 0:
            self.root.after(0, self.load_data)
    def load_data(self):
        if (self.freeze.get() == 0):
            try:
                self.load_image(DATACAMERA)
                self.load_obstacles(DATALIDAR)
                self.update_homography()
            except:
                pass
            self.root.after(500, self.load_data)


    def parameters(self):
        self.frame_config = Frame(self.root, borderwidth=1, relief=SOLID)
        self.frame_config.place(x=10, y=600)
        self.frame_config.config(width=600, height=600)

        self.frame_h = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_h.config(width=600, height=600)
        self.frame_h.grid(row=0, column=0, padx=5, pady=5)

        lx = Label(self.frame_h, text="Horizontal")
        lx.grid(row=1, column=0)

        self.wx = Scale(self.frame_h, from_=0, to=self.__img_height, orient=HORIZONTAL, variable=self.varh,
                        command=self.my_callback)
        self.wx.config(length=150)
        self.wx.grid(row=2, column=0)
        self.wx.bind("<ButtonRelease-1>", self.my_callback)

        self.wx.bind("<Button-1>", lambda event: event.widget.focus_set())

        self.frame_src = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_src.config(width=600, height=600)
        self.frame_src.grid(row=0, column=1, padx=5, pady=5)

        lx = Label(self.frame_src, text="SRC points")
        lx.grid(row=1, column=0, columnspan=2)

        self.frame_dst = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_dst.config(width=600, height=600)
        self.frame_dst.grid(row=0, column=2, padx=5, pady=5)

        lx = Label(self.frame_dst, text="DST points")
        lx.grid(row=1, column=0, columnspan=2)

        for i in range(0, 4):
            self.src_x_sliders.append(Scale(self.frame_src, from_=0, to=self.__img_width, orient=HORIZONTAL,
                                            variable=self.src_x_list[i], command=self.my_callback))
            self.src_y_sliders.append(Scale(self.frame_src, from_=0, to=self.__img_height, orient=HORIZONTAL,
                                            variable=self.src_y_list[i], command=self.my_callback))
            self.src_x_sliders[i].grid(row=i + 2, column=0)
            self.src_y_sliders[i].grid(row=i + 2, column=1)
            self.src_x_sliders[i].bind("<Button-1>", lambda event: event.widget.focus_set())
            self.src_y_sliders[i].bind("<Button-1>", lambda event: event.widget.focus_set())


            self.dst_x_sliders.append(Scale(self.frame_dst, from_=0, to=self.__img_width, orient=HORIZONTAL,
                                            variable=self.dst_x_list[i], command=self.my_callback))
            self.dst_y_sliders.append(Scale(self.frame_dst, from_=0, to=self.__img_height, orient=HORIZONTAL,
                                            variable=self.dst_y_list[i], command=self.my_callback))
            self.dst_x_sliders[i].bind("<Button-1>", lambda event: event.widget.focus_set())
            self.dst_y_sliders[i].bind("<Button-1>", lambda event: event.widget.focus_set())
            self.dst_x_sliders[i].grid(row=i + 2, column=0)
            self.dst_y_sliders[i].grid(row=i + 2, column=1)

        self.frame_pos = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_pos.config(width=600, height=600)
        self.frame_pos.grid(row=0, column=3, padx=5, pady=5)

        lx = Label(self.frame_pos, text="position")
        lx.grid(row=1, column=0, columnspan=2)
        
        lx = Label(self.frame_pos, text="X")
        lx.grid(row=2, column=0, columnspan=2)
        self.posx = Scale(self.frame_pos, from_=0, to=self.__img_width, orient=HORIZONTAL, variable=self.varx,
                        command=self.my_callback)
        self.posx.config(length=150)
        self.posx.grid(row=2, column=1)
        self.posx.bind("<ButtonRelease-1>", self.my_callback)
        self.posx.bind("<Button-1>", lambda event: event.widget.focus_set())

        lx = Label(self.frame_pos, text="Y")
        lx.grid(row=3, column=0, columnspan=2)

        self.posy = Scale(self.frame_pos, from_=0, to=self.__img_height, orient=HORIZONTAL, variable=self.vary,
                        command=self.my_callback)
        self.posy.config(length=150)
        self.posy.grid(row=3, column=1)
        self.posy.bind("<ButtonRelease-1>", self.my_callback)
        self.posy.bind("<Button-1>", lambda event: event.widget.focus_set())

        lx = Label(self.frame_pos, text="S")
        lx.grid(row=4, column=0, columnspan=2)
        self.scale = Scale(self.frame_pos, digits=3, resolution=0.01, from_=0.01, to=1, orient=HORIZONTAL, variable=self.vars,
                        command=self.my_callback)
        self.scale.config(length=150)
        self.scale.grid(row=4, column=1)
        self.scale.bind("<ButtonRelease-1>", self.my_callback)
        self.scale.bind("<Button-1>", lambda event: event.widget.focus__v_lineframe_set())


        self.frame_pos_obs = Frame(self.frame_config, borderwidth=1, relief=SOLID)
        self.frame_pos_obs.config(width=600, height=600)
        self.frame_pos_obs.grid(row=1, column=0, columnspan=4, padx=5, pady=5)

        lx = Label(self.frame_pos_obs, text="position")
        lx.grid(row=1, column=0, columnspan=2)
        
        lx = Label(self.frame_pos_obs, text="X")
        lx.grid(row=2, column=0, columnspan=2)
        self.posx_obs = Scale(self.frame_pos_obs, from_=0, to=self.__img_width, orient=HORIZONTAL, variable=self.varx_obs,
                        command=self.my_callback)
        self.posx_obs.config(length=150)
        self.posx_obs.grid(row=2, column=1)
        self.posx_obs.bind("<ButtonRelease-1>", self.my_callback)
        self.posx_obs.bind("<Button-1>", lambda event: event.widget.focus_set())

        lx = Label(self.frame_pos_obs, text="Y")
        lx.grid(row=3, column=0, columnspan=2)

        self.posy_obs = Scale(self.frame_pos_obs, from_=0, to=self.__img_height, orient=HORIZONTAL, variable=self.vary_obs,
                        command=self.my_callback)
        self.posy_obs.config(length=150)
        self.posy_obs.grid(row=3, column=1)
        self.posy_obs.bind("<ButtonRelease-1>", self.my_callback)
        self.posy_obs.bind("<Button-1>", lambda event: event.widget.focus_set())

        lx = Label(self.frame_pos_obs, text="S")
        lx.grid(row=4, column=0, columnspan=2)
        self.scale_obs = Scale(self.frame_pos_obs, from_=1, to=20, orient=HORIZONTAL, variable=self.vars_obs,
                        command=self.my_callback)
        self.scale_obs.config(length=150)
        self.scale_obs.grid(row=4, column=1)
        self.scale_obs.bind("<ButtonRelease-1>", self.my_callback)
        self.scale_obs.bind("<Button-1>", lambda event: event.widget.focus_set())

    def update_parameters(self):
        self.wx.config(to=self.__img_height)
        for i in range(0, 4):
            self.src_x_sliders[i].config(to=self.__img_width)
            self.src_y_sliders[i].config(to=self.__img_height)
            self.dst_x_sliders[i].config(to=self.__img_width)
            self.dst_y_sliders[i].config(to=self.__img_height)

    def change_var(self, var):
        self.place_elements()
        self.show_homography()
        self.update_parameters()

    def my_callback(self, var):
        self.place_elements()
        self.update_homography()
        self.update_parameters()

    def place_elements(self):
        self.limits_container.delete('all')
        self.limits_container.create_image(0, 0, image=self.__prev_canvas_image, anchor=NW, tags="preview")
        self.limits_container.config(width=1650,
                                     height=1000)
        self.__cut_line = self.limits_container.create_line(0, self.varh.get(), self.__img_prev_width,
                                                            self.varh.get(), tags="__cut_line",
                                                            fill="red",
                                                            width=2)
        self.lt = self.limits_container.create_rectangle(self.src_x_list[0].get(), self.src_y_list[0].get(),
                                                         self.src_x_list[0].get() + self.RECT_SIZE,
                                                         self.src_y_list[0].get() + self.RECT_SIZE, tags="lt",
                                                         fill="red")
        self.lb = self.limits_container.create_rectangle(self.src_x_list[1].get(), self.src_y_list[1].get(),
                                                         self.src_x_list[1].get() + self.RECT_SIZE,
                                                         self.src_y_list[1].get() + self.RECT_SIZE, tags="lb",
                                                         fill="green")
        self.rt = self.limits_container.create_rectangle(self.src_x_list[2].get(), self.src_y_list[2].get(),
                                                         self.src_x_list[2].get() + self.RECT_SIZE,
                                                         self.src_y_list[2].get() + self.RECT_SIZE, tags="rt",
                                                         fill="blue")
        self.rb = self.limits_container.create_rectangle(self.src_x_list[3].get(), self.src_y_list[3].get(),
                                                         self.src_x_list[3].get() + self.RECT_SIZE,
                                                         self.src_y_list[3].get() + self.RECT_SIZE, tags="rb",
                                                         fill="yellow")
        self.lt_dst = self.limits_container.create_oval(self.dst_x_list[0].get(), self.dst_y_list[0].get(),
                                                        self.dst_x_list[0].get() + self.RECT_SIZE,
                                                        self.dst_y_list[0].get() + self.RECT_SIZE, tags="lt_dst",
                                                        fill="orangered1")
        self.lb_dst = self.limits_container.create_oval(self.dst_x_list[1].get(), self.dst_y_list[1].get(),
                                                        self.dst_x_list[1].get() + self.RECT_SIZE,
                                                        self.dst_y_list[1].get() + self.RECT_SIZE, tags="lb_dst",
                                                        fill="lightseagreen")
        self.rt_dst = self.limits_container.create_oval(self.dst_x_list[2].get(), self.dst_y_list[2].get(),
                                                        self.dst_x_list[2].get() + self.RECT_SIZE,
                                                        self.dst_y_list[2].get() + self.RECT_SIZE, tags="rt_dst",
                                                        fill="lightblue")
        self.rb_dst = self.limits_container.create_oval(self.dst_x_list[3].get(), self.dst_y_list[3].get(),
                                                        self.dst_x_list[3].get() + self.RECT_SIZE,
                                                        self.dst_y_list[3].get() + self.RECT_SIZE, tags="rb_dst",
                                                        fill="lightyellow1")
        self.__v_line = self.limits_container.create_line(0, 0, self.__img_height, 0, fill="gray", tags="__v_line",
                                                          width=1)
        self.__h_line = self.limits_container.create_line(0, 0, 0, self.__img_width, fill="gray", tags="__h_line",
                                                          width=1)

    def show_homography(self):
        self.limits_container.create_image(self.__img_prev_width + 20 + self.varx.get(), self.vary.get(), image=self.photo, anchor=NW,
                                           tags="image")
        self.print_obstacles()

    def update_homography(self):
        photo = self.find_homography()
        photo = photo.resize( [int(self.vars.get() * s) for s in photo.size] )
        self.photo = ImageTk.PhotoImage(photo)
        self.show_homography()


    def fill_pts_from_sliders(self):
        self.__horizont_line_height = self.varh.get()
        
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

        # поправка на отрезанный горизонт Important Very
        for i in range(len(self.pts_src)):
            self.pts_src[i][1] -= self.__horizont_line_height
            self.pts_dst[i][1] -= self.__horizont_line_height
    
    def load_config_file(self):
        filename = filedialog.askopenfilename(filetypes=[("YAML config", "*.yaml")])
        self.load_config(filename)
        self.update_sliders()
        self.place_elements()
        self.update_homography()


    def save_config_file(self):
        filename = filedialog.asksaveasfile(filetypes=[("YAML config", "*.yaml")])
        self.save_config(filename.name)

    def save_config(self, filename):
        self.fill_pts_from_sliders()
        src = self.pts_src.copy()
        src[:, 1] = src[:, 1] + self.__horizont_line_height
        dst = self.pts_dst.copy()
        dst[:, 1] = dst[:, 1] + self.__horizont_line_height
        config = {
            'homography': self.ipm_transformer.get_homography_matrix().tolist(),
            'horizont': self.__horizont_line_height,
            'src_points': src.tolist(),
            'dst_points': dst.tolist(),
            'height': self.__img_height,
            'width': self.__img_width,
            'posx': self.varx.get(),
            'posy': self.vary.get(),
            'scale': self.vars.get(),
        }
        with open(filename, 'w') as file:
            documents = yaml.dump(config, file, default_flow_style=False)

    def load_config(self, filename):
        if not os.path.exists(filename):
            print('Config file not found. Use default values')
            return
        with open(filename) as file:
            config = yaml.full_load(file)
        self.ipm_transformer = IPMTransformer(np.array(config['homography']))
        self.__horizont_line_height = config['horizont']
        self.pts_src = np.array(config['src_points'])
        self.pts_dst = np.array(config['dst_points'])
        self.__img_height = config['height']
        self.__img_width = config['width']
        try:
            self.varx.set(config['posx'])
            self.vary.set(config['posy'])
            self.vars.set(config['scale'])
        except:
            pass


    def find_homography(self):
        self.fill_pts_from_sliders()
        self.ipm_transformer.calc_homography(self.pts_src, self.pts_dst)
        h_img = cv2.resize(self.__src_image, (self.__img_width, self.__img_height))
        self.__img_ipm = self.ipm_transformer.get_ipm(h_img, is_mono=False,
                                                      horizont=self.__horizont_line_height) 
        pil_img = Image.fromarray(self.__img_ipm)
        target_width = self.__img_width  # 800
        pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))
        return pil_img

    # def open_image_dialog(self):
    #     # Позволяет пользователю выбрать файл, поддерживаются форматы изображений
    #     file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.jpg *.jpeg *.png *.gif *.bmp")])
    #     if file_path:
    #         self.load_image(file_path)

    def open_lidar_dialog(self):
        # Позволяет пользователю выбрать файл, поддерживаются форматы изображений
        file_path = filedialog.askopenfilename(filetypes=[("Obstacles files", "*.json")])
        if file_path:
            self.load_obstacles(file_path)

    def load_obstacles(self, file_path):
        if not os.path.exists(file_path):
            print('Config file not found. Use default values')
            return
        with open(file_path) as file:
            self.obstacles_data = json.load(file)["data"]
        
        self.update_homography()

    def print_obstacles(self, scale=15):

        p1 = [0, 0]
        p2 = [0, 0]
        p3 = [0, 0]
        p4 = [0, 0]
        self.img = np.zeros(shape=(1024, 1024, 4), dtype=np.uint8)

        for p in self.obstacles_data:
            figure = (p[4], p[5], p[6], p[7])
            p1[0] = int(self.varx_obs.get() - figure[0][0] * self.scale_obs.get()) # Вычисляем X первой точки препятствия на нашем рисунке
            p1[1] = int(self.vary_obs.get() - figure[0][1] * self.scale_obs.get()) # Вычисляем Y первой точки препятствия на нашем рисунке
            p4[0] = int(self.varx_obs.get() - figure[1][0] * self.scale_obs.get())
            p4[1] = int(self.vary_obs.get() - figure[1][1] * self.scale_obs.get())
            p2[0] = int(self.varx_obs.get() - figure[2][0] * self.scale_obs.get())
            p2[1] = int(self.vary_obs.get() - figure[2][1] * self.scale_obs.get())
            p3[0] = int(self.varx_obs.get() - figure[3][0] * self.scale_obs.get())
            p3[1] = int(self.vary_obs.get() - figure[3][1] * self.scale_obs.get())
            # Рисуем текст - номер препятствия
            # cv2.putText(self.img, "Fig #" + str(p[0]), (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0, 255))
            # Рисуем рамку препятствия
            cv2.line(self.img, (p1[0], p1[1]), (p2[0], p2[1]), (255,0,0, 255), 1);
            cv2.line(self.img, (p2[0], p2[1]), (p3[0], p3[1]), (255,0,0, 255), 1);
            cv2.line(self.img, (p3[0], p3[1]), (p4[0], p4[1]), (255,0,0, 255), 1);
            cv2.line(self.img, (p4[0], p4[1]), (p1[0], p1[1]), (255,0,0, 255), 1);
        
        self.img = cv2.flip(self.img, 1)
        self.__obstacle_image = Image.fromarray(self.img)
        self.obstacle_photo = ImageTk.PhotoImage(self.__obstacle_image)
        self.limits_container.create_image(self.__img_prev_width + 20, 0, image=self.obstacle_photo, anchor=NW,
                                           tags="imageO")
    
        
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
        # self.pts_src, self.pts_dst = self.calc_pts(self.__img_width, self.__img_height)

        self.place_elements()
        self.update_parameters()
        # self.update_homography()
        self.wx.config(to=self.__img_height)

    def update_sliders(self):
        self.varh.set(int(self.__horizont_line_height))
        for i in range(4):
            self.src_x_list[i].set(int(self.pts_src[i][0]))
            self.src_y_list[i].set(int(self.pts_src[i][1]))

            self.dst_x_list[i].set(int(self.pts_dst[i][0]))
            self.dst_y_list[i].set(int(self.pts_dst[i][1]))

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
    
    def sync_elements(self):
        self.limits_container.moveto(self.__cut_line, -3, self.varh.get() - 3)
        self.limits_container.moveto(self.lt, self.src_x_list[0].get() - 1, self.src_y_list[0].get() - 1)
        self.limits_container.moveto(self.lb, self.src_x_list[1].get() - 1, self.src_y_list[1].get() - 1)
        self.limits_container.moveto(self.rt, self.src_x_list[2].get() - 1, self.src_y_list[2].get() - 1)
        self.limits_container.moveto(self.rb, self.src_x_list[3].get() - 1, self.src_y_list[3].get() - 1)

        self.limits_container.moveto(self.lt_dst, self.dst_x_list[0].get() - 1, self.dst_y_list[0].get() - 1)
        self.limits_container.moveto(self.lb_dst, self.dst_x_list[1].get() - 1, self.dst_y_list[1].get() - 1)
        self.limits_container.moveto(self.rt_dst, self.dst_x_list[2].get() - 1, self.dst_y_list[2].get() - 1)
        self.limits_container.moveto(self.rb_dst, self.dst_x_list[3].get() - 1, self.dst_y_list[3].get() - 1)

    def drag(self, event):
        print(self.limits_container.gettags("current"))
        # Заморозка положения картинки
        if self.limits_container.gettags("current") in (self.limits_container.gettags("image"),
                                                         self.limits_container.gettags("imageO"),
                                                         self.limits_container.gettags("preview")):
            return

        # Перемещение
        if self.limits_container.gettags(self.__cut_line) == self.limits_container.gettags("current"):
            self.varh.set(event.y - 1)
            self.src_y_list[0].set(self.varh.get())
            self.src_y_list[2].set(self.varh.get())
        # Зависимое перемещение _dst точек
        if self.limits_container.gettags("current") in (
                self.limits_container.gettags(self.lt_dst), self.limits_container.gettags(self.rt_dst),
                self.limits_container.gettags(self.lb_dst), self.limits_container.gettags(self.rb_dst)):
            curtag = self.limits_container.gettags("current")[0]
            save_x_tag, save_y_tag = self.adj_dst[curtag]
            save_x_point, save_y_point = self.limits_container.coords(save_x_tag), self.limits_container.coords(save_y_tag)
            
            self.dst_x_list[self.dst_idx[curtag]].set(event.x)
            self.dst_y_list[self.dst_idx[curtag]].set(event.y)

            self.dst_x_list[self.dst_idx[save_x_tag]].set(event.x)
            self.dst_y_list[self.dst_idx[save_x_tag]].set(save_x_point[1])

            self.dst_x_list[self.dst_idx[save_y_tag]].set(save_y_point[0])
            self.dst_y_list[self.dst_idx[save_y_tag]].set(event.y)

        if self.limits_container.gettags("current") in (
                self.limits_container.gettags(self.lt), self.limits_container.gettags(self.rt),
                self.limits_container.gettags(self.lb), self.limits_container.gettags(self.rb)):
            curtag = self.limits_container.gettags("current")[0]
            self.src_x_list[self.src_idx[curtag]].set(event.x)
            self.src_y_list[self.src_idx[curtag]].set(event.y)

        self.limits_container.coords(self.__h_line, 0, event.y, self.__img_width, event.y)
        self.limits_container.coords(self.__v_line, event.x, 0, event.x, self.__img_width)

        self.sync_elements()



if __name__ == "__main__":
    app = ImageApp()
