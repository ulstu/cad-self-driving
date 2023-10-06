from tkinter import *
from tkinter import filedialog as fd
from tkinter.filedialog import asksaveasfile
from PIL import ImageTk as itk
from PIL import Image
from fastseg.image import colorize, blend
import numpy as np
import cv2
import yaml
import os

class IPMTransformer(object):
    def __init__(self, homography_matrix=None) -> None:
        self.__h = homography_matrix

    def calc_homography(self, pts_src, pts_dst):
        self.__h, status = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC)
        return self.__h
    
    def get_homography_matrix(self):
        return self.__h

    def get_ipm(self, im_src, dst_size=(1200, 800), horizont=0, is_mono=False):
        im_src = im_src[horizont:,:]
        im_dst = cv2.warpPerspective(im_src, self.__h, dst_size)
        if not is_mono:
            rows = np.sum(im_dst, axis=(1, 2))
            cols = np.sum(im_dst, axis=(0, 2))
        else:
            rows = np.sum(im_dst, axis=(1))
            cols = np.sum(im_dst, axis=(0))

        nonzero_rows = len(rows[np.nonzero(rows)])
        nonzero_cols = len(cols[np.nonzero(cols)])
        im_dst = im_dst[:nonzero_rows,:nonzero_cols]
        im_dst = cv2.resize(im_dst, (im_dst.shape[1], im_dst.shape[1]), interpolation= cv2.INTER_AREA)
        return im_dst


class CalibrationTool(object):
    def __init__(self) -> None:
        self.__img_width = 800
        self.__img_height = 800
        self.__src_image = None
        self.__canvas_image = None
        self.__horizont_line_height = 20
        self.ipm_transformer = IPMTransformer()
        self.RECT_SIZE = 15
        self.__img_filetype = None
        self.__img_filename = None

        self.rt, self.lt, self.lb, self.rb = None, None, None, None
        self.rt_dst, self.lt_dst, self.lb_dst, self.rb_dst = None, None, None, None

        self.root = Tk()

        self.root.title("Calibration tool for IPM")  # title of the GUI window
        self.root.maxsize(1500, 1000)  # specify the max size the window can expand to

        self.left_frame = Frame(self.root, width=1000, height=100)
        self.left_frame.grid(row=0, column=0, padx=10, pady=5, columnspan=2)

        self.center_frame = Frame(self.root, width=500, height=400)
        self.center_frame.grid(row=1, column=0, padx=10, pady=5)

        self.open_image_button = Button(
            self.left_frame,
            text='Open an image',
            command=self.load_image
        )

        self.find_homography_button = Button(
            self.left_frame,
            text='Warp it!',
            command=self.find_homography
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

        self.warp_depth_button = Button(
            self.left_frame,
            text='Warp depth',
            command=self.transform_depth
        )

        self.open_image_button.grid(row=0, column=0, padx=5, pady=5)
        self.find_homography_button.grid(row=0, column=1, padx=5, pady=5)
        self.load_config_button.grid(row=0, column=2, padx=5, pady=5)
        self.save_config_button.grid(row=0, column=3, padx=5, pady=5)
        self.warp_depth_button.grid(row=0, column=4, padx=5, pady=5)

        self.image_placeholder = Canvas(self.center_frame)
        self.image_placeholder.grid(row=0,column=0, padx=5, pady=5)
        self.image_placeholder.bind("<B1-Motion>", self.drag)

        self.__src_point_placeholder = Label(self.center_frame, text="...")
        self.__src_point_placeholder.grid(row=1,column=0, padx=5, pady=5)
        self.root.mainloop()

    def calc_pts(self, width, height):
        pts_src = np.array([[int(width / 4), int(height / 4)],
                                 [int(width / 4), int(3 * height / 4)],
                                 [int(3 * width / 4), int(height / 4)],
                                 [int(3 * width / 4), int(3 * height / 4)]])
        pts_dst = np.copy(pts_src)
        return pts_src, pts_dst

    def save_config(self, filename):
        src = self.pts_src.copy()
        src[:,1] = src[:,1] + self.__horizont_line_height
        dst = self.pts_dst.copy()
        dst[:,1] = dst[:,1] + self.__horizont_line_height
        config = {
            'homography': self.ipm_transformer.get_homography_matrix().tolist(),
            'horizont': self.__horizont_line_height,
            'src_points': src.tolist(),
            'dst_points': dst.tolist(),
            'height': self.__img_height,
            'width': self.__img_width
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
        
    def load_config_file(self):
        filename = fd.askopenfilename(filetypes=[("YAML config", "*.yaml")])
        self.load_config(filename)
        self.place_control_elements()

    def save_config_file(self):
        filename = fd.asksaveasfile(filetypes=[("YAML config", "*.yaml")])
        self.save_config(filename.name)

    def transform_depth(self):
        depth_map = np.load(self.__img_filename.replace('_seg.npy', '_range.npy'))
        depth_map = depth_map[self.__horizont_line_height:,:]
        output_size = (depth_map.shape[0], depth_map.shape[1])

        # Создаем пустое изображение для коррекции глубины
        corrected_depth = np.zeros_like(self.__img_ipm)

        # Получаем высоту и ширину изображения
        height, width = depth_map.shape[:2]

        # Коррекция глубины с учетом преобразования
        for y in range(height):
            for x in range(width):
                # Применяем трансформацию глубины в соответствии с преобразованием IPM
                x_corrected, y_corrected = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), self.ipm_transformer.get_homography_matrix())[0][0]
                x_corrected, y_corrected = int(x_corrected), int(y_corrected)

                if 0 <= x_corrected < output_size[1] and 0 <= y_corrected < output_size[0]:
                    corrected_depth[y_corrected, x_corrected] = depth_map[y, x]
        cv2.imshow('h depth', depth_map)
        cv2.imshow('source depth', depth_map)
        cv2.imshow('corrected depth', corrected_depth)
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        return corrected_depth

    def fill_pts_from_canvas(self):
        self.__horizont_line_height = int(self.image_placeholder.coords(self.__cut_line)[1])
        lt = self.image_placeholder.coords(self.lt)
        lb = self.image_placeholder.coords(self.lb)
        rt = self.image_placeholder.coords(self.rt)
        rb = self.image_placeholder.coords(self.rb)
        lt_dst = self.image_placeholder.coords(self.lt_dst)
        lb_dst = self.image_placeholder.coords(self.lb_dst)
        rt_dst = self.image_placeholder.coords(self.rt_dst)
        rb_dst = self.image_placeholder.coords(self.rb_dst)
        
        # поправка на отрезанный горизонт
        lt[1] = lt[1] - self.__horizont_line_height
        lb[1] = lb[1] - self.__horizont_line_height
        rt[1] = rt[1] - self.__horizont_line_height
        rb[1] = rb[1] - self.__horizont_line_height
        lt_dst[1] = lt_dst[1] - self.__horizont_line_height
        lb_dst[1] = lb_dst[1] - self.__horizont_line_height
        rt_dst[1] = rt_dst[1] - self.__horizont_line_height
        rb_dst[1] = rb_dst[1] - self.__horizont_line_height
        
        self.pts_src = np.array([[lt[0] + self.RECT_SIZE / 2, lt[1] + self.RECT_SIZE / 2],
                        [lb[0] + self.RECT_SIZE / 2, lb[1] + self.RECT_SIZE / 2],
                        [rt[0] + self.RECT_SIZE / 2, rt[1] + self.RECT_SIZE / 2],
                        [rb[0] + self.RECT_SIZE / 2, rb[1] + self.RECT_SIZE / 2]], dtype=np.float32)
        self.pts_dst = np.array([[lt_dst[0] + self.RECT_SIZE / 2, lt_dst[1] + self.RECT_SIZE / 2],
                        [lb_dst[0] + self.RECT_SIZE / 2, lb_dst[1] + self.RECT_SIZE / 2],
                        [rt_dst[0] + self.RECT_SIZE / 2, rt_dst[1] + self.RECT_SIZE / 2],
                        [rb_dst[0] + self.RECT_SIZE / 2, rb_dst[1] + self.RECT_SIZE / 2]], dtype=np.float32)
        
        print(self.pts_src, self.pts_dst, self.__horizont_line_height)

    def find_homography(self):
        self.fill_pts_from_canvas()
        self.ipm_transformer.calc_homography(self.pts_src, self.pts_dst)
        h_img = cv2.resize(self.__src_image, (self.__img_width, self.__img_height))
        self.__img_ipm = self.ipm_transformer.get_ipm(h_img, is_mono=self.__img_filetype == "npy", horizont=self.__horizont_line_height)
        if (self.__img_filetype == "npy"):
            colorized = colorize(self.__img_ipm)
            colorized = np.asarray(colorized)
            pil_img = Image.fromarray(colorized)
        else:
            pil_img = Image.fromarray(self.__img_ipm)
        target_width = self.__img_width #400
        pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))
        pil_img.show(title="Converted image")

    def place_control_elements(self):
        self.image_placeholder.delete('all')
        self.image_placeholder.config(width = self.__img_width, height = self.__img_height)
        self.image_placeholder.create_image(0, 0, image = self.__canvas_image, anchor=NW)
        self.lt = self.image_placeholder.create_rectangle(self.pts_src[0][0], self.pts_src[0][1], self.pts_src[0][0] + self.RECT_SIZE, self.pts_src[0][1] + self.RECT_SIZE, fill="red")
        self.lb = self.image_placeholder.create_rectangle(self.pts_src[1][0], self.pts_src[1][1], self.pts_src[1][0] + self.RECT_SIZE, self.pts_src[1][1] + self.RECT_SIZE, fill="green")
        self.rt = self.image_placeholder.create_rectangle(self.pts_src[2][0], self.pts_src[2][1], self.pts_src[2][0] + self.RECT_SIZE, self.pts_src[2][1] + self.RECT_SIZE, fill="blue")
        self.rb = self.image_placeholder.create_rectangle(self.pts_src[3][0], self.pts_src[3][1], self.pts_src[3][0] + self.RECT_SIZE, self.pts_src[3][1] + self.RECT_SIZE, fill="yellow")

        self.lt_dst = self.image_placeholder.create_oval(self.pts_dst[0][0], self.pts_dst[0][1], self.pts_dst[0][0] + self.RECT_SIZE, self.pts_dst[0][1] + self.RECT_SIZE, fill="orangered1")
        self.lb_dst = self.image_placeholder.create_oval(self.pts_dst[1][0], self.pts_dst[1][1], self.pts_dst[1][0] + self.RECT_SIZE, self.pts_dst[1][1] + self.RECT_SIZE, fill="lightseagreen")
        self.rt_dst = self.image_placeholder.create_oval(self.pts_dst[2][0], self.pts_dst[2][1], self.pts_dst[2][0] + self.RECT_SIZE, self.pts_dst[2][1] + self.RECT_SIZE, fill="lightblue")
        self.rb_dst = self.image_placeholder.create_oval(self.pts_dst[3][0], self.pts_dst[3][1], self.pts_dst[3][0] + self.RECT_SIZE, self.pts_dst[3][1] + self.RECT_SIZE, fill="lightyellow1")
        self.__v_line = self.image_placeholder.create_line(0, 0, self.__img_height, 0, fill="gray", width=1)
        self.__h_line = self.image_placeholder.create_line(0, 0, 0, self.__img_width, fill="gray", width=1)
        self.__cut_line = self.image_placeholder.create_line(0, self.__horizont_line_height, self.__img_width, self.__horizont_line_height, fill="red", width=2)

    def load_image(self):
        self.__img_filename = fd.askopenfilename(filetypes=[("Numpy Array", "*.npy"), ("Image files", "*.jpeg *.png *.jpg")])
        self.__img_filetype = self.__img_filename.split(".")[-1]
        if (self.__img_filetype == "npy"):
            self.__src_image = np.load(self.__img_filename).astype(np.float32)
            colorized = colorize(self.__src_image)
            colorized = np.asarray(colorized)
            pil_img = Image.fromarray(colorized)
        else:
            pil_img = Image.open(self.__img_filename)
            self.__src_image = np.array(pil_img)

        pil_img = pil_img.resize((self.__img_width, int(pil_img.size[1] * self.__img_width / pil_img.size[0])))
        self.__canvas_image = itk.PhotoImage(pil_img)
        self.__img_height = self.__canvas_image.height()
        self.__img_width = self.__canvas_image.width()
        self.pts_src, self.pts_dst = self.calc_pts(self.__img_width, self.__img_height)
        self.place_control_elements()


    def drag(self, event):
        self.image_placeholder.moveto("current", event.x - int(self.RECT_SIZE / 2), event.y - int(self.RECT_SIZE / 2))
        self.image_placeholder.coords(self.__h_line, 0, event.y, self.__img_width, event.y)
        self.image_placeholder.coords(self.__v_line, event.x, 0, event.x, self.__img_width)
        self.pts_src, self.pts_dst = self.calc_pts(self.__img_width, self.__img_height)

CalibrationTool()

