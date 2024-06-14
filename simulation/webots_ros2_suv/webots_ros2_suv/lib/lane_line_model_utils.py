import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon
from timeit import default_timer as timer
import time
import os
import re
import shutil
import yaml

# default_palette = [
#     (255, 0, 0),
#     (0, 255, 0),
#     (0, 0, 255),
#     (0, 255, 255), 
#     (255, 0, 255), 
#     (255, 255, 0), 
#     (150, 255, 255)
# ]

default_palette = [
    (0, 0, 0), # unkown #000000
    (85, 172, 238), # white-dash #55acee
    (120, 177, 89), # white-solid #78b159
    (13, 90, 2), # double-white-dash #0d5a02
    (221, 46, 68), # double-white-solid #dd2e44
    (194, 190, 255), # white-ldash-rsolid #c2beff
    (130, 233, 255), # white-lsolid-rdash #82e9ff
    (172, 168, 0), # yellow-dash #aca800
    (253, 203, 88), # yellow-solid #fdcb58
    (44, 25, 255), # double-yellow-dash #2c19ff
    (244, 144, 12), # double-yellow-solid #f4900c
    (221, 46, 68), # yellow-ldash-rsolid #dd2e44
    (255, 79, 204), # yellow-lsolid-rdash #ff4fcc
    (109, 0, 109), # left-curbside #6d006d
    (0, 109, 109), # right-curbside #006d6d
]


class LaneLine():
    def __init__(self, points: np.ndarray, label: int, points_n: np.ndarray = [], elapsed_time=0, mask_count_points=0):
        self.points = points
        self.points_n = points_n
        self.label = label
        self.elapsed_time = elapsed_time
        self.mask_count_points=mask_count_points


class LaneMask():
    def __init__(self, points: np.ndarray = [], points_n: np.ndarray = [], label: int = [], orig_shape: np.ndarray = None):
        self.points = points
        self.label = label
        self.orig_shape = orig_shape
        self.points_n = points_n


    @staticmethod
    def from_predictions(predictions, tolerance=0) -> list:
        mask_batches = []
        for prediction in predictions:
            lane_masks = []
            if prediction.masks is not None:
                for (xy, xyn, cls) in zip(prediction.masks.xy, prediction.masks.xyn, prediction.boxes.cls):
                    if tolerance > 0:
                        simplified_polygon = Polygon(xyn).simplify(tolerance, preserve_topology=True)
                        points_n = np.array(simplified_polygon.exterior.coords)
                    else:
                        points_n = xyn
                    label = int(cls)

                    lane_mask = LaneMask(xy, points_n, label, prediction.masks.orig_shape)
                    lane_masks.append(lane_mask)
            mask_batches.append(lane_masks)
        return mask_batches
    

    @staticmethod
    def from_line_to_mask(line: LaneLine, shape=(1920, 1080), tolerance=0.0015):
        if line.points.shape[0] <= 0:
            return None

        # shape = (shape[1], shape[0])

        image = np.zeros((int(shape[1]), int(shape[0]), 1), dtype=np.uint8)
        
        if line.points.shape[0] == 1:
            cv2.circle(image, (line.points[0] * np.array(shape)).astype(int), 10, (255), -1) # You may need to adjust the radius
        else:
            for idx in range(1, len(line.points)):
                cv2.line(image, 
                        (line.points[idx - 1]).astype(int),
                        (line.points[idx]).astype(int), 
                        color=(255), 
                        thickness=20) # You may need to adjust the thickness
        
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        points = np.array(contours[0]).reshape(-1, 2).astype(np.float32)
        points /= np.array(shape).astype(np.float32)

        simplified_polygon = Polygon(points).simplify(tolerance, preserve_topology=True)
        points_n = np.array(simplified_polygon.exterior.coords)

        points = points_n * np.array(shape).astype(np.float32)
        points = points.astype(np.int32)

        lane_mask = LaneMask(points, points_n, line.label, shape)
        return lane_mask
    

    def from_line_batches_to_mask_batches(line_batches: list, orig_shape) -> list:
        mask_batches = []
        for lines in line_batches:
            mask_batches.append([])
            for line in lines:
                lane_mask = LaneMask.from_line_to_mask(line, orig_shape, 0.0015)
                # lane_mask = LaneMask(line.points, line.points_n, line.label, orig_shape)
                mask_batches[-1].append(lane_mask)
        return mask_batches
    

    def from_file(file_path: str, orig_shape = None, image: np.ndarray = None, image_path: str = None) -> list:
        masks = []
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                # values = re.findall(r'\d+', line)
                values = re.findall(r"[-+]?(?:\d*\.*\d+)", line)
                label = int(values[0])
                points_n = []

                for idx in range(1, len(values[1:]), 2):
                    point = [float(values[idx]), float(values[idx + 1])]
                    points_n.append(point)
                points_n = np.array(points_n)

                shape = None
                if orig_shape is not None:
                    shape = orig_shape
                elif image is not None:
                    shape = (image.shape[0], image.shape[1])
                elif image_path is not None:
                    new_image = cv2.imread(image_path)
                    shape = (new_image.shape[0], new_image.shape[1])
                
                points = None
                if shape is not None:
                    points = points_n.copy()
                    # points = points_n
                    # points[:, 0] *= shape[0]
                    # points[:, 1] *= shape[1]
                    points[:, 0] *= shape[1]
                    points[:, 1] *= shape[0]
                # points = np.array(points, dtype=int)

                lane_mask = LaneMask(points, points_n, label, shape)
                masks.append(lane_mask)
        return masks
    

    def substitute_label(self, substitutions: list):
        if str(self.label) in substitutions:
            self.label = int(substitutions[str(self.label)])
    

    @staticmethod
    def substitute_labels(masks, substitutions: list):
        for mask in masks:
            mask.substitute_label(substitutions)
    

    @staticmethod
    def generate_plot(masks, image=None, image_path=None, mask_alpha=0.2, draw_lines_bool=True):
        # image_to_draw = np.copy(image)
        # batch_lines = get_lines([masks])
        # draw_segmentation(image_to_draw, [masks])
        # draw_lines(image_to_draw, batch_lines)
        if image is None:
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        mask_batches = [masks]
        batch_lines = get_lines(mask_batches)

        images_to_draw = np.copy([image])
        draw_segmentation(images_to_draw, mask_batches, mask_alpha)
        if draw_lines_bool:
            draw_lines(images_to_draw, batch_lines)

        return images_to_draw[0]
    

    @staticmethod
    def visualize_masks(masks=None, image=None, masks_path: str = None, image_path: str = None, mask_alpha=0.2, draw_lines=True):
        if image is None:
            image = cv2.imread(image_path)

        if masks is None:
            masks = LaneMask.from_file(masks_path, image=image)

        plot_image = LaneMask.generate_plot(masks, image=image, mask_alpha=mask_alpha, draw_lines_bool=draw_lines)
        plot_image = cv2.cvtColor(plot_image, cv2.COLOR_BGR2RGB)
        
        show_images([plot_image])





def get_lines(mask_batches, subtitutions: list = None):
        # if self.use_curve_line:
        #     batch_lines = get_lines_contours(mask_batches)
        # else:
        #     batch_lines = get_straight_lines(mask_batches)
        batch_lines = get_lines_contours(mask_batches)
        return batch_lines


def get_label_names(config_path: str) -> list:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
        label_names = config["names"]
    return label_names



def draw_segmentation_(image, masks, alpha=0.4, palette=default_palette):
    if not masks:
        return []

    mask_image = np.zeros(image.shape[:-1], dtype=np.uint8)
    for mask in masks:
        if mask.points.shape[0] == 0:
            break
        color = palette[int(mask.label) % len(palette)]
        color = (color[2], color[1], color[0])
        cv2.drawContours(mask_image, [np.expand_dims(mask.points, 1).astype(int)], contourIdx=-1, color=(255), thickness=-1)
        
        indices = mask_image != 0 
        image[indices] = image[indices] * (1 - alpha) + np.array(color) * alpha
        mask_image[:] = 0

    return image


def draw_segmentation(images, mask_batches, alpha=0.2, palette=default_palette):
    for (image, masks) in zip(images, mask_batches):
        draw_segmentation_(image, masks, alpha, palette)
    
    if len(images) == 1:
        return images[0]
    
    return images


def draw_lines(images, batch_curves, palette=default_palette, thickness=4):
    for (image, mask_curves) in zip(images, batch_curves):
        for idx, lane_line in enumerate(mask_curves):
            color = palette[lane_line.label % len(palette)]
            color = (color[2], color[1], color[0])
            for id in range(1, len(lane_line.points)):
                cv2.line(image, lane_line.points[id - 1], lane_line.points[id], color, thickness=thickness)


def draw_labels(
        images, 
        mask_batches,
        label_names, 
        margin=5, 
        padding=5,
        font_scale=0.8,
        thickness=2, 
        alpha=0.7,
        alpha_font=0.4):
    widget_position = np.array([margin, margin])

    unactive_color = tuple((np.array([255, 255, 255], dtype=np.uint8) * alpha_font).tolist())
    for image, masks in zip(images, mask_batches):
        #alpha_mask = np.zeros(image.shape[:2] + (1,))
        name_data = []
        mask_image = np.zeros_like(image)
        for idx, name in enumerate(label_names):
            text = f"{idx}: {name}"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness=thickness)[0]
            element_size = text_size + np.array([padding, padding]) * 2

            mask_position = widget_position + np.array([0, (element_size[1] + margin) * idx])
            text_position = mask_position + np.array([padding, padding])
            
            name_data.append({"text": text, "text_size": text_size, "element_size": element_size, "mask_position": mask_position, "text_position": text_position})
            cv2.rectangle(mask_image, mask_position, mask_position + element_size, (1, 1, 1), -1)
        
        indices = mask_image != np.array([0, 0, 0], dtype=np.uint8)
        image[indices] = mask_image[indices] * alpha + image[indices] * (1 - alpha)

        labels = [mask.label for mask in masks]

        for idx, data in enumerate(name_data):
            color = unactive_color
            if idx in labels:
                color = default_palette[idx]
            
            color = (color[2], color[1], color[0])
            cv2.putText(image, data['text'], data['text_position'] + np.array([0, data['text_size'][1]]), cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, color=color, thickness=thickness)
    return images

def show_images(images, figsize=(15, 5), count_images_for_ineration=2, columns=2):
    columns = min(len(images), columns)
    count_images_for_ineration = min(len(images), count_images_for_ineration)
    for slice_id in range(len(images) // count_images_for_ineration):
        rows = math.ceil(count_images_for_ineration / float(columns))
        fig, axes = plt.subplots(nrows=rows, ncols=columns, figsize=figsize)

        slice_min = slice_id * count_images_for_ineration
        slice_max = (slice_id + 1) * count_images_for_ineration

        if rows * columns > 1:
            axes = axes.flatten()
        else:
            axes = [axes]
        for (idx, (ax, image)) in enumerate(zip(axes, images[slice_min:slice_max])):
            if idx >= len(axes):
                break

            ax.imshow(image)
            ax.axis('off')

        plt.tight_layout()
        plt.show()


def get_mean_elapsed_time(batch_lines: list):
    mean_elapsed_time = 0
    count = 0
    for masks in batch_lines:
        for line in masks:
            mean_elapsed_time += line.elapsed_time
            count += 1
    
    if count == 0:
        return 0

    return mean_elapsed_time / count

def paint_str(string: str, color):
    start = "\033[38;2;{};{};{}m"
    end = "\033[0m"
    text = start + string + end
    return text.format(color[0], color[1], color[2])
    # print(text.format(color[0], color[1], color[2]))


def view_prediction_video(model, src, label_names=[], resized_width=-1, save_predictions=False, verbose=0, 
                          start_frame_callback=None, end_frame_callback=None, max_frames=-1):
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print("Не удалось открыть файл.")
        cap.release()
        return

    mean_hertz = 0
    mean_elapsed_time = 0
    i = 0

    count_tests = 500
    mean_elapsed_time = 0

    max_count_points = 5000
    measurements = np.zeros((max_count_points,), dtype=np.float32)
    hits = np.zeros((max_count_points,), dtype=np.int32)
    
    ret_images = []
    ret_predictions = []

    frame = 0
    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            break

        if max_frames > 0 and frame >= max_frames:
            break

        if start_frame_callback is not None:
            start_frame_callback()

        if resized_width > 0:
            resized_height = image.shape[0] / image.shape[1] * resized_width
            image = cv2.resize(image, (int(resized_width), int(resized_height)))
        
        if verbose != 0:
            print(f"id: {i}")

        # Обработка изображения
        start = timer()
        batch_lines, mask_batches, predictions = model.predict([image])
        end = timer()

        if save_predictions:
            ret_images.append(image.copy())
            # cv2.imwrite(os.path.join("test", "sdfsdfsdf" + ".jpg"), ret_images[0])
            ret_predictions.append(predictions[0])

        elapsed_time = end - start
        hertz = 1.0 / elapsed_time
        
        mean_hertz += hertz 
        mean_elapsed_time += elapsed_time * 1000.0 # ms

        if verbose != 0:
            print(f"frame: {i + 1}     elapsed time: {round(elapsed_time * 1000.0, 2)} ms")
            print(f"hertz: {round(hertz, 2)}     mean hertz: {round(mean_hertz / float(i + 1), 2)}    mean elapsed time: {round(mean_elapsed_time / float(i + 1), 2)} ms")
            print()

        for masks in batch_lines:
            for line in masks:
                measurements[line.mask_count_points] += line.elapsed_time
                hits[line.mask_count_points] += 1

        mean_elapsed_time += get_mean_elapsed_time(batch_lines)


        if i > 0 and i % count_tests == 0:
            if verbose != 0:
                print(f"Test {i // count_tests}. Elapsed time = {mean_elapsed_time / count_tests * 1000}")

            # measurements /= hits
            # x = np.linspace(0, max_count_points, max_count_points)
            # plt.plot(x, measurements)
            # plt.title("Сложность алгоритма")
            # plt.xlabel("Количество точек контура")
            # plt.ylabel("Время")
            # plt.show()
            # measurements *= hits

            mean_elapsed_time = 0

        draw_segmentation([image], mask_batches)
        draw_lines([image], batch_lines)
        draw_labels([image], mask_batches, label_names)

        cv2.imshow('prediction video', image)

        if end_frame_callback is not None:
            end_frame_callback()

        key_code = cv2.waitKey(5) & 0xFF
        if key_code == ord('q'):
            break
        
        i += 1
        frame += 1
    
    cap.release()
    return ret_images, ret_predictions


def get_straight_lines(mask_batches):
    batch_lines = []
    for masks in mask_batches:
        if not masks:
            batch_lines.append([])
            continue

        mask_image = np.zeros(masks[0].orig_shape + (1,), dtype=np.uint8)
        
        mask_lines = []
        for mask in masks:
            if mask.points.shape[0] == 0:
                break

            t1 = time.time()
            cv2.drawContours(mask_image, [np.expand_dims(mask.points, 1).astype(np.int32)], contourIdx=-1, color=(255), thickness=-1)
            lines = cv2.HoughLinesP(mask_image, 1, np.pi / 180, threshold=100, minLineLength=25, maxLineGap=30)
        
            if lines is not None:
                best_line = None
                max_lenght = 0

                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    lenght = np.linalg.norm([x2-x1, y2-y1])
                    if best_line is None or lenght > max_lenght:
                        max_lenght = lenght
                        best_line = line

                t2 = time.time()
                mask_lines.append(LaneLine(np.reshape(np.array(list(best_line), dtype=np.int32), (2, 2)), int(mask.label), t2-t1, int(mask.points.shape[0])))
            
            mask_image[:] = 0
        batch_lines.append(mask_lines)
    return batch_lines

    
def correct_point(xy, const_point_id, moving_point_id, max_distance, dist_accum_factor, n_accum, direction=[1, -1], decrease_distance=True):
    temp_point2_id = moving_point_id
    best_dis = np.linalg.norm(xy[const_point_id] - xy[temp_point2_id])
    best_point_id = temp_point2_id
    
    for dir in direction:
        i = 0
        prev_dist = best_dis
        accum_dist_change = 0
        while True:
            temp_point2_id = (temp_point2_id + dir) % xy.shape[0]

            if temp_point2_id == const_point_id:
                return -1
            
            distance = np.linalg.norm(xy[const_point_id] - xy[temp_point2_id])
            dist_change = distance - prev_dist
            prev_dist = distance
            accum_dist_change = accum_dist_change * dist_accum_factor + dist_change * (1 - dist_accum_factor)

            if decrease_distance:
                if distance < best_dis:
                    best_dis = distance
                    best_point_id = temp_point2_id
                
                if (i >= n_accum and accum_dist_change > 0):
                    break
            else:
                if distance > best_dis:
                    best_dis = distance
                    best_point_id = temp_point2_id
                
                if (i >= n_accum and accum_dist_change < 0):
                    break
            
            i += 1
    
    return best_point_id


def get_line_contour(
        masks, 
        max_distance=100, 
        min_id_dis_ratio=0.5, 
        edge_point_dis=20, 
        dist_accum_factor=0.8, 
        n_accum=5, 
        tolerance=0.0001) -> list:
    if not masks:
        return []
    
    mask_lines = []
    for mask in masks:
        t1 = time.time()
        if mask.points_n.shape[0] > 4:
            simplified_polygon = Polygon(mask.points_n).simplify(tolerance, preserve_topology=True)
            points = np.array(simplified_polygon.exterior.coords)
            points *= np.array([mask.orig_shape[1], mask.orig_shape[0]])
        else:
            points = mask.points
        
        if points.shape[0] == 0:
            break
        min_id_dis = int(points.shape[0] * min_id_dis_ratio)

        start_point1_id = np.random.randint(0, points.shape[0])
        start_point2_id = start_point1_id

        # best_distance = 0
        # best_point_id = -1
        # monotone = False
        # count_pass = 0
        # while True:

        #     if count_pass > 6:
        #         break

        #     start_point2_id = correct_point(points, start_point1_id, start_point2_id, max_distance, dist_accum_factor, n_accum, [1], monotone)
        #     monotone = not monotone

        #     if start_point2_id == -1:
        #         if best_distance > max_distance:
        #             start_point1_id = np.random.randint(0, points.shape[0])
        #             start_point2_id = start_point1_id

        #             best_distance = 0
        #             best_point_id = -1
        #             monotone = False
        #             count_pass += 1
        #             continue

        #     if start_point2_id == -1:
        #         break

        #     distance = np.linalg.norm(points[start_point2_id] - points[start_point1_id])
        #     if best_point_id < 0 or (distance < best_distance):
        #         best_distance = distance
        #         best_point_id = start_point2_id
        
        # if count_pass > 5:
        #     mask_lines.append([int(cls), []])
        #     break

        # start_point2_id = best_point_id

        




        diff_start_point_1 = (points[(start_point1_id + 1) % points.shape[0]] - points[start_point1_id])
        dir_start_point_1 = diff_start_point_1 / np.linalg.norm(diff_start_point_1)

        best_dis = max_distance + 1
        best_start_point2_id = start_point2_id
        i = 1 + min_id_dis
        count_pass = 0
        while i < points.shape[0] + 1:
            start_point2_id = (start_point1_id - i) % points.shape[0]
            diff_start_point_2 = (points[(start_point2_id - 1) % points.shape[0]] - points[start_point2_id])
            dir_start_point_2 = diff_start_point_2 / np.linalg.norm(diff_start_point_2)

            distance = np.linalg.norm(points[start_point2_id] - points[start_point1_id])

            if distance <= max_distance:
                if distance < best_dis:
                    best_dis = distance
                    best_start_point2_id = start_point2_id
                    if best_dis <= edge_point_dis:
                        count_pass += 1
                        if count_pass >= min_id_dis:
                            break
                        i = (1 + min_id_dis - count_pass)
                        best_dis = max_distance + 1
                        start_point1_id = np.random.randint(0, points.shape[0])
                        start_point2_id = start_point1_id
                else:
                    start_point2_id = best_start_point2_id
                    break
            
            i += 1
        if count_pass >= min_id_dis:
            mask_lines.append(LaneLine(np.array([], dtype=np.int32), int(mask.label)))
            break
            
        start_point2_id = correct_point(points,start_point1_id, start_point2_id, max_distance, dist_accum_factor, n_accum)

        moving_point1_id = start_point1_id
        moving_point2_id = start_point2_id


        line = [(points[moving_point1_id] + points[moving_point2_id]) / 2]
        back_line = []
        for dir in [1, -1]:
            moving_point1_id = start_point1_id
            moving_point2_id = start_point2_id

            while True:
                moving_point1_id = (moving_point1_id + dir) % points.shape[0]
                moving_point2_id = correct_point(points, moving_point1_id, moving_point2_id, max_distance, dist_accum_factor, n_accum, [-dir])

                if moving_point2_id == -1:
                    break
                else:
                    if dir == 1:
                        line = line + [(points[moving_point1_id] + points[moving_point2_id]) / 2]
                    else:
                        line = [(points[moving_point1_id] + points[moving_point2_id]) / 2] + line
        
        t2 = time.time()
        mask_lines.append(LaneLine(np.array(line, dtype=np.int32), mask.label, t2-t1, len(line)))

    return mask_lines
        

def get_lines_contours(mask_batches, max_distance=100, min_id_dis_ratio=0.5, edge_point_dis=20, dist_accum_factor=0.99, n_accum=10, subtitutions: list = None):
    lines = []
    for masks in mask_batches:
        lines.append(get_line_contour(masks, max_distance, min_id_dis_ratio, edge_point_dis, dist_accum_factor, n_accum))
    return lines


def str_to_seconds(string: str):
    splits = string.split(':')
    
    seconds = 0
    minutes = 0
    hours = 0
    if len(splits) == 1:
        seconds = int(splits[0])
    elif len(splits) == 2:
        minutes = int(splits[0])
        seconds = int(splits[1])
    elif len(splits) == 3:
        hours = int(splits[0])
        minutes = int(splits[1])
        seconds = int(splits[2])
    
    output = seconds + minutes * 60 + hours * 60 * 60
    return output


def move_files(source_path, target_path, max_items=-1, verbose=0):
    if os.path.isdir(target_path):
        idx = 0
        for image_name in os.listdir(source_path):
            full_source_path = os.path.join(source_path, image_name)
            if os.path.isfile(full_source_path):
                if max_items >= 0:
                    if idx >= max_items:
                        break
                shutil.move(full_source_path, target_path)
                if verbose == 1:
                    print(f"moved: {idx}/{max_items}    path: {full_source_path}")
                idx += 1