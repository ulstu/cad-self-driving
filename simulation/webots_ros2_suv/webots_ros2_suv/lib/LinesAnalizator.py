import numpy as np
import cv2
from collections import defaultdict

class LinesAnalizator():
    def __init__(self, accamulators_length=10):
        self.count_roads_accamulator = []
        self.line_indices_accamulator = []
        self.last_count_roads_answer = 1
        self.last_line_id_answer = 1
        self.accamulators_length = accamulators_length


    def get_line_points(self, image, y, batch_lines, min_distance=20, distance_to_error=20):
        points = []
        for line in batch_lines[0]:
            if line.label == 14:
                pass
            for point in line.points:
                if abs(point[1] - y) < min_distance:
                    # cv2.circle(image, (int(point[0]), int(y)), 3, (0, 0, 255), -1)
                    points.append([np.array([point[0], y]), line.label])
                    break
        points = sorted(points, key=lambda x: -x[0][0])
        
        idx = 0
        while idx < len(points):
            if idx + 1 < len(points):
                if points[idx + 1][1] == points[idx][1] and abs(points[idx + 1][0][0] - points[idx][0][0]) <= distance_to_error:
                    points.pop(idx + 1)
                    continue
            idx += 1
            

        return points
    

    def find_curbsides(self, line_points):
        right_curbside_id = 0
        right_curbside_find_is = False

        left_curbside_id = len(line_points) - 1
        left_curbside_find_is = False

        for idx in range(len(line_points)):
            if not left_curbside_find_is and line_points[idx][1] == 13: # left_curbside
                left_curbside_id = idx
                left_curbside_find_is = True
            if not right_curbside_find_is and line_points[idx][1] == 14: # right_curbside
                right_curbside_id = idx
                right_curbside_find_is = True
        
        return [[left_curbside_id, left_curbside_find_is], [right_curbside_id, right_curbside_find_is]]


    def num_road_to_line_id(self, line_points, from_line_id, to_line_id, border_conditions=True):
        count_roads = 0
        for idx in range(from_line_id, to_line_id + 1):
            # 14: right_curbside
            if line_points[idx][1] == 14:
                 continue

            
            # Если сразу за right_curbside идёт white-solid - значит это одна линия
            # 2: white-solid
            # 14: right_curbside
            if idx - 1 >= 0:
                if line_points[idx][1] == 2 and line_points[idx - 1][1] == 14:
                    continue
                
            if border_conditions:
                # 2: white-solid
                # 13: left_curbside
                if idx + 1 < len(line_points):
                    if line_points[idx][1] == 2 and line_points[idx + 1][1] == 13:
                        continue
            
            count_roads += 1

        return count_roads

    
    # def analize_line_points(self, line_points, left_curbside, right_curbside):
    #     count_roads = 0
    #     for idx in range(left_curbside[0], right_curbside[1] + 1):
    #         if line_points[idx][1] == 13:
    #              continue

    #         # Если сразу за curbside идёт white-solid - значит это одна линия
    #         # 2: white-solid
    #         # 13: left_curbside
    #         if idx - 1 >= 0:
    #             if line_points[idx][1] == 2 and line_points[idx - 1][1] == 13:
    #                 continue
            
    #         # 2: white-solid
    #         # 14: right_curbside
    #         if idx + 1 < len(line_points):
    #             if line_points[idx][1] == 2 and line_points[idx + 1][1] == 14:
    #                 continue
            
    #         count_roads += 1

    #     return count_roads


    def get_current_car_line(self, line_points, left_curbside, right_curbside, image_size_x, car_point_part=0.5):
        car_point_x = car_point_part * image_size_x
        car_line_id = -1
        if right_curbside[1]:
            for idx in range(right_curbside[0], left_curbside[0] + 1):
                if line_points[idx][0][0] < car_point_x:
                    car_line_id = self.num_road_to_line_id(line_points, right_curbside[0], idx, border_conditions=False)
                    break
        return car_line_id
                


    def get_count_roads(self, image, batch_lines, min_y, max_y, image_shape, gap=20, min_distance=20, car_point_part=0.5):
        height = (max_y - min_y) * image_shape[0]
        count_lines = int(height / gap)

        max_y_absolute = max_y * image_shape[0]
        count_roads = []
        car_line_indices = []
        for line_id in range(count_lines):
            line_y = max_y_absolute - line_id * gap

            # cv2.line(image, (0, int(line_y)), (int(image.shape[1]), int(line_y)), (0, 255, 0), 2)

            line_points = self.get_line_points(image, line_y, batch_lines, min_distance)

            left_curbside, right_curbside = self.find_curbsides(line_points)

            # line_count_roads = self.analize_line_points(line_points)
            line_count_roads = self.num_road_to_line_id(line_points, right_curbside[0], left_curbside[0])
            car_line_id = self.get_current_car_line(line_points, left_curbside, right_curbside, image_shape[1], car_point_part)

            if left_curbside[1] or right_curbside[1]:
                count_roads.append(line_count_roads)
            
            if right_curbside[1] and car_line_id != 0 and car_line_id != -1:
                car_line_indices.append(car_line_id)
        
        best_count_roads = max(count_roads, key=count_roads.count) if len(count_roads) > 0 else -1
        best_car_line_id = max(car_line_indices, key=car_line_indices.count) if len(car_line_indices) > 0 else -1

        return best_count_roads, best_car_line_id
    

    def most_stable_number(self, sequence):
        subsequences = defaultdict(list)
        
        current_value = sequence[0]
        current_length = 1
        
        for i in range(1, len(sequence)):
            if sequence[i] == current_value:
                current_length += 1
            else:
                subsequences[current_value].append(current_length)
                current_value = sequence[i]
                current_length = 1
        
        subsequences[current_value].append(current_length)
        
        max_avg_length = 0
        most_stable_num = None
        
        for num, lengths in subsequences.items():
            avg_length = sum(lengths) / len(lengths)
            if avg_length > max_avg_length:
                max_avg_length = avg_length
                most_stable_num = num
        
        return most_stable_num

        
    def analize_roads_with_accamulator(self, image, batch_lines, min_y, max_y, image_shape, gap=20, min_distance=20, car_point_part=0.5, reset_accamulator=False):
        count_roads, car_line_id = self.get_count_roads(image, batch_lines, min_y, max_y, image_shape, gap, min_distance, car_point_part)
        
        if reset_accamulator:
            self.count_roads_accamulator.clear()
            self.line_indices_accamulator.clear()
        
        self.count_roads_accamulator.append(count_roads)
        self.line_indices_accamulator.append(car_line_id)
        if len(self.count_roads_accamulator) > self.accamulators_length: self.count_roads_accamulator.pop(0)
        if len(self.line_indices_accamulator) > self.accamulators_length: self.line_indices_accamulator.pop(0)
        
        best_count_roads = self.most_stable_number(self.count_roads_accamulator)
        best_line_id = self.most_stable_number(self.line_indices_accamulator)

        if best_count_roads == -1 or best_count_roads == 0:
            best_count_roads = self.last_count_roads_answer
        
        if best_line_id == -1 or best_line_id == 0:
            best_line_id = self.last_line_id_answer
        
        self.last_count_roads_answer = best_count_roads
        self.last_line_id_answer = best_line_id
        return best_count_roads, best_line_id
        

    def draw_labels(
        self,
        image, 
        label_names,
        colors,
        margin=5, 
        padding=5,
        font_scale=0.4,
        thickness=2, 
        alpha=0.7,
        alpha_font=0.4):
        widget_position = np.array([margin, image.shape[0]])

        for idx, name in enumerate(label_names):
            text = name
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness=thickness)[0]
            element_size = text_size + np.array([padding, padding]) * 2
            widget_position[1] -= element_size[1] + margin

        unactive_color = tuple((np.array([255, 255, 255], dtype=np.uint8) * alpha_font).tolist())
        #alpha_mask = np.zeros(image.shape[:2] + (1,))
        name_data = []
        mask_image = np.zeros_like(image)
        for idx, name in enumerate(label_names):
            text = name
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness=thickness)[0]
            element_size = text_size + np.array([padding, padding]) * 2

            mask_position = widget_position + np.array([0, (element_size[1] + margin) * idx])
            text_position = mask_position + np.array([padding, padding])
            
            name_data.append({"text": text, "text_size": text_size, "element_size": element_size, "mask_position": mask_position, "text_position": text_position})
            cv2.rectangle(mask_image, mask_position, mask_position + element_size, (1, 1, 1), -1)
        
        indices = mask_image != np.array([0, 0, 0], dtype=np.uint8)
        image[indices] = mask_image[indices] * alpha + image[indices] * (1 - alpha)

        for idx, data in enumerate(name_data):
            color = colors[idx]
            color = (color[2], color[1], color[0])
            cv2.putText(image, data['text'], data['text_position'] + np.array([0, data['text_size'][1]]), cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, color=color, thickness=thickness)
        return image