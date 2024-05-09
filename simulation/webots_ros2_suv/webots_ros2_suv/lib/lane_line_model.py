import ultralytics
from ultralytics import YOLO
import matplotlib.pyplot as plt
import yaml
import numpy as np
import cv2
from webots_ros2_suv.lib.lane_line_model_utils import *
import os
from collections.abc import Iterable
import torch


class LaneLineModel:
    def __init__(self, path: str, use_curve_line=True):
        self.model = YOLO(path)
        if torch.cuda.is_available():
            self.model.to('cuda')
        self.use_curve_line = use_curve_line
    
    def get_lines(self, mask_batches, subtitutions: list = None):
        if self.use_curve_line:
            batch_lines = get_lines_contours(mask_batches)
        else:
            batch_lines = get_straight_lines(mask_batches)
        return batch_lines

    def train(self, dataset_path, epochs, output_directory="runs", train_path="images/train", val_path="images/valid"):
        with open('config.yaml', 'r') as file:
            config = yaml.safe_load(file)

            absolute_path = os.path.abspath(dataset_path)

            config['path'] = absolute_path
            config['train'] = train_path
            config['val'] = val_path

        if not os.path.exists("tmp"):
            os.makedirs("tmp")
        
        with open("tmp/tmp_config.yaml", 'w') as file:
            yaml.dump(config, file)
        
        results = self.model.train(data="tmp/tmp_config.yaml", epochs=epochs, project=output_directory)
        return results

    def predict_masks(self, images):
        results = self.model.predict(images, verbose=False)
        mask_batches = LaneMask.from_predictions(results)
        return mask_batches, results

    def predict(self, images):
        mask_batches, results = self.predict_masks(images)
        lines = self.get_lines(mask_batches)
        return lines, mask_batches, results
    
    def generate_prediction_plots_yolo(self, images):
        results = self.model.predict(images, verbose=False)
        plot_images = [result.plot() for result in results]
        return plot_images
    
    def generate_prediction_plots(self, images, labels=[], mask_batches=None, batch_lines=None, do_copy=True):
        if mask_batches is None:
            mask_batches, _ = self.predict_masks(images)
        if batch_lines is None:
            batch_lines = self.get_lines(mask_batches)

        if do_copy:
            images_to_draw = np.copy(np.array(images))
        else:
            images_to_draw = images
        draw_segmentation(images_to_draw, mask_batches)
        draw_lines(images_to_draw, batch_lines)

        if len(labels) > 0:
            draw_labels(images_to_draw, mask_batches, labels, font_scale=0.5)

        return images_to_draw
    

    def visualize_prediction(self, images, yolo_vis=False):
        plot_images = self.generate_prediction_plots_yolo(images) if yolo_vis else self.generate_prediction_plots(images)
        show_images(plot_images)
        return plot_images
        