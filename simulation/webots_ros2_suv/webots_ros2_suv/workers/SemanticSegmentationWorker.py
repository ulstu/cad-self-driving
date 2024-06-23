from AbstractWorker import AbstractWorker
import torch
import pathlib
import os
import numpy as np
import traceback
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
import cv2
from ultralytics import YOLO

PACKAGE_NAME = 'webots_ros2_suv'
local_seg_model_path = "resource/lane_line_model/LLD-level-5-v2.pt"


class SemanticSegmentationWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory(PACKAGE_NAME)
        weights_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'mobilev3large-lraspp.pt')))
        seg_model_path = os.path.join(package_dir, local_seg_model_path)
        self.model = YOLO(seg_model_path)


        if torch.cuda.is_available():
            self.model.to("cuda")
            self.seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
        else:
            self.seg_model = MobileV3Large.from_pretrained(weights_path).eval()
        super().log('Segmentation Node initialized')


    def on_event(self, event, scene=None):
        print("Emergency State")
        return None
    
    #@timeit
    def on_data(self, world_model):
        try:
            img = Image.fromarray(world_model.rgb_image)

            results = self.model.predict(np.array(img))

            world_model.seg_image = np.ones(world_model.rgb_image.shape[:2] + (1,), dtype=np.uint8)

            for xy in results[0].masks.xy:
                cv2.drawContours(world_model.seg_image, [np.expand_dims(xy, 1).astype(int)], contourIdx=-1, color=0, thickness=-1)



            world_model.seg_image = Image.fromarray(world_model.seg_image)

            #world_model.seg_image = self.seg_model.predict_one(img)
            world_model.seg_colorized = colorize(world_model.seg_image)
            world_model.seg_composited = blend(img, world_model.seg_colorized)

            world_model.seg_image = np.asarray(world_model.seg_image)
            world_model.seg_colorized = np.asarray(world_model.seg_colorized)
            world_model.seg_composited = np.asarray(world_model.seg_composited)
        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
        
        return world_model
