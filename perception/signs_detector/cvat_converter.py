import xml.etree.ElementTree as ET
import numpy as np
import os
import cv2
import uuid

class SegmentedImage:
    def __init__(self, image: np.ndarray = [], boxes: np.ndarray = [], labels: list = [], image_name = ""):
        self.image = image
        self.boxes = boxes
        self.labels = labels
        self.image_name = image_name


def get_segmented_images(xml_path: str, images_path) -> list:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    label_names = [label_element.text for label_element in root.findall('.//name')]

    inverse_label_names = {}
    for idx, label_name in enumerate(label_names):
        inverse_label_names[label_name] = idx

    image_elements = root.findall('.//image')

    segmented_images = []
    for image_element in image_elements:
        image_id = int(image_element.attrib['id'])
        image_name = image_element.attrib['name']
        width = image_element.attrib['width']
        height = image_element.attrib['height']
        
        box_elements = image_element.findall('.//box')
        
        boxes = []
        labels = []
        for box_element in box_elements:
            label_id = int(inverse_label_names[box_element.attrib['label']])
            x1 = float(box_element.attrib['xtl'])
            y1 = float(box_element.attrib['ytl'])
            
            x2 = float(box_element.attrib['xbr'])
            y2 = float(box_element.attrib['ybr'])
            
            boxes.append(np.array([x1, y1, x2, y2], dtype=np.int32))
            labels.append(label_id)
        boxes = np.stack(boxes)
        
        image_path = os.path.join(images_path, image_name)
        image = cv2.imread(image_path)        
        segmented_image = SegmentedImage(image, boxes, labels, image_name.split('.')[0])
        segmented_images.append(segmented_image)
    
    return segmented_images, label_names


def save_image_segments(image_segments: list = [], save_path: str = "perception/signs_detector/signs_data_classify", label_names: list = []):
    label_paths = [os.path.join(save_path, label) for label in label_names]
    
    for path in label_paths:
        os.makedirs(path, exist_ok=True)
    
    for image_segment in image_segments:
        for box, label in zip(image_segment.boxes, image_segment.labels):
            x1, y1, x2, y2 = box
            segment = image_segment.image[y1:y2, x1:x2]
            
            count_name_symbols = max(-7, -len(image_segment.image_name))
            count_label_symbols = max(-3, -len(label_names[label]))
            segment_name = image_segment.image_name[count_name_symbols:] + "-" + label_names[label][-count_label_symbols:] + "-" + f"{x1}-{y1}-{x2}-{y2}" + ".jpg"
            segment_path = os.path.join(label_paths[label], segment_name)
            cv2.imwrite(segment_path, segment)


def save_cvat_to_fragments(xml_path: str, images_path: str, save_path: str):
    segmented_images, xml_label_names = get_segmented_images(xml_path, images_path)
    save_image_segments(segmented_images, save_path, xml_label_names)