import cv2
import torch
from datetime import datetime
from map_builder import MapBuilder
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
import time
import numpy as np
from orientation import quaternion_from_euler, euler_from_quaternion, local_to_global, draw_absolute_tracks
from behavioral_analysis import BehaviourAnalyser


cap = cv2.VideoCapture("https://restreamer.vms.evo73.ru/909026beec0c23c6/stream.m3u8")
map_builder = MapBuilder(model_path=f'/home/hiber/ros2_ws/src/webots_ros2_suv/resource/yolov8l.pt',
                                            ipm_config=f'/home/hiber/ros2_ws/src/webots_ros2_suv/config/webcam_config.yaml')
weights_path = "/home/hiber/ros2_ws/src/webots_ros2_suv/resource/mobilev3large-lraspp.pt"
if torch.cuda.is_available():
    seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
else:
    seg_model = MobileV3Large.from_pretrained(weights_path).eval()

last_time = time.perf_counter() 

while(True):
    ret, frame = cap.read()

    end_time = time.perf_counter()
    total_time = end_time - last_time
    # if total_time < 0.5:
    #     continue
    last_time = end_time
    print(f'elapsed time: {total_time:.4f}')
   

    image = map_builder.resize_img(frame)
    results = map_builder.detect_objects(image)
    cboxes = results[0].boxes.data.cpu()
    tbs, widths = map_builder.transform_boxes(cboxes)


    labels = np.asarray(seg_model.predict_one(frame))
    labels = map_builder.resize_img(labels.astype('float32'))
    colorized = np.array(colorize(labels))

    image_seg = map_builder.remove_detected_objects(labels, cboxes)
    ipm_image_seg = map_builder.generate_ipm(labels, is_mono=True, need_cut=False)
    ipm_image_seg = map_builder.put_objects(ipm_image_seg, tbs, widths, results)
    ipm_image_seg_colorized = np.array(colorize(ipm_image_seg))
    ipm_image = map_builder.generate_ipm(image, is_mono=False, need_cut=True)

    pov_point = (image.shape[0], int(image.shape[1] / 2))
    pov_point = map_builder.calc_bev_point(pov_point)
    ipm_image_seg_colorized = ipm_image_seg_colorized[:pov_point[1], :]
    colorized = colorized[:pov_point[1], :]


    ipm_image_seg_colorized, track_ids = map_builder.track_objects(results, ipm_image_seg_colorized, (0, 0, 0))

    ipm_image_seg_colorized = cv2.resize(ipm_image_seg_colorized, (500, 500), cv2.INTER_AREA)
    ipm_image = cv2.resize(ipm_image, (500, 500), cv2.INTER_AREA)
    frame = cv2.resize(frame, (500, 500), cv2.INTER_AREA)

    cv2.imshow('frame', frame)
    cv2.imshow("IPM seg", ipm_image_seg_colorized)
    cv2.imshow("IPM original", ipm_image)
    cv2.imshow("segmented", np.array(colorized))


    # img_filename = datetime.now().strftime("%Y%m%d-%H%M%S")
    # base_path = "/home/hiber/Downloads/"
    # cv2.imwrite(f"{base_path}{img_filename}.png", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break