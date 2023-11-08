import cv2
import torch
from datetime import datetime
from map_builder import MapBuilder
from fastseg import MobileV3Large
from fastseg.image import colorize, blend


cap = cv2.VideoCapture("https://restreamer.vms.evo73.ru/909026beec0c23c6/stream.m3u8")
map_builder = MapBuilder(model_path=f'/Users/hiber/Downloads/yolov8l.pt',
                                            ipm_config=f'/Users/hiber/Downloads/config.yaml')
weights_path = "/Users/hiber/Downloads/mobilev3large-lraspp.pt"
if torch.cuda.is_available():
    seg_model = MobileV3Large.from_pretrained(weights_path).cuda().eval()
else:
    seg_model = MobileV3Large.from_pretrained(weights_path).eval()

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)


    # labels = seg_model.predict_one(frame)
    # colorized = colorize(labels)
    #composited = blend(frame, colorized)

    # image = map_builder.resize_img(frame)
    # results = map_builder.detect_objects(image)
    ipm_image = map_builder.generate_ipm(frame, is_mono=False, need_cut=False)

    cv2.imshow("IPM", ipm_image)
    # cv2.imshow("objects", results[0].plot())



    # img_filename = datetime.now().strftime("%Y%m%d-%H%M%S")
    # base_path = "/Users/hiber/Downloads/"
    # cv2.imwrite(f"{base_path}{img_filename}.png", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break