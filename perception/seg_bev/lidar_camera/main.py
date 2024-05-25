import cv2

from LidarTool import LidarTool

if __name__ == '__main__':
    lt = LidarTool()
    lt.set_npy("data/20240524-213140.npy")
    cv2.imshow("title", lt.get_XYPlane_colored_by_height())
