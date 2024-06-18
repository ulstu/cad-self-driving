from perception.seg_bev.lidar_camera.LidarTool import LidarTool

if __name__ == '__main__':
    lt = LidarTool()
    lt.set_npy("data/20240524-213140.npy")
    lt.get_YZPlane_colored_by_height(scale_factor=20)
