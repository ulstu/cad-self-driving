# cредства симуляции на основе Webots и ROS 
ros-humble-nav2-common
ros-humble-webots-ros2

pip install opencv-python
pip install prometheus-client

Image segmentation: fastseg
Camera and rangeFinder resolution: 840x480

Bird's-Eye-View Panoptic Segmentation Using Monocular Frontal View Images
https://arxiv.org/abs/2108.03227

Cam2BEV
https://github.com/ika-rwth-aachen/Cam2BEV

RANSAC
Martin A. Fischler and Robert C. Bolles (June 1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography". Comm. of the ACM 24: 381–395.



# Запуск из-под виртуальной машины
Инструкция для humble: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html#install-webots-ros2
Устанавливать webots-ros2 только из исходников
В файлах изменить код:
1. webots_ros2_driver/scripts/webots_tcp_client.py перед строкой 28 дописать:
```
    if 'WEBOTS_IP' in os.environ:
        return os.environ['WEBOTS_IP']
```
2. webots_ros2_driver/webots_ros2_driver/utils.py перед строкой 128
```
    if 'WEBOTS_IP' in os.environ:
        return os.environ['WEBOTS_IP']
```
3. В пользовательском файле ~/.bashrc добавить переменную среды со значением IP хостового компьютера:
```
export WEBOTS_IP=10.211.55.2
```


