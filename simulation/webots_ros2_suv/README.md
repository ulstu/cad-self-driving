# cредства симуляции на основе Webots и ROS 
## ПО для запуска
* Linux Ubuntu 22.04
* ROS2 Humble
* Webots 2023b

## Установка пакетов Linux после установки ROS Humble по стандартной инструкции
```
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-webots-ros2
```
## Установка пакетов python
```
pip install opencv-python
pip install prometheus-client
pip install fastseg
pip install cherrypy
pip install pyyaml
pip install ultralytics
pip install shapely
pip install pyproj
```

## Установка решения
* Необходимо установить символические ссылки на папки simulation/pcl_maps, simulation/robot_interfaces и simulation/webots_ros2_suv в папку ~/ros2_ws/src
```
ln -s ~/repositories/cad-self-driving/pcl_maps ~/ros2_ws/pcl_maps
ln -s ~/repositories/cad-self-driving/robot_interfaces ~/ros2_ws/robot_interfaces
ln -s ~/repositories/cad-self-driving/webots_ros2_suv ~/ros2_ws/webots_ros2_suv
```

* В ~/.bashrc добавить переменные среды (внимательнее с именем пользователя!):
```
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
export WEBOTS_HOME=/usr/local/webots
export ROS2_WEBOTS_HOME=/usr/local/webots
```
* Компиляция командой 'colcon build' из папки ~/ros2_ws 
* Запуск командой 'ros2 launch webots_ros2_suv robot_launch.py'
* Если стоит conda/miniconda, необходимо ее деактивировать 'conda deactivate'

Image segmentation: fastseg

Camera and rangeFinder resolution: 840x480



# Запуск из-под виртуальной машины в MacOS
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


# Полезные статьи для обзора
Bird's-Eye-View Panoptic Segmentation Using Monocular Frontal View Images
https://arxiv.org/abs/2108.03227

Cam2BEV
https://github.com/ika-rwth-aachen/Cam2BEV

RANSAC
Martin A. Fischler and Robert C. Bolles (June 1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography". Comm. of the ACM 24: 381–395.

Visualization of Obstacles on Bird’s-eye View Using Depth Sensor for Remote Controlled Robot
http://www.robot.t.u-tokyo.ac.jp/~yamashita/paper/B/B172Final.pdf


# Неявные ошибки
1. Если при импорте torch возникает ошибка "name 'sympy' is not defined", то выполнить команду 
```bash
pip install mpmath --upgrade
```
