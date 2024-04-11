# Установка решения в docker

## Установка в Linux

1. Установите `git`, `make`, `curl`, и `docker`
```sh
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
```
   Docker желательно поставить в режиме, чтобы команды была доступна для запуска [без sudo](https://docs.docker.com/engine/install/linux-postinstall/).

   Перезагрузите компьютер.
   
   Установите [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) для обработки данных в GPU.

2. Запустите сценарий подготовки, чтобы получить образ и запустить контейнер:

```sh
cd ./simulation/docker
make build
```
   
Дождитесь завершения работы скрипта (до 1 часа)

3. Настройка и запуск
Откройте файл start.sh и определите в нем значения переменных:
* название контейнера FLAVOR
* порт для vscode VS_PORT (10002 по умолчанию)
* порт для визуализации сцены webots WEBOTS_STREAM_PORT (10003 по умолчанию)
* порт для дашборда робота ROBOT_PANEL_PORT (10001 по умолчанию)).
Запустите скрипт start.sh
```sh
./start.sh
```

4. Доступ к терминалу запущенного контейнера возможен выполнением команды
   ```sh
   docker exec -it ulstu-${FLAVOR} bash
   ```
   Вместо ${FlAVOR} пропишите название контейнера, которое было прописано в скрипте start.sh

5. Доступ к VSCode: http://localhost:10002, вместо VS_PORT пропишите адрес порта из start.sh (10002 по умолчанию)
Для запуска решения откройте в vscode терминал, зайдите в папку ~/ros2_ws, постройте решение командой colcon build  запустите свое решение webots_ros2:
```sh
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch webots_ros2_suv robot_launch.py
```

6. Доступ к визуализации сцены после запуска: http://localhost:10003, WEBOTS_STREAM_PORT по умолчанию 10003

7. Доступ к панели управления роботом: http://localhost:10001, ROBOT_PANEL_PORT по умолчанию 10001

8. Если необходимо в контейнер прокинуть другой пакет ros2, то необходимо внести соотвесттвующие изменения в файлы Dockerfile.base (создание директорий) и Makefile (подключение дисков в образ и создание копий файлов).

9. В настоящее время исходники копируются для доступа в docker контейнер, т.к. можно запускать несколько контейнеров из одного образа.
Для отключения этой опции закомментируйте вызов copy-working-folders в run внутри Makefile.


## Установка через docker в MacOS m1 не поддерживается, т.к. отсутствует linux дистрибутив webots arm

