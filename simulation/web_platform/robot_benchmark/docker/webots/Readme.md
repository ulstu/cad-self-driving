Установка решения в docker
Установка в Linux

    Установите git, make, curl, и docker

sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER

Docker желательно поставить в режиме, чтобы команды была доступна для запуска без sudo.

Перезагрузите компьютер.

Установите nvidia-container-toolkit для обработки данных в GPU.

    Запустите сценарий подготовки, чтобы получить образ и запустить контейнер:

cd ./docker
make build

Дождитесь завершения работы скрипта (до 1 часа)

    Настройка и запуск Откройте файл start.sh и определите в нем значения переменных:

    название контейнера FLAVOR
    порт для vscode VS_PORT (10002 по умолчанию)
    порт для визуализации сцены webots WEBOTS_STREAM_PORT (10003 по умолчанию)
    порт для дашборда робота ROBOT_PANEL_PORT (10001 по умолчанию)). Запустите скрипт start.sh


cd webots
./start.sh

Доступ к терминалу запущенного контейнера возможен выполнением команды

docker exec -it ulstu-${FLAVOR} bash

    Вместо ${FlAVOR} пропишите название контейнера, которое было прописано в скрипте start.sh

    Доступ к VSCode: http://localhost:10002, вместо VS_PORT пропишите адрес порта из start.sh (10002 по умолчанию) 
    Для запуска решения откройте в vscode терминал, зайдите в папку ~/ros2_ws, 
    постройте решение командой colcon build запустите свое решение webots_ros2:
cd src/webots_ros2_suv/map-server
npm i
npm run build

cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch webots_ros2_suv robot_launch.py

    Доступ к визуализации сцены после запуска: http://localhost:10003, WEBOTS_STREAM_PORT по умолчанию 10003

    Доступ к панели управления роботом: http://localhost:10001, ROBOT_PANEL_PORT по умолчанию 10001

    Если необходимо в контейнер прокинуть другой пакет ros2, то необходимо внести соотвесттвующие изменения в файлы 
    Dockerfile.base (создание директорий) и Makefile (подключение дисков в образ и создание копий файлов).

    В настоящее время исходники копируются для доступа в docker контейнер, т.к. можно запускать несколько контейнеров из одного образа. 
    Для отключения этой опции закомментируйте вызов copy-working-folders в run внутри Makefile.

Основные адреса:
- http://localhost:10001
- http://localhost:10002
- http://localhost:10003/index.html

Установка через docker в MacOS m1 не поддерживается, т.к. отсутствует linux дистрибутив webots arm