@echo off
setlocal

REM Переопределение переменных из аргументов командной строки
if not "%~2"=="" set "IMAGE=%~2"
if not "%~3"=="" set "FLAVOR=%~3"
if not "%~4"=="" set "ROBOT_PANEL_PORT=%~4"
if not "%~5"=="" set "VS_PORT=%~5"
if not "%~6"=="" set "WEBOTS_STREAM_PORT=%~6"

REM Значения по умолчанию, если аргументы не переданы
if not defined IMAGE set "IMAGE=ulstu"
if not defined FLAVOR set "FLAVOR=devel"
if not defined ROBOT_PANEL_PORT set "ROBOT_PANEL_PORT=8008"
if not defined VS_PORT set "VS_PORT=31415"
if not defined WEBOTS_STREAM_PORT set "WEBOTS_STREAM_PORT=1234"

REM 
set "DOCKER_DIR=%~dp0"
set "PROJECT_DIR=%~dp0"

REM 
for /f "usebackq tokens=*" %%a in (`docker info ^| findstr Runtimes ^| findstr nvidia`) do (
    set "NVIDIA_GPU=--runtime nvidia --gpus all"
)
if not defined NVIDIA_GPU set "NVIDIA_GPU="

REM 
set "NC="
set "RED="
set "GREEN="
set "BOLD="

REM 
if "%1"=="build" goto build
if "%1"=="run" goto run
if "%1"=="test-nvidia" goto test-nvidia
if "%1"=="setup-environment" goto setup-environment
if "%1"=="wsl-fix" goto wsl-fix
if "%1"=="copy-working-folders" goto copy-working-folders
if "%1"=="start-code-server" goto start-code-server
if "%1"=="stop-code-server" goto stop-code-server
if "%1"=="exec" goto exec
if "%1"=="destroy" goto destroy
if "%1"=="setup-default" goto setup-default
if "%1"=="setup-interactive" goto setup-interactive
goto end

:build
docker build %DOCKER_DIR% -f %DOCKER_DIR%Dockerfile.base -t %IMAGE%
goto end

:run
docker run --ipc=host ^
     --cap-add SYS_ADMIN ^
     --name %IMAGE%-%FLAVOR% ^
     --privileged ^
     --restart unless-stopped ^
      -p %ROBOT_PANEL_PORT%:8008 -p %VS_PORT%:31415 -p %WEBOTS_STREAM_PORT%:1234 ^
      -e NVIDIA_DRIVER_CAPABILITIES=all %NVIDIA_GPU% ^
      -e DISPLAY=host.docker.internal:0 ^
      -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY ^
      -e PULSE_SERVER=$PULSE_SERVER ^
      -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR ^
      -e LD_LIBRARY_PATH=/usr/lib/wsl/lib ^
      -v %USERPROFILE%/.Xauthority:/ulstu/.host/.Xauthority:ro ^
      -v /mnt/wslg:/mnt/wslg ^
      -v /usr/lib/wsl:/usr/lib/wsl ^
      -v /usr/lib/wsl:/usr/lib/wsl ^
      -v /mnt/wslg:/mnt/wslg ^
      -v /tmp/.X11-unix/:/tmp/.X11-unix:rw ^
      -v /dev/dri:/dev/dri:ro ^
      -v /dev:/dev:rw ^
      -v %PROJECT_DIR%/../../:/ulstu/repositories:rw ^
      --device /dev/dri/card0 --device /dev/dri/renderD128  ^
      --device=/dev/dxg ^
      -it --gpus all  ^
      -d -it %IMAGE%
goto end

:test-nvidia
if exist "%ProgramFiles%/NVIDIA Corporation/NVSMI/nvidia-smi.exe" (
    echo Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!
) else (
    echo NVIDIA GPU not detected.
)
goto end

:setup-environment
docker exec -it %IMAGE%-%FLAVOR% ln -s /ulstu/repositories/webots_ros2_suv /ulstu/ros2_ws/src/webots_ros2_suv
docker exec -it %IMAGE%-%FLAVOR% ln -s /ulstu/repositories/robot_interfaces /ulstu/ros2_ws/src/robot_interfaces
docker exec -it %IMAGE%-%FLAVOR% mkdir /ulstu/ros2_ws/data
docker exec -it %IMAGE%-%FLAVOR% mkdir /ulstu/ros2_ws/data/paths
goto end

:wsl-fix
docker cp %IMAGE%-%FLAVOR%:/ulstu/.bashrc C:\Windows\Temp\.bashrc
echo export LD_LIBRARY_PATH=/usr/lib/wsl/lib >> C:\Windows\Temp\.bashrc
echo export DISPLAY=host.docker.internal:0 >> C:\Windows\Temp\.bashrc
echo export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA >> C:\Windows\Temp\.bashrc
docker cp C:\Windows\Temp\.bashrc %IMAGE%-%FLAVOR%:/ulstu/.bashrc
docker exec -it %IMAGE%-%FLAVOR% sudo sed -i -e "s|return 'microsoft-standard' in uname().release|return False|" /opt/ros/humble/local/lib/python3.10/dist-packages/webots_ros2_driver/utils.py
docker exec -it %IMAGE%-%FLAVOR% sudo sed -i "s/\r$//g" /ulstu/.bashrc


goto end

:copy-working-folders
if exist "%PROJECT_DIR%/projects/%FLAVOR%" (
    echo project dir exists. Remove it first
) else (
    mkdir "%PROJECT_DIR%/projects/%FLAVOR%"
    xcopy "%PROJECT_DIR%/webots_ros2_suv" "%PROJECT_DIR%/projects/%FLAVOR%/webots_ros2_suv" /E /H /K
    xcopy "%PROJECT_DIR%/robot_interfaces" "%PROJECT_DIR%/projects/%FLAVOR%/robot_interfaces" /E /H /K
)
goto end

:start-code-server
docker exec -d -it %IMAGE%-%FLAVOR% bash -c "pgrep code-server || code-server /ulstu/ros2_ws"
start http://localhost:%VS_PORT%?folder=/ulstu/ros2_ws
goto end

:stop-code-server
docker exec -it %IMAGE%-%FLAVOR% pkill -f code-server
goto end

:exec
docker exec -it %IMAGE%-%FLAVOR% bash
goto end

:destroy
docker container kill %IMAGE%-%FLAVOR%
docker container rm -f %IMAGE%-%FLAVOR%
goto end

:setup-default
docker exec -it %IMAGE%-%FLAVOR% sh -c '/usr/bin/setup.sh --first-time-ros-setup'
goto end

:setup-interactive
docker exec -it %IMAGE%

:end
echo finished
