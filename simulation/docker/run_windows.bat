@echo off
setlocal

REM -- Set Environment Variables
set "IMAGE=ulstu"
set "FLAVOR=devel"
set "ROBOT_PANEL_PORT=8008"
set "VS_PORT=31415"
set "WEBOTS_STREAM_PORT=1234"
set "DOCKER_DIR=%~dp0"
set "PROJECT_DIR=%~dp0.."

REM -- Detect GPU capabilities
for /f "usebackq tokens=*" %%a in (`docker info ^| findstr Runtimes ^| findstr nvidia`) do (
    set "NVIDIA_GPU=--runtime nvidia --gpus all"
)
if not defined NVIDIA_GPU set "NVIDIA_GPU="

REM -- Define Colors (Simulated as Windows CMD does not support ANSI colors)
set "NC="
set "RED="
set "GREEN="
set "BOLD="

REM -- Main targets
if "%1"=="build" goto build
if "%1"=="run" goto run
if "%1"=="test-nvidia" goto test-nvidia
if "%1"=="copy-working-files" goto copy-working-files
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
docker run --ipc=host --cap-add SYS_ADMIN --name %IMAGE%-%FLAVOR% --privileged --restart unless-stopped -p %ROBOT_PANEL_PORT%:8008 -p %VS_PORT%:31415 -p %WEBOTS_STREAM_PORT%:1234 -e NVIDIA_DRIVER_CAPABILITIES=all %NVIDIA_GPU% -v %USERPROFILE%/.Xauthority:/ulstu/.host/.Xauthority:ro -v /tmp/.X11-unix/:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri:ro -v /dev:/dev:rw -v %PROJECT_DIR%/docker/projects/%FLAVOR%:/ulstu/repositories:rw -d -it %IMAGE%
goto end

:test-nvidia
if exist "%ProgramFiles%/NVIDIA Corporation/NVSMI/nvidia-smi.exe" (
    echo Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!
) else (
    echo NVIDIA GPU not detected.
)
goto end

:copy-working-files
docker exec -it %IMAGE%-%FLAVOR% ln -s /ulstu/repositories/webots_ros2_suv /ulstu/ros2_ws/src/webots_ros2_suv
docker exec -it %IMAGE%-%FLAVOR% ln -s /ulstu/repositories/robot_interfaces /ulstu/ros2_ws/src/robot_interfaces
goto end

:copy-working-folders
if exist "%PROJECT_DIR%/docker/projects/%FLAVOR%" (
    echo project dir exists. Remove it first
) else (
    mkdir "%PROJECT_DIR%/docker/projects/%FLAVOR%"
    xcopy "%PROJECT_DIR%/webots_ros2_suv" "%PROJECT_DIR%/docker/projects/%FLAVOR%/webots_ros2_suv" /E /H /K
    xcopy "%PROJECT_DIR%/robot_interfaces" "%PROJECT_DIR%/docker/projects/%FLAVOR%/robot_interfaces" /E /H /K
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
