MAKEFLAGS+=--silent
UID:=$(shell id -u)
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
IMAGE=ulstu
ROS2_PROJECT="webots_ros2_suv"
FLAVOR=devel
ROBOT_PANEL_PORT=8008
VS_PORT=31415
WEBOTS_STREAM_PORT=1234

.PHONY: all

#

all: test-nvidia copy-working-folders  run copy-working-files start-code-server

colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

build:
	echo ${NO_CACHE_ARG}
	DOCKER_BUILDKIT=1 docker build ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.base -t ulstu ${DOCKER_ARGS} --build-arg UID=${UID}

#--net=host

run:
	docker run \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name ulstu-${FLAVOR} \
		--privileged \
		--restart unless-stopped \
		-p ${ROBOT_PANEL_PORT}:8008 \
		-p ${VS_PORT}:31415 \
		-p ${WEBOTS_STREAM_PORT}:1234 \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/ulstu/.host/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v /dev:/dev:rw \
		-v ${PROJECT_DIR}/projects/${FLAVOR}:/ulstu/repositories:rw \
		-d -it ${IMAGE} 1>/dev/null

test-nvidia: colors
	lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	true

copy-working-files:
	docker exec -it ulstu-${FLAVOR} ln -s /ulstu/repositories/${ROS2_PROJECT} /ulstu/ros2_ws/src/${ROS2_PROJECT} && \
	docker exec -it ulstu-${FLAVOR} ln -s /ulstu/repositories/robot_interfaces /ulstu/ros2_ws/src/robot_interfaces ; \

copy-working-folders:
	if [ -d ${PROJECT_DIR}/projects/${FLAVOR}/]; then \
		echo "project dir exists. Remove it first" ; \
	else \
		mkdir ${PROJECT_DIR}/projects/${FLAVOR} && \
		cp -r ${PROJECT_DIR}/simulation/${ROS2_PROJECT} ${PROJECT_DIR}/projects/${FLAVOR}/${ROS2_PROJECT}  && \
		cp -r ${PROJECT_DIR}/simulation/robot_interfaces ${PROJECT_DIR}/projects/${FLAVOR}/robot_interfaces ; \
#cp -r ${PROJECT_DIR}/docker ${PROJECT_DIR}/docker/projects/${FLAVOR}/docker ; \
	fi

start-code-server:
	docker exec -d -it ulstu-${FLAVOR} bash -c 'pgrep code-server || code-server /ulstu/ros2_ws' && \
	xdg-open 'localhost:31415?folder=/ulstu/ros2_ws'

stop-code-server:
	docker exec -it ulstu-${FLAVOR} pkill -f code-server

exec:
	docker exec -it ulstu-${FLAVOR} bash

destroy:
	docker container kill ulstu-${FLAVOR} 1>/dev/null || true
	docker container rm -f ulstu-${FLAVOR} 1>/dev/null || true

setup-default: colors
	docker exec -it ulstu-${FLAVOR} sh -c '/usr/bin/setup.sh --first-time-ros-setup'

setup-interactive: colors
	docker exec -it ulstu-${FLAVOR} sh -c '/usr/bin/setup.sh --interactive'
	printf '%b\n%b\n' "${GREEN}Interactive setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it ulstu-${FLAVOR}${NC} to access the container"
