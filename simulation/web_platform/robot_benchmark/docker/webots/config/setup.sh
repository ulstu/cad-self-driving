#!/usr/bin/env bash

_configure_proxy=false
_vnc=false
_first_time_ros_setup=false
_enhanced_shell_prompt=false
_shell_shortcuts=false
_interactive=false

usage() {
	echo "Usage: $0 [--help] [--configure-proxy] [--vnc] [--first-time-ros-setup] [--interactive]"
	echo "--help print help and exit"
	echo "--interactive do everything interactively"
	echo "--no-default do not use default setup configurations"
	echo "--configure-proxy configures proxy for use in ftn network"
	echo "--vnc configures VNC server in the container for outside remote DE use"
	echo "--first-time-ros-setup configures ros for the first time setup"
	echo "--default configures ros for the first time setup and vnc"
	exit
}

default_configure_proxy() {
	sed '/# Setup_proxy/d' -i /ulstu/.bashrc
	sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
}

configure_proxy() {
	if $_interactive; then
		# FTN proxy is actually enabled here
		if dialog --title 'ulstu config' --defaultno --yesno 'Enable UNS proxy' 5 30; then
			sed '/# Setup_proxy/d' -i /ulstu/.bashrc
			sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
			echo "source /ulstu/ros2_ws/src/webots_ros2_suv/docker/config/proxy.sh &>/dev/null # Setup_proxy" >>/ulstu/.bashrc
			source /ulstu/ros2_ws/src/webots_ros2_suv/docker/config/proxy.sh
			echo 'Acquire::http::Proxy "http://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
			echo 'Acquire::https::Proxy "https://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
			return
		fi
	fi
	default_configure_proxy
}

default_first_time_ros_setup() {
	sudo -E rosdep init
	sudo -E apt-get install -y python3-vcstool
	rosdep --rosdistro "${ROS_DISTRO}" update
	cd /ulstu/ros2_ws && yes | rosdep --rosdistro "${ROS_DISTRO}" install -r --from-paths src --ignore-src
}

first_time_ros_setup() {
	if $_interactive; then
		if dialog --title 'ulstu config' --yesno 'Run first time ROS setup' 5 30; then
			default_first_time_ros_setup
			return
		fi
	fi
	default_first_time_ros_setup
}

interactive_vnc() {
	echo "Interactive VNC invoked"
	if dialog --title 'ulstu config' --yesno 'Enable VNC' 5 30; then
		VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
		if dialog --title 'ulstu config' --yesno 'Enable GPU acceleration' 5 30; then
			VNC_CMD="VGL=-vgl ${VNC_CMD}"
		fi
		if dialog --title 'ulstu config' --inputbox 'Enter host X11 display server location' 8 30 "$DISPLAY" 2>/tmp/answer; then
			VNC_CMD=" DISPLAY=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		if dialog --title 'ulstu config' --inputbox 'Enter VNC server port' 8 30 5910 2>/tmp/answer; then
			VNC_CMD=" PORT_VNC=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		if dialog --title 'ulstu config' --inputbox 'Enter noVNC webserver port' 8 30 6810 2>/tmp/answer; then
			VNC_CMD=" PORT_WEB=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		sed '/# Setup_VNC/d' -i /ulstu/.bashrc
		echo "$VNC_CMD # Setup_VNC" >>/ulstu/.bashrc
	else
		sed '/# Setup_VNC/d' -i /ulstu/.bashrc
	fi
}

default_vnc() {
	VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
	VNC_CMD="VGL=-vgl DISPLAY=$DISPLAY PORT_VNC=5910 PORT_WEB=6810 ${VNC_CMD}"
	sed '/# Setup_VNC/d' -i /ulstu/.bashrc
	echo "$VNC_CMD # Setup_VNC" >>/ulstu/.bashrc
}

vnc() {
	if test -f /opt/TurboVNC/bin/vncserver; then
		echo "TurboVNC is not installed"
		exit 1
	fi
	if $_interactive; then
		echo "TurboVNC is installed"
		interactive_vnc
		return
	fi
	default_vnc
}

finalize() {
	sed '/# Setup_shell/d' -i /ulstu/.bashrc
	echo 'echo "$-" | grep i -q  # Setup_shell' >>/ulstu/.bashrc
	clear
}

while [ "$#" -gt 0 ]; do
	case "$1" in
	--default)
		_first_time_ros_setup=true
		_vnc=false
		;;
	--configure-proxy)
		_configure_proxy=true
		;;
	--vnc)
		_vnc=true
		;;
	--first-time-ros-setup)
		_first_time_ros_setup=true
		;;
	--interactive)
		_interactive=true
		;;
	--help|*)
		usage
		;;
	esac
	shift
done

if $_interactive; then
	configure_proxy
	first_time_ros_setup
	vnc
else
	if $_configure_proxy; then
		configure_proxy
	fi
	if $_first_time_ros_setup; then
		first_time_ros_setup
	fi
	if $_vnc; then
		default_vnc
	fi
fi

finalize