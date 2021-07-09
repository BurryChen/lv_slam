#!/usr/bin/env bash

set -e
  
# 安装ROS，配置ROS环境
apt-get -y update && \
    apt-get install -y --no-install-recommends \
    gnupg2  \
    gnupg-agent \
    curl

sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get -y update && \
    apt-get -y install ros-melodic-desktop-full --allow-unauthenticated && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

apt-get -y update && \
    apt-get install -y --no-install-recommends \
    wget nano build-essential unzip \
    cmake python-catkin-tools \
    ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-rviz ros-melodic-tf-conversions \
    ros-melodic-cv-bridge \
    libgoogle-glog-dev libatlas-base-dev \
    libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libcxsparse3 libcholmod3

apt-get clean && \
    rm -rf /var/lib/apt/lists/*    