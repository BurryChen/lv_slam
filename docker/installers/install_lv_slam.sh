#!/usr/bin/env bash

set -e

# Change apt source to Tsinghua University mirror
sed -i "s@http://archive.ubuntu.com/ubuntu/@https://mirrors.tuna.tsinghua.edu.cn/ubuntu/@g" /etc/apt/sources.list
sed -i "s@http://packages.ros.org/ros/ubuntu@http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu@g" /etc/apt/sources.list.d/ros1-latest.list
#sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

apt-get update 
apt-get install -y --no-install-recommends \
    wget nano build-essential unzip \
    cmake python-catkin-tools \
    ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-rviz ros-melodic-tf-conversions \
    ros-melodic-cv-bridge \
    libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev 
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* 

# copy and unzip lib file
mkdir -p /home/chenshoubin/slam_ws/src && cd /home/chenshoubin/slam_ws/src
git clone https://github.com/BurryChen/A-LOAM.git 
git clone https://github.com/BurryChen/lv_slam.git 
mkdir -p /home/chenshoubin/tools && cd /home/chenshoubin/tools
cp -r ../slam_ws/src/lv_slam/3rdtools/*.zip .
cp -r ../slam_ws/src/lv_slam/3rdtools/*.gz .

tar -xzvf ceres-solver-1.14.0.tar.gz
unzip DBow3-master.zip 
unzip g2o-a48ff8c.zip
unzip Sophus-a621ff2-ubuntu18.04.zip

    cd ./ceres-solver-1.14.0
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j8 install
    cd ../../
      
    cd ./DBow3
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j8 install
    cd ../../
    
    cd ./g2o
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j8 install
    cd ../../
      
    cd ./Sophus
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j8 install
    cd ../../
      
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd /home/chenshoubin/slam_ws
#catkin_make -DCATKIN_WHITELIST_PACKAGES="A-LOAM" --build='./build/lv_slam' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install
#catkin_make -DCATKIN_WHITELIST_PACKAGES="lv_slam" --build='./build/lv_slam' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install
