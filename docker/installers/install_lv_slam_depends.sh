#!/usr/bin/env bash

set -e

#copy and unzip lib file
cd /home/chenshoubin/tools

tar -xzvf ceres-solver-1.14.0.tar.gz
unzip DBow3-master.zip 
unzip g2o-a48ff8c.zip
unzip Sophus-a621ff2-ubuntu18.04.zip

    cd ./ceres-solver-1.14.0
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j12 install
    cd ../../
      
    cd ./DBow3
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j12 install
    cd ../../

    cd ./Sophus
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install  ..
    make -j12 install
    cd ../../

    # #g2o源代码文件组织不规范,一定安装在默认路径下才能便于工程的cmakelist.txt查找库.
    # cd ./g2o
    # mkdir build && cd build
    # cmake -DCMAKE_BUILD_TYPE=Release ..
    # make -j12 install
    # cd ../../