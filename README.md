
# lv_slam
## lidar_visual_slam
## An slam with Lidar-visual fusion and graph optimization

## 1. node introduction
### 1.1 ndt_omp & ndt_pca 
Follow [ndt_omp Repositoty](https://github.com/koide3/ndt_omp,https://github.com/BurryChen/ndt_omp).
### classical ndt, weighted ndt

### 1.2 lidar_odometry 

### 1.3 global_graph
### A global graph lidar slam using visual loop dectection

### 1.4 test
### test/pose_estimation_2d3d_l2v_ceres
### test/pose_estimation_ceres
### test/dlg_segmentation,


## 2 Requirements
***lv_slam*** requires the following libraries:
- OpenMP
- PCL
- g2o
- suitesparse
- Sophus
- OpenCV 
- DBoW3
- A-LOAM

### 2.0. base
```
    sudo apt-get install -y --no-install-recommends \
         libsuitesparse-dev \
         ros-$ROS_DISTRO-geodesy ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-nmea-msgs ros-$ROS_DISTRO-libg2o
```

### 2.1. **Ceres Solver** installed to /usr/local/libname
```
    sudo apt-get install -y --no-install-recommends \
         cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev 
```
```  
    #下载 ceres-solver-1.14.0.tar.gz并解压
    # http://www.ceres-solver.org/installation.html#linux
    cd ceres-solver-1.14.0
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/ceres-solver ..
    make -j12
    sudo make install
   // cd ..&& sudo rm -r ./build
```

### 2.2. **g2o** installed to default path
```
    sudo apt-get install ros-$ROS_DISTRO-libg2o
```

### 2.3. **Sophus** installed to /usr/local/libname
```
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout a621ff
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/Sophus ..
    make -j12
    sudo make install
    //cd ..&& sudo rm -r ./build
```

### 2.4. **DBow3** installed to /usr/local/libname
**OpenCV**
```
   #version default is 3.2.0 in ubuntu18.04, 4.5.0 in ubuntu20.04
   sudo apt install libopencv-dev python3-opencv
```

**DBow3**
```
    git clone https://github.com/rmsalinas/DBow3
    cd DBow3
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/DBow3 ..
    make -j12
    sudo make install
    // cd ..&& sudo rm -r ./build
```

### 2.5. **A-LOAM**
Follow [A-LOAM Repositoty](https://github.com/BurryChen/A-LOAM).

```
cd workspace
cd src
git clone https://github.com/BurryChen/A-LOAM.git
cd A-LOAM
#set status on 1df43a1 in ubuntu18.04, master in ubuntu20.04
git checkout 1df43a1 
cd ../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="aloam_velodyne" --build='./build/A-LOAM' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install  install
```

### 2.6. **evo**
Follow [evo Repositoty](https://github.com/MichaelGrupp/evo).


## 3. Build 
Clone the repository and catkin_make:

```
    cd ~/slam_ws/src
    git clone https://github.com/BurryChen/lv_slam.git
    cd ../
catkin_make -DCATKIN_WHITELIST_PACKAGES="lv_slam" --build='./build/lv_slam' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install  install
    source ~/slam_ws/devel/setup.bash
```
Download [lv_slam_data](https://drive.google.com/drive/folders/1YTTXD4QmFrA6LQrfslRlUzATxYG0u32R?usp=sharing) to YOUR_DATASET_FOLDER.

## 4. Example dlo_lfa_ggo_kitti

Download [velo_img_04](https://drive.google.com/file/d/1PD9RHqhYCuFaXSo95ARs3ALMUDD9y5mn/view?usp=sharing) to YOUR_DATASET_FOLDER.
### 4.1 dlo_lfa_kitti
```
# realtive path
roslaunch lv_slam dlo_lfa_kitti.launch  calib_file:='$(find lv_slam)/config/kitti_calib/calib04-12.txt'     odom_file:='$(find lv_slam)/data/kitti_lv_dlo_lfa/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='$(find lv_slam)/data/kitti_lv_dlo_lfa'
rosbag play --clock './src/lv_slam/data/velo_img_04.bag'    -r 1.0
```

```
    roslaunch lv_slam dlo_lfa_kitti.launch  calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kitti_calib/calib04-12.txt'     odom_file:='/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa'
    rosbag play --clock '/home/chenshoubin/data/data_source_KITTI/velostereobag/velo_img_04.bag'    -r 1.0

    #cpp ./evaluate_odometry_seq '/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa/aft_mapped_to_init_high_frec_file' 04
    #seq 04 (t_avg,r_avg)=(0.003118,0.000026)
```
   (t,r)=(0.008990,0.000058)
   ALOAM config:seq01->lfa_3,others->lfa_1

### 4.2 dlo_lfa_ggo_kitti
```
# realtive path
roslaunch lv_slam dlo_lfa_ggo_kitti.launch  calib_file:='$(find lv_slam)/config/kitti_calib/calib04-12.txt'     odom_file:='$(find lv_slam)/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='$(find lv_slam)/data/kitti_lv_dlo_lfa_ggo'
rosbag play --clock './src/lv_slam/data/velo_img_04.bag'    -r 1.0
```

```
    roslaunch lv_slam dlo_lfa_ggo_kitti.launch  calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kitti_calib/calib04-12.txt'     odom_file:='/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo'
    rosbag play --clock '/home/chenshoubin/data/data_source_KITTI/velostereobag/velo_img_04.bag'    -r 1.0
    
    rosservice call /global_graph/dump "destination: '/home/chenshoubin/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_06'  "
    rosservice call /global_graph/save_map "{resolution: 0.05, destination: '/home/chenshoubin/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_06/map.pcd'}"

    evo_traj kitti '/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' '/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo/aft_mapped_to_init_high_frec_file/data/KITTI_04_odom.txt'   '/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_04/ggo_wf_odom.txt'      --plot_mode=xz  --ref='/home/chenshoubin/data/data_source_KITTI/gt/04.txt'   -p --save_plot  '/home/chenshoubin/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_04/ggo_wf_odom.pdf'
```

## 5. Example dlo_lfa_ggo_kylin
### 5.1 dlo_lfa_kylin
```
    roslaunch lv_slam dlo_lfa_kylin.launch  calib_file:='$(find lv_slam)/config/kylin_calib/calib.txt'     odom_file:='$(find lv_slam)/data/kylin_lv_dlo_lfa/dlo_lfa_global/data/kylin_02_odom.txt' seq:=k2  lfa_output_path:='$(find lv_slam)/data/kylin_lv_dlo_lfa'

    rosbag play --clock  '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/k2_vlp16_2_imu_mynt_2020-01-09-15-54-50.bag'    /ns1/horizontal_laser_3d:=/velodyne_points    -r 1.0
```

### 5.1 dlo_lfa_ggo_kylin
```
    roslaunch lv_slam dlo_lfa_ggo_kylin.launch calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/kylin_02_odom.txt'    seq:=k2  lfa_output_path:='/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo' 
    rosbag play --clock  '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/k2_vlp16_2_imu_mynt_2020-01-09-15-54-50.bag'    /ns1/horizontal_laser_3d:=/velodyne_points    -r 1.0
    rosservice call /global_graph/dump "destination: '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_k2'   "

    rosservice call /global_graph/save_map "resolution: 0.05                                                                                 
    destination: '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_k2/map.pcd'" 

    evo_traj kitti '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/kylin_02_odom.txt' '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/aft_mapped_to_init_high_frec_file/data/KITTI_k2_odom.txt' '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_k2/ggo_kf_odom.txt'    --plot_mode=xyz    -p --save_plot  '/home/chenshoubin/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_k2/ggo_kf_odom.pdf'
```

## 6 Example test
### 6.1  pose_estimation_2d3d_l2v_ceres
```
    '/home/chenshoubin/slam_ws/devel/lib/lv_slam/pose_estimation_2d3d_l2v_ceres' 'DataPath/02calib_2d3d_l2v'
```

### 6.2  pose_estimation_ceres
```
    '/home/chenshoubin/slam_ws/devel/lib/lv_slam/pose_estimation_ceres' 'DataPath/03calib_LRF/cp_all' 'DataPath/03calib_LRF/cp_all/cp_all.csv'
    '/home/chenshoubin/slam_ws/devel/lib/lv_slam/pose_estimation_ceres' 'DataPath/03calib_LRF/cp_r_R' 'DataPath/03calib_LRF/cp_r_R/cp_r_R.csv'
```

### 6.3  dlg_segmentation
```
    '/home/chenshoubin/slam_ws/devel/lib/lv_slam/dlg_segmentation' 'DataPath/04dlg_2D_UTM30LX/_2018-01-06-18-22-17.bag_points.ply'
```

## 7. Example dlo_lfa_ggo_ob
### 7.1 dlo_lfa_ob
```
    roslaunch lv_slam dlo_lfa_ob.launch calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/chenshoubin/data/ob_lv_dlo_lfa/dlo_lfa_global/data/ob_01_odom.txt'    seq:=o1  lfa_output_path:='/home/chenshoubin/data/ob_lv_dlo_lfa'
    
    rosbag play --clock  /home/chenshoubin/data/ob_2020-07-03-17-28-30.bag    /rslidar_points:=/velodyne_points    -r 1.0
```
### 7.2 dlo_lfa_ggo_ob
```
    roslaunch lv_slam dlo_lfa_ggo_ob.launch calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/chenshoubin/data/ob_lv_dlo_lfa/dlo_lfa_global/data/ob_02_odom.txt'    seq:=o2  lfa_output_path:='/home/chenshoubin/data/ob_lv_dlo_lfa'
    
    rosbag play --clock  /home/chenshoubin/data/ob_02_rs_kinect_4_2020-08-18-16-45-02.bag    /rslidar_points:=/velodyne_points    -r 1.0
    
    rosservice call /global_graph/dump "destination: '/home/chenshoubin/data/ob_lv_dlo_lfa/dlo_lfa_global/data/dump_o2' " 
    
    rosservice call /global_graph/save_map '{resolution: 0.05, destination: '/home/chenshoubin/data/ob_lv_dlo_lfa/dlo_lfa_global/data/dump_o2/map.pcd'}'
```

```
    roslaunch lv_slam dlo_lfa_ggo_ob.launch calib_file:='/home/chenshoubin/slam_ws/src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/chenshoubin/data/ob_lv_dlo_lfa_ggo/dlo_lfa_global/data/ob_03_odom.txt'    seq:=o3  lfa_output_path:='/home/chenshoubin/data/ob_lv_dlo_lfa_ggo' img_topic:=/ns0/rgb/image_rect_color
    
    rosbag play --clock  '/media/chenshoubin/Research/data/ob_03_rs_kinect4_rect_rgbd_2020-10-27-15-36-09.bag'     /rslidar_points:=/velodyne_points    -r 1.0

    rosservice call /global_graph/dump "destination: '/home/chenshoubin/data/ob_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_o3' " 
    
    rosservice call /global_graph/save_map '{resolution: 0.05, destination: '/home/chenshoubin/data/ob_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_o3/map.pcd'}'
```
