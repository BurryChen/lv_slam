# hdl_graph_slam

Original repository: https://github.com/koide3/hdl_graph_slam


## Build
```bash
cd hdl_graph_slam/docker
./build.sh

cd /home/chenshoubin/slam_docker_ws#
bash ./docker/installers/install_lv_slam_vscode.sh
catkin_make  -DCATKIN_WHITELIST_PACKAGES="lv_slam"  --build='./build/lv_slam'  - DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install

catkin_make  -DCATKIN_WHITELIST_PACKAGES="aloam_velodyne"  --build='./build/A-LOAM'  - DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install
```

## Run

### On host:
```bash
roscore
```

```bash
rosparam set use_sim_time true

cd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_400.bag

roslaunch lv_slam dlo_lfa_ob.launch calib_file:='./src/lv_slam/config/kylin_calib/calib.txt'   odom_file:='/home/chenshoubin/data/ob_lv_dlo_lfa/dlo_lfa_global/data/ob_01_odom.txt'    seq:=o1  lfa_output_path:='/home/chenshoubin/data/ob_lv_dlo_lfa'
    
rosbag play --clock  ./data/ob_01_rs_kinect_4_2020-08-18-16-39-08.bag    /rslidar_points:=/velodyne_points    -r 1.0

roslaunch lv_slam dlo_lfa_ggo_ob.launch calib_file:='./src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/chenshoubin/data/ob_lv_dlo_lfa_ggo/dlo_lfa_global/data/ob_03_odom.txt'    seq:=o3  lfa_output_path:='/home/chenshoubin/data/ob_lv_dlo_lfa_ggo' img_topic:=/ns0/rgb/image_rect_color

rosbag play --clock  '/media/chenshoubin/Research/data/ob_03_rs_kinect4_rect_rgbd_2020-10-27-15-36-09.bag'     /rslidar_points:=/velodyne_points    -r 1.0
```
http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz

### On docker image:
```bash
cd hdl_graph_slam/docker
./run.sh

roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```


![hdl_graph_slam](https://user-images.githubusercontent.com/31344317/98347836-4fed5a00-205b-11eb-931c-158f6cd056bf.gif)
