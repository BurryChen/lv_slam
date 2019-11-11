
# lv_slam
## lidar_visual_slam
## An slam with Lidar-visual fusion and graph optimization

## 1. node introduction
### 1.1 ndt_omp & ndt_pca 底层匹配算法 及其改进

### 1.2 lidar_odometry 

### 1.3 local_mapping

### 1.4 global_graph
### A global graph lidar slam using visual loop dectection

## 2. example 
### 2.1 
'/home/whu/slam_ws/src/lv_slam/scripts/lidar_odom_kitti.sh'  '/home/whu/data/lv_slam_kitti/KITTI_lv_odom' 

### 2.2 
roslaunch lv_slam local_mapping_kitti.launch res_dir:='/home/whu/data/lv_slam_kitti/KITTI_lv_local'       seq:=04
rosbag play --clock   '/home/whu/data/data_source_KITTI/velostereobag/velo_img_04.bag'

evo_traj kitti '/home/whu/data/lv_slam_kitti/KITTI_lv_local/data/KITTI_04_odom.txt' '/home/whu/data/lv_slam_kitti/KITTI_lv_local/data/KITTI_04_odom_after_local.txt'      --plot_mode=xz  --ref='/home/whu/data/data_source_KITTI/gt/04.txt'   --save_plot  '/home/whu/data/lv_slam_kitti/KITTI_lv_local/data/KITTI_04_odom_after_local.pdf'

### 2.3
roslaunch lv_slam global_graph_kitti.launch res_dir:='/home/whu/data/lv_slam_kitti/KITTI_lv_global'       seq:=04
rosbag play --clock   '/home/whu/data/data_source_KITTI/velostereobag/velo_img_04.bag'
rosservice call /global_graph/dump "destination: '/home/whu/data/lv_slam_kitti/KITTI_lv_global/data/dump_04'  "
