
# lv_slam
## lidar_visual_slam
## An slam with Lidar-visual fusion and graph optimization

## 1. node introduction
### 1.1 ndt_omp & ndt_pca 底层匹配算法 及其改进

### 1.2 lidar_odometry 

### 1.3 global_graph
### A global graph lidar slam using visual loop dectection

# example 
roslaunch lv_slam global_graph_kitti.launch res_dir:='/home/whu/data/lv_slam_kitti/KITTI_lv_global'       seq:=04
rosbag play --clock   '/home/whu/data/data_source_KITTI/velostereobag/velo_img_04.bag'
rosservice call /global_graph/dump "destination: '/home/whu/data/lv_slam_kitti/KITTI_lv_global/data/dump_04'  "
