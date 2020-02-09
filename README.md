
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

## 2. dlo_lfa_ggo_kitti
### 2.1 
roslaunch lv_slam dlo_lfa_ggo_kitti.launch  calib_file:='/home/whu/slam_ws/src/lv_slam/config/kitti_calib/calib00-02_13-21_corrected.txt'     odom_file:='/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/KITTI_00_odom.txt' seq:=00  lfa_output_path:='/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib'
rosbag play --clock '/home/whu/data/data_source_KITTI/velostereobag/velo_img_00.bag'    -r 1.0
rosservice call /global_graph/dump "destination: '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/dump_00'  "

rosservice call /global_graph/save_map "resolution: 0.05                                                                                 
destination: '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/dump_00/map.pcd'"

evo_traj kitti '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/KITTI_00_odom.txt' '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/aft_mapped_to_init_high_frec_file/data/KITTI_00_odom.txt'   '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/dump_00/ggo_wf_odom.txt'      --plot_mode=xz  --ref='/home/whu/data/data_source_KITTI/gt/00.txt'   -p --save_plot  '/home/whu/data/lv_slam_kitti/kitti_lv_dlo_lfa_ggo_calib/dlo_lfa_global/data/dump_00/ggo_wf_odom.pdf'

## 3. dlo_lfa_ggo_kylin
### 3.1
roslaunch lv_slam dlo_lfa_ggo_kylin.launch calib_file:='/home/whu/slam_ws/src/lv_slam/config/kylin_calib/calib.txt'    odom_file:='/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/kylin_02_odom.txt'   seq:=k2  lfa_output_path:='/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global'
rosbag play --clock  '/home/whu/data/lv_slam_kylin/selected_for_dissertation/k2_vlp16_2_imu_mynt_2020-01-09-15-54-50.bag'    /ns1/horizontal_laser_3d:=/velodyne_points    -r 1.0
rosservice call /global_graph/dump "destination: '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/dump_k3'   "

rosservice call /global_graph/save_map "resolution: 0.05                                                                                 
destination: '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/dump_k3/map.pcd'" 

evo_traj kitti '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/kylin_01_odom.txt' '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/aft_mapped_to_init_high_frec_file/data/KITTI_k1_odom.txt' '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/dump_k1/ggo_kf_odom.txt'    --plot_mode=xyz    -p --save_plot  '/home/whu/data/lv_slam_kylin/selected_for_dissertation/kylin_lv_dlo_lfa_global/dlo_lfa_global/data/dump_k1/ggo_kf_odom.pdf'