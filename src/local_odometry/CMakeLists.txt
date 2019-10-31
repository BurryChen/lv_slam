cmake_minimum_required(VERSION 2.8.3)

#############################
## 3.lv_slam(lib&exe) ##
#############################
# nodelets

add_library(local_odometry_nodelet
  local_odometry_nodelet.cpp
)
target_link_libraries(local_odometry_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ndt_pca
)