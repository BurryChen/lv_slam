cmake_minimum_required(VERSION 2.8.3)

#############################
## 3.lv_slam(lib&exe) ##
#############################
# nodelets
add_library(prefiltering_nodelet prefiltering_nodelet.cpp)
target_link_libraries(prefiltering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(scan_matching_odom_nodelet
  scan_matching_odom_nodelet.cpp
)
target_link_libraries(scan_matching_odom_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ndt_pca
)