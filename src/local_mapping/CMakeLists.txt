cmake_minimum_required(VERSION 2.8.3)

#############################
## 3.lv_slam(lib&exe) ##
#############################
# nodelets
# opencv 
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dbow3 
# dbow3 is a simple lib so I assume you installed it in default directory 
#set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
#set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )
find_package( DBoW3 REQUIRED PATHS "/home/whu/tools/DBow3/build")
include_directories( ${DBoW3_INCLUDE_DIRS} )

add_library(local_mapping_nodelet
  local_mapping_nodelet.cpp
)
target_link_libraries(local_mapping_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ndt_pca
  global_graph_nodelet
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
)