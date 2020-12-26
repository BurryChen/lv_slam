cmake_minimum_required(VERSION 2.8.3)

#############################
## 3.global_graph(lib&exe) ##
#############################
# nodelets
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(STATUS "OpenCV_FOUND:" ${OpenCV_FOUND})
message(STATUS "OpenCV_INCLUDE_DIRS:" ${OpenCV_INCLUDE_DIRS})

# dbow3 
# dbow3 is a simple lib so I assume you installed it in default directory 
#set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
#set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )
find_package( DBoW3 REQUIRED)
include_directories( ${DBoW3_INCLUDE_DIRS} )

add_library(global_graph_nodelet 
  global_graph_nodelet.cpp
  graph_slam.cpp
  keyframe.cpp
  map_cloud_generator.cpp
  registrations.cpp
  information_matrix_calculator.cpp
  g2o/robust_kernel_io.cpp
)
target_link_libraries(global_graph_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ndt_omp
  ndt_pca
  ${OpenCV_LIBS}
  ${DBoW3_LIBS}
)
add_dependencies(global_graph_nodelet ${PROJECT_NAME}_gencpp)