cmake_minimum_required(VERSION 2.8.3)
project(lv_slam)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O0")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "Release")

find_package(Sophus REQUIRED PATHS "/home/whu/tools/Sophus/build")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  #geodesy
  nmea_msgs
  sensor_msgs
  message_generation
  Sophus
  cv_bridge
  image_transport
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
# link_libraries(${G2O_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################
## message generation, 手动加载##
################################
#add_message_files(FILES
#  xx.msg
#)
add_message_files(FILES
  FloorCoeffs.msg
)

add_service_files(FILES
  SaveMap.srv
  DumpGraph.srv
)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_omp ndt_pca ndt_registration
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

####################
## 1.ndt_omp(lib) ##
####################
add_subdirectory(src/ndt_omp)
add_subdirectory(src/ndt_pca)

#############################
## 2.lidar_odometry(lib&exe) ##
#############################
# nodelets
add_subdirectory(src/lidar_odometry)


add_subdirectory(src/work_flow)
add_subdirectory(test)

add_subdirectory(src/local_mapping)
add_subdirectory(src/global_graph)