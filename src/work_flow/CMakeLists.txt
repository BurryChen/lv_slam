cmake_minimum_required(VERSION 2.8.3)

# exe
add_executable(match
  match.cpp
)
add_dependencies(match
  ndt_omp
)
target_link_libraries(match
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)


add_executable(s2s_lo_kitti
  s2s_lo_kitti.cpp
)
add_dependencies(s2s_lo_kitti
  ndt_omp
)
target_link_libraries(s2s_lo_kitti
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)

add_executable(s2s_lvo_kitti
  s2s_lvo_kitti.cpp
)
add_dependencies(s2s_lvo_kitti
  ndt_omp
)
target_link_libraries(s2s_lvo_kitti
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)