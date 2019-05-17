cmake_minimum_required(VERSION 2.8.3)

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

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

add_executable(s2k_lvo_kitti
  s2k_lvo_kitti.cpp
)
add_dependencies(s2k_lvo_kitti
  ndt_omp
)
target_link_libraries(s2k_lvo_kitti
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
  ${OpenCV_LIBS} 
  ${G2O_LIBS}
)

# direct_sparse
add_executable( direct_sparse direct_sparse.cpp )
target_link_libraries( direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} )