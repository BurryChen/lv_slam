cmake_minimum_required(VERSION 2.8.3)

####################
## 1.ndt_omp(lib) ##
####################
add_library(ndt_omp
  voxel_grid_covariance_omp.cpp
  ndt_omp.cpp
  gicp_omp.cpp
)

