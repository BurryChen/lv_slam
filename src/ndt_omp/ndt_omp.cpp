#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/ndt_omp_impl2.hpp>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZRGBL, pcl::PointXYZRGBL>;


#include <ndt_omp/ndt_ground.h>
#include <ndt_omp/ndt_ground_impl.hpp>

template class pclomp_ground::NormalDistributionsTransformGround<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp_ground::NormalDistributionsTransformGround<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp_ground::NormalDistributionsTransformGround<pcl::PointXYZRGBL, pcl::PointXYZRGBL>;