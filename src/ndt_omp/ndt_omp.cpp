#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/ndt_omp_impl2.hpp>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
