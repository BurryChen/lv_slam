#include <ndt_pca/ndt_pca.h>
#include <ndt_pca/ndt_pca_impl2.hpp>

template class pclpca::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclpca::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
