#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <ros/ros.h>

#include <pcl/registration/registration.h>

#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/gicp_omp.h>

namespace lidar_odometry {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method(ros::NodeHandle& pnh);

/**
 * @brief ndtomp initialization according to rosparams
 * @param pnh
 * @return ndtomp scan matching
 */
boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt_omp_init(ros::NodeHandle& pnh);


}

#endif //
