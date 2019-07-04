#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace lidar_odometry {

class PrefilteringNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNodelet() {}
  virtual ~PrefilteringNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/velodyne_points", 128, &PrefilteringNodelet::cloud_callback, this);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 128);
  }

private:
  void initialize_params() {
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = private_nh.param<double>("radius_radius", 0.8);
      int min_neighbors = private_nh.param<int>("radus_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter = private_nh.param<bool>("use_distance_filter", true);
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);

    base_link_frame = private_nh.param<std::string>("base_link_frame", "");
    use_angle_calibration = private_nh.param<bool>("use_angle_calibration", false);
  }

  //void cloud_callback(pcl::PointCloud<PointT>::ConstPtr src_cloud)
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
  {
    pcl::PointCloud<PointT>::Ptr src_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*in_cloud_ptr, *src_cloud);
    if(src_cloud->empty()) {
      return;
    }

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!base_link_frame.empty()) {
      if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
      }

      tf::StampedTransform transform;
      tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = src_cloud->header.stamp;
      src_cloud = transformed;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered;
    if(use_angle_calibration) 
    {
      filtered = vertical_angle_calibration(src_cloud);  
      filtered = distance_filter(filtered);  
    }
    else
    {
      filtered = distance_filter(src_cloud);
    }
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    //points_pub.publish(filtered);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered, cloud_msg);
    cloud_msg.header = in_cloud_ptr->header;
    points_pub.publish(cloud_msg);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) {
        double d = p.getVector3fMap().norm();
        return d > distance_near_thresh && d < distance_far_thresh;
      }
    );

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }
  
  pcl::PointCloud<PointT>::ConstPtr vertical_angle_calibration(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    for (size_t cp = 0; cp < cloud->points.size (); ++cp)
    {
      PointT p=cloud->points[cp];
      double detal=0.11*M_PI/180;  //delta_vertical_angle=0.11
      /*double range = sqrt(p.x * p.x + p.y * p.y+ p.z * p.z);
      double cos_theta=p.x/sqrt(p.x * p.x + p.y * p.y);
      double sin_theta=p.y/sqrt(p.x * p.x + p.y * p.y);
      double phi = std::asin(p.z/range);     
      phi+=detal;
      PointT p1;
      p1.z=range*std::sin(phi);
      p1.x=range*std::cos(phi)*cos_theta;
      p1.y=range*std::cos(phi)*sin_theta;*/
          
      //Intrinsic calibration of the vertical angle of laser fibers (take the same correction for all lasers)
      Eigen::Vector3d rotationVector = (Eigen::Vector3d(p.x,  p.y, p.z)).cross(Eigen::Vector3d(0., 0., 1.));
      rotationVector.normalize();
      Eigen::Matrix3d rotationScan;
      rotationScan = Eigen::AngleAxisd(detal, rotationVector);
      Eigen::Vector3d p2Vector=rotationScan*Eigen::Vector3d(p.x,  p.y, p.z); 
      PointT p2;
      p2.x=p2Vector(0);p2.y=p2Vector(1);p2.z=p2Vector(2);   

      filtered->push_back(p2);            
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  tf::TransformListener tf_listener;

  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;
  bool use_angle_calibration;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

};

}

PLUGINLIB_EXPORT_CLASS(lidar_odometry::PrefilteringNodelet, nodelet::Nodelet)
