#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>
using namespace cv;

namespace g2o
{
  class VertexSE3;
  class HyperGraph;
  class SparseOptimizer;
}

namespace lv_slam
{

  /**
 * @brief KeyFrame (pose node)
 */
  struct KeyFrame
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointT = pcl::PointXYZI;
    using Ptr = std::shared_ptr<KeyFrame>;

    KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr &cloud);
    KeyFrame(const ros::Time &stamp, const int &seq, const Eigen::Isometry3d &odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr &cloud, const cv::Mat &descriptor);
    KeyFrame(const std::string &directory, g2o::HyperGraph *graph);
    virtual ~KeyFrame();

    void save(const std::string &directory);
    bool load(const std::string &directory, g2o::HyperGraph *graph);

    long id() const;
    Eigen::Isometry3d estimate() const;

  public:
    ros::Time stamp;                               // timestamp
    int seq;                                       // seq int bag message
    Eigen::Isometry3d odom;                        // odometry (estimated by scan_matching_odometry) 优化前odom系下的pose
    Eigen::Isometry3d odom_after_local;            // odometry after local mapping
    double accum_distance;                         // accumulated distance from the first node (by scan_matching_odometry)
    pcl::PointCloud<PointT>::ConstPtr cloud;       // point cloud
    cv::Mat descriptor;                            // image
    boost::optional<Eigen::Vector4d> floor_coeffs; // detected floor's coefficients
    boost::optional<Eigen::Vector3d> utm_coord;    // UTM coord obtained by GPS

    boost::optional<Eigen::Vector3d> acceleration;   //
    boost::optional<Eigen::Quaterniond> orientation; //

    g2o::VertexSE3 *node; // node instance,特殊欧式群special european，转换矩阵,优化后map系下的pose
  };

  /**
 * @brief KeyFramesnapshot for map cloud generation
 */
  struct KeyFrameSnapshot
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using PointT = KeyFrame::PointT;
    using Ptr = std::shared_ptr<KeyFrameSnapshot>;

    KeyFrameSnapshot(const KeyFrame::Ptr &key);
    KeyFrameSnapshot(const Eigen::Isometry3d &pose, const pcl::PointCloud<PointT>::ConstPtr &cloud);

    ~KeyFrameSnapshot();

  public:
    Eigen::Isometry3d pose;                  // pose estimated by graph optimization
    pcl::PointCloud<PointT>::ConstPtr cloud; // point cloud
  };

}

#endif // KEYFRAME_HPP
