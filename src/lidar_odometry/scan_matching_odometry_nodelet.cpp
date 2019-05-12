#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <lidar_odometry/ros_utils.hpp>
#include <lidar_odometry/registrations.hpp>

#include <pcl/io/ply_io.h>
#include <ndt_omp/ndt_omp.h>

namespace lidar_odometry {

class ScanMatchingOdometryNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNodelet() {}
  virtual ~ScanMatchingOdometryNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/filtered_points", 256, &ScanMatchingOdometryNodelet::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 256);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 256);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    odom_file = pnh.param<std::string>("odom_file", "/home/whu/data/ndt_odom_KITTI/KITTI_0X_odom.txt");

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);
    
    windowmap_trans = pnh.param<double>("windowmap_trans", 5.0);
    windowmap_angle = pnh.param<double>("windowmap_angle", 0.17);
    windowmap_frame = pnh.param<double>("windowmap_frame", 10);

    // Registration validation by thresholding
    transform_thresholding = pnh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
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
      boost::shared_ptr<pcl::PassThrough<PointT>> passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    registration = select_registration_method(pnh);
    ndt_resolution = pnh.param<double>("ndt_resolution", 0.5);
    //reg_undistort= select_registration_method(pnh);
    //ndt_omp= ndt_omp_init(pnh);
    
    //pose file with KITTI calibration tf_cal
    fp = fopen(odom_file.c_str(),"w");
    //世界坐标系map,以第一帧velo为基准建立，而kitti ground truth 是以第一帧camera为世界坐标系的velo pose，需要世界系calibration参数
    Eigen::Matrix4d mat;
    mat<<      
     4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,-1.198459927713e-02,
    -7.210626507497e-03,  8.081198471645e-03, -9.999413164504e-01,-5.403984729748e-02, 
     9.999738645903e-01,  4.859485810390e-04, -7.206933692422e-03,-2.921968648686e-01,
      0,0,0,1;
     tf_velo2cam=mat.cast<double>();
    //初值
    tf::Matrix3x3 R(1,0,0,0,1,0,0,0,1);
    //R=tf_velo2cam.getBasis()*R;
    tf::Vector3 T(0,0,0);
    fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	   R[0][0],R[0][1],R[0][2],T[0],
	   R[1][0],R[1][1],R[1][2],T[1],
	   R[2][0],R[2][1],R[2][2],T[2]);
    seq=0;
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix4f pose = matching_scan2key(cloud_msg->header.stamp, cloud);//base系在odom系下的变换（odom=第一帧keyframe的base）
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = "/velodyne_points";
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr 
  downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }
  
    /**
   * @brief undistort a point cloud
   * @param cloud  input cloud
   * @return the size of undistorted point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr 
  undistort(const pcl::PointCloud<PointT>::ConstPtr& cloud,Eigen::Matrix4d delta_s2s){
    size_t cloudSize = cloud->size();

    //四元数
    Eigen::Isometry3d Trans_End2Srart(delta_s2s.cast<double>());
    Eigen::Matrix4d delta_S2E=delta_s2s.inverse();
    Eigen::Quaterniond p = Eigen::Quaterniond(Eigen::Matrix3d::Identity());  
    Eigen::Quaterniond q = Eigen::Quaterniond(delta_s2s.block<3, 3>(0, 0));  //Quaterniond from start to end 
    Eigen::Quaterniond q_S2E = Eigen::Quaterniond(delta_S2E.block<3, 3>(0, 0));  //Quaterniond from end to start
    double sita = std::acos(q.w());
    
    Eigen::Vector3d translation_End2Mid = 0.5*(delta_s2s.block<3, 1>(0, 3));
    Eigen::Quaterniond rotation_End2Mid;
    rotation_End2Mid.x()=(p.x()+q.x())/2;
    rotation_End2Mid.y()=(p.y()+q.y())/2;
    rotation_End2Mid.z()=(p.z()+q.z())/2;
    rotation_End2Mid.w()=(p.w()+q.w())/2;

    // extract valid points from input cloud
    pcl::PointCloud<PointT>::Ptr undistorted(new pcl::PointCloud<PointT>());
    undistorted->header = cloud->header;
    undistorted->height=cloud->height;
    undistorted->width=cloud->width;

    /**
    * atan2(-1,0)=-M_PI/2,atan2(0,1)=0,atan2(1,0)=M_PI/2,atan2(0,-1)=M_PI,
    * 线头的正对面是+X,上方是+Z，右手法则 physical 0 position= -X asix（-M_PI） (逆时针旋转)
    */
    float ori,scale=0.25;  
    float oriStart=-M_PI;
    float oriEnd=M_PI;
    for (int i = 0; i < cloudSize; i++) {
      PointT point = cloud->points[i];
      
      // skip NaN and INF valued points
      if (!pcl_isfinite(point.x) ||
          !pcl_isfinite(point.y) ||
          !pcl_isfinite(point.z)) {
        continue;
      }
      // skip zero valued points
      if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
        continue;
      }

      // calculate horizontal point angle   
      ori=std::atan2(point.y,point.x);
      ori=oriEnd-ori;
      // calculate relative scan scale/time based on point orientation    
      double t = scale*ori/ (2 * M_PI);
      //point.intensity=t;   
      //interpolate translation and rotation by scaleTime
      Eigen::Vector3d translation_t2End = t*(delta_S2E.block<3, 1>(0, 3));
      Eigen::Quaterniond rotation_t2End;
      rotation_t2End.x()=(std::sin((1-t)*sita)*p.x()+std::sin(t*sita)*q_S2E.x())/std::sin(sita);
      rotation_t2End.y()=(std::sin((1-t)*sita)*p.y()+std::sin(t*sita)*q_S2E.y())/std::sin(sita);
      rotation_t2End.z()=(std::sin((1-t)*sita)*p.z()+std::sin(t*sita)*q_S2E.z())/std::sin(sita);
      rotation_t2End.w()=(std::sin((1-t)*sita)*p.w()+std::sin(t*sita)*q_S2E.w())/std::sin(sita);
      Eigen::Vector3d p0(point.x,point.y,point.z);                        //in current frame
      Eigen::Vector3d p_toEnd=rotation_t2End*p0+translation_t2End;  //projected end frame
      Eigen::Vector3d p_toMid=rotation_End2Mid*p_toEnd+translation_End2Mid;  //projected end frame
      
      point.x=p_toEnd[0];
      point.y=p_toEnd[1];
      point.z=p_toEnd[2];
      undistorted->push_back(point);
    }

    return undistorted;
  }
  
  /**
   * @brief estimate the relative pose between an input cloud and a first keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud
   */
  Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    //std::cout<<" filtered size:"<<filtered->size()<<std::endl;
    registration->setInputSource(filtered);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());   
    auto t1 = ros::WallTime::now();
    registration->align(*aligned, prev_trans);//初值
    auto t2 = ros::WallTime::now();
    std::cout <<++seq<<" "<< stamp<<"/"<<keyframe_stamp << "-t : " << (t2 - t1).toSec() 
    << "-fitness: " << registration->getFitnessScore() << std::endl;

    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return keyframe_pose * prev_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();//当前scan相对keyframe的转换
    Eigen::Matrix4f odom = keyframe_pose * trans;//前一帧的pose*相邻scan转换
    
    //wite odom pose
    Eigen::Isometry3d tf_velo2odom(odom.cast<double>());
    Eigen::Isometry3d pose=tf_velo2cam*tf_velo2odom*tf_velo2cam.inverse();   
    auto data=pose.matrix();
    //std::cout<<"pose=\n"<<pose.matrix()<<std::endl;
    fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    data(0,0),data(0,1),data(0,2),data(0,3),
	    data(1,0),data(1,1),data(1,2),data(1,3),
	    data(2,0),data(2,1),data(2,2),data(2,3));  

    if(transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;//上一scan相对keyframe的转换×当前scan相对keyframe的转换=相邻scan的变换
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());
      std::cout<<"dx="<<dx<<"da="<<da<<std::endl;
      if(dx > max_acceptable_trans || da > max_acceptable_angle) {
        NODELET_INFO_STREAM("too large transform!!  " << dx << "[m] " << da << "[rad]");
        NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
        return keyframe_pose * prev_trans;
      }
    }

    prev_trans = trans;//下一次align的初值

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).toSec();
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_trans.setIdentity();
    }

    return odom;
  }

    /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and last scan
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4f matching_scan2scan(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!lastframe) { 
      lastframe_pose.setIdentity();
      lastframe= downsample(cloud);
      registration->setInputTarget(lastframe);
      prev_trans.setIdentity();
      last_t = ros::WallTime::now();
      return Eigen::Matrix4f::Identity();
    } 
      
    auto filtered = downsample(cloud);
    //std::cout<<" filtered size:"<<filtered->size()<<std::endl;
    registration->setInputSource(filtered);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());    
    registration->align(*aligned, prev_trans);//初值

    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return lastframe_pose * prev_trans;
    }
    Eigen::Matrix4f trans = registration->getFinalTransformation();//当前scan相对lastframe的转换

    double dx = trans.block<3, 1>(0, 3).norm();
    double da = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    if(dx > max_acceptable_trans || da > max_acceptable_angle) {
      trans=prev_trans;
      NODELET_INFO_STREAM("too large transform or gitness!! trans=prev_trans");
    }
    
    Eigen::Matrix4f odom = lastframe_pose * trans;//前一帧的pose*转换
    
    //write odom pose file
    Eigen::Isometry3d tf_velo2odom(odom.cast<double>());
    Eigen::Isometry3d pose=tf_velo2cam*tf_velo2odom*tf_velo2cam.inverse();   
    auto data=pose.matrix();
    //std::cout<<"pose=\n"<<pose.matrix()<<std::endl;
    fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    data(0,0),data(0,1),data(0,2),data(0,3),
	    data(1,0),data(1,1),data(1,2),data(1,3),
	    data(2,0),data(2,1),data(2,2),data(2,3));  
      
    lastframe_pose = odom;
    //lastframe= filtered;
    registration->setInputTarget(filtered);
    prev_trans = trans;//下一次align的初值  
    seq++;

    // judge and publish keyframe,per 10
    if(!keyframe)   //initialize
    {
      keyframe_stamp = stamp;
      keyframe_pose.setIdentity();
      keyframe = filtered;
    }
    if(seq%10==0) {      
      keyframe_stamp = stamp;
      keyframe_pose=odom;
      keyframe = filtered; 
    }
    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);

    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout <<seq<<" "<< stamp << "--t: " << (t2 - last_t).toSec() 
    << "--fitness: " << registration->getFitnessScore() 
    <<"--dx: "<<dx<<"--da: "<<da<<std::endl;
    last_t=t2;
    
    return odom;
  }
  
   /**
   * @brief estimate the relative pose between an input cloud and a first keyframe cloud
   *        matching between current scan and selected keyframe by keyframe_delta_trans/angle/time with more accurate init
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud
   */
  Eigen::Matrix4f matching_scan2key(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_trans.setIdentity();
      guess_trans.setIdentity();
      guess_trans(0,3)=1.5;
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      last_t = ros::WallTime::now();
      return Eigen::Matrix4f::Identity();
    }

    //auto filtered = downsample(cloud);
    //std::cout<<" filtered size:"<<filtered->size()<<std::endl;
    registration->setInputSource(cloud);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());   
    registration->align(*aligned, guess_trans);//初值

    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return keyframe_pose * guess_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();//当前scan相对keyframe的转换
    Eigen::Matrix4f odom = keyframe_pose * trans;//前一帧的pose*相邻scan转换
    
    //wite odom pose
    Eigen::Isometry3d tf_velo2odom(odom.cast<double>());
    Eigen::Isometry3d pose=tf_velo2cam*tf_velo2odom*tf_velo2cam.inverse();   
    auto data=pose.matrix();
    //std::cout<<"pose=\n"<<pose.matrix()<<std::endl;
    fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    data(0,0),data(0,1),data(0,2),data(0,3),
	    data(1,0),data(1,1),data(1,2),data(1,3),
	    data(2,0),data(2,1),data(2,2),data(2,3));  

    //delta between current scan and last scan
    Eigen::Matrix4f delta_s2s = prev_trans.inverse() * trans;//上一scan相对keyframe的转换×当前scan相对keyframe的转换=相邻scan的变换
    double dx_s2s = delta_s2s.block<3, 1>(0, 3).norm();
    double da_s2s = 2*std::acos(Eigen::Quaternionf(delta_s2s.block<3, 3>(0, 0)).w());
    //delta between current scan and keyframe
    double dx_s2k = trans.block<3, 1>(0, 3).norm();
    double da_s2k = 2*std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double dt_s2k = (stamp - keyframe_stamp).toSec();
    
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout <<++seq<<" "<< stamp<<"/"<<keyframe_stamp << "--t: " << (t2 - last_t).toSec() 
    <<"--fitness(dx_s2s): " << std::sqrt(registration->getFitnessScore(dx_s2s)) 
    <<"--dx_s2s: "<<dx_s2s<<"--da_s2s: "<<da_s2s
    <<"--dx_s2k: "<<dx_s2k<<"--da_s2k: "<<da_s2k<<"--dt_s2k: "<<dt_s2k<<std::endl;
    last_t=t2;
    
    if(transform_thresholding&&(dx_s2s > max_acceptable_trans || da_s2s > max_acceptable_angle)) {
      NODELET_INFO_STREAM("too large transform!!  " << dx_s2s << "[m] " << da_s2s << "[rad]");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return keyframe_pose * guess_trans;
    }

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);

    if(dx_s2k > keyframe_delta_trans || da_s2k > keyframe_delta_angle || dt_s2k > keyframe_delta_time) {
      keyframe = cloud;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_trans.setIdentity();
    }
    else
    {
      prev_trans = trans;       //用于计算delta_s2s
    }       
    guess_trans=prev_trans*delta_s2s;  //下一次align的估计初值
        
    return odom;
  }
  
   /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and mobile window map including n scans
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4f matching_scan2winmap(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_trans.setIdentity();
      guess_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      last_t = ros::WallTime::now();
      
      scan_queue.push_back(keyframe);
      odom_queue.push_back(prev_trans);
      windowmap+=*keyframe;
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    //std::cout<<" filtered size:"<<filtered->size()<<std::endl;
    registration->setInputSource(filtered);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());   
    registration->align(*aligned, guess_trans);//初值

    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return guess_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();//absolute trans
    Eigen::Matrix4f odom =  trans;//前一帧的pose*相邻scan转换
    
    //wite odom pose
    Eigen::Isometry3d tf_velo2odom(odom.cast<double>());
    Eigen::Isometry3d pose=tf_velo2cam*tf_velo2odom*tf_velo2cam.inverse();   
    auto data=pose.matrix();
    //std::cout<<"pose=\n"<<pose.matrix()<<std::endl;
    fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    data(0,0),data(0,1),data(0,2),data(0,3),
	    data(1,0),data(1,1),data(1,2),data(1,3),
	    data(2,0),data(2,1),data(2,2),data(2,3));  

    //delta between current scan and last scan
    Eigen::Matrix4f delta_s2s = prev_trans.inverse() * trans;//上一scan相对keyframe的转换×当前scan相对keyframe的转换=相邻scan的变换
    double dx_s2s = delta_s2s.block<3, 1>(0, 3).norm();
    double da_s2s = std::acos(Eigen::Quaternionf(delta_s2s.block<3, 3>(0, 0)).w());
    //delta between current scan and keyframe
    Eigen::Matrix4f delta_s2k=keyframe_pose.inverse() * odom;
    double dx_s2k = delta_s2k.block<3, 1>(0, 3).norm();
    double da_s2k = std::acos(Eigen::Quaternionf(delta_s2k.block<3, 3>(0, 0)).w());
    double dt_s2k = (stamp - keyframe_stamp).toSec();
    
    if(transform_thresholding&&(dx_s2s > max_acceptable_trans || da_s2s > max_acceptable_angle)) {
      NODELET_INFO_STREAM("too large transform!!  " << dx_s2s << "[m] " << da_s2s << "[rad]");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return  guess_trans;
    }

    // mobile windowmap including n scans as target pc
    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud (*filtered, *transformed, trans);  //转到odom系下，第二个参数不能为常量
    windowmap+=*transformed;
    scan_queue.push_back(transformed);
    odom_queue.push_back(odom);
    
    //delta between current scan and keyframe
    Eigen::Matrix4f delta_wp=odom_queue[0].inverse() * odom;
    double dx_wp = delta_wp.block<3, 1>(0, 3).norm();
    double da_wp = std::acos(Eigen::Quaternionf(delta_wp.block<3, 3>(0, 0)).w());
    int df_wp=scan_queue.size();
    while(dx_wp > windowmap_trans || da_wp > windowmap_angle || df_wp>windowmap_frame)
    {
      windowmap.erase(windowmap.begin(),windowmap.begin()+(scan_queue.front())->size());
      scan_queue.pop_front();
      odom_queue.pop_front();
      delta_wp=odom_queue[0].inverse() * odom;
      dx_wp = delta_wp.block<3, 1>(0, 3).norm();
      da_wp = std::acos(Eigen::Quaternionf(delta_wp.block<3, 3>(0, 0)).w());
      df_wp=scan_queue.size();
    }
    registration->setInputTarget(windowmap.makeShared());
 
    prev_trans = trans;       //用于计算delta_s2s  
    guess_trans=prev_trans*delta_s2s;  //下一次align的估计初值
    
    //发布更新keyframe
    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);    
    if(dx_s2k > keyframe_delta_trans || da_s2k > keyframe_delta_angle || dt_s2k > keyframe_delta_time) {          
      keyframe = filtered;
      keyframe_pose = odom;
      keyframe_stamp = stamp;    
      //registration->setInputTarget(transformed);
    }
          
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout <<++seq<<" "<< stamp<<"/"<<keyframe_stamp << "--t: " << (t2 - last_t).toSec() 
    <<"--fitness: " << registration->getFitnessScore() 
    <<"--dx_s2s: "<<dx_s2s<<"--da_s2s: "<<da_s2s
    //<<"--dx_s2k: "<<dx_s2k<<"--da_s2k: "<<da_s2k<<"--dt_s2k: "<<dt_s2k<<std::endl;
    <<"--dx_wp: "<<dx_wp<<"--da_wp: "<<da_wp<<"--df_wp: "<<df_wp<<std::endl;
    last_t=t2;
    
    return odom;
  }
  
  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);
  }


private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;

  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;

  std::string odom_frame_id;
  ros::Publisher read_until_pub;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud
  pcl::PointCloud<PointT>::ConstPtr lastframe;  // lastframe point cloud
  Eigen::Matrix4f lastframe_pose;               // lastframe pose

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_omp;
  
  //odom pose file with KITTI calibration tf_cal
  FILE *fp;
  Eigen::Isometry3d tf_velo2cam;
  int seq;
  double ndt_resolution;
  ros::WallTime last_t;
  Eigen::Matrix4f guess_trans;                  //init guess for mathing
  pcl::PointCloud<PointT> windowmap;            // windowmap point cloud
  std::deque<pcl::PointCloud<PointT>::ConstPtr> scan_queue;
  std::deque<Eigen::Matrix4f> odom_queue; 
  double windowmap_trans;
  double windowmap_angle;
  int windowmap_frame; 
  
  std::string odom_file;
  Eigen::Matrix4f delta_s2s;                     //distortion tranformation
  pcl::Registration<PointT, PointT>::Ptr reg_undistort;  //registration between undistorted keyframe
  pcl::PointCloud<PointT> keymap;           // keymap point cloud

};

}

PLUGINLIB_EXPORT_CLASS(lidar_odometry::ScanMatchingOdometryNodelet, nodelet::Nodelet)
