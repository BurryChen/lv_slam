#include <memory>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <lidar_odometry/ros_utils.hpp>

#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/gicp_omp.h>
#include <ndt_pca/ndt_pca.h>
#include <ndt_omp/ndt_ground.h>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;

namespace lidar_odometry {

class ScanMatchingOdomNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdomNodelet() {}
  virtual  ~ScanMatchingOdomNodelet() 
  {
    fclose(fp_odom);
    fclose(fp_scan_error);  
  }

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odom_nodelet...");
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // subscribers
    //no_ground_cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 256));
    //ground_cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points_ground", 256));
    //sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(*no_ground_cloud_sub, *ground_cloud_sub, 256));
    //sync->registerCallback(boost::bind(&ScanMatchingOdomNodelet::cloud_callback_sync, this, _1, _2));
    
    points_sub = nh.subscribe("/filtered_points", 256, &ScanMatchingOdomNodelet::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odom/read_until", 256);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 256);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 5);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.17);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);
    
    // load calib /ground truth file and build output file
    string res_dir = pnh.param<std::string>("res_dir", "/home/whu/data/ndt_odom_KITTI");
    string seq = pnh.param<std::string>("seq", "04");
    std::cout<<"res_dir= "<<res_dir<<std::endl;
    std::cout<<"seq= "<<seq<<std::endl;
    std::string calibdir="/media/whu/HD_CHEN_2T/02data/KITTI_odometry/dataset/sequences/"+seq+"/calib.txt";
    std::string gt_file="/home/whu/data/data_source_KITTI/devkit_old/cpp/data/poses/"+seq+".txt";
    std::string odom_file=res_dir+"/data/KITTI_"+seq+"_odom.txt";
    std::string scan_error_file=res_dir+"/errors/KITTI_"+seq+"_scan_error.txt";
    ifstream fin ( calibdir );
    string tmp;
    for(int i=0;i<4;i++)getline(fin,tmp);
    tf_velo2cam.setIdentity();
    fin>>tmp>>tf_velo2cam(0,0)>>tf_velo2cam(0,1)>>tf_velo2cam(0,2)>>tf_velo2cam(0,3)
    >>tf_velo2cam(1,0)>>tf_velo2cam(1,1)>>tf_velo2cam(1,2)>>tf_velo2cam(1,3)
    >>tf_velo2cam(2,0)>>tf_velo2cam(2,1)>>tf_velo2cam(2,2)>>tf_velo2cam(2,3);
    
    FILE *fp = fopen(gt_file.c_str(),"r");
    if (fp) {
    while (!feof(fp)) {
      Eigen::Matrix4d p=Eigen::Matrix4d::Identity();
      if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &(p(0,0)), &(p(0,1)), &(p(0,2)), &(p(0,3)),
                   &(p(1,0)), &(p(1,1)), &(p(1,2)), &(p(1,3)),
                   &(p(2,0)), &(p(2,1)), &(p(2,2)), &(p(2,3)))) {
      poses_cam.push_back(p);
      poses_velo.push_back(tf_velo2cam.inverse()*p*tf_velo2cam);
      }
    }
    fclose(fp);
    }  
  
    //registration parameters
    std::cout << "--- reg_s2s,reg_s2k=pcl::NDT_OMP ---" << std::endl;
    reg_s2s.setResolution(1.0);
    reg_s2s.setNumThreads(8);
    reg_s2s.setNeighborhoodSearchMethod(pclpca::DIRECT1);
    reg_s2s.setTransformationEpsilon(0.01);
    reg_s2s.setMaximumIterations(64);
    
    reg_s2k.setResolution(1.0);
    reg_s2k.setNumThreads(8);
    reg_s2k.setNeighborhoodSearchMethod(pclpca::DIRECT1);
    reg_s2k.setTransformationEpsilon(0.01);
    reg_s2k.setMaximumIterations(64);
  
    // ground_s2k
    ground_s2k.setResolution(10.0);
    ground_s2k.setNumThreads(8);
    ground_s2k.setNeighborhoodSearchMethod(pclomp_ground::DIRECT1);
    ground_s2k.setTransformationEpsilon(0.01);
    ground_s2k.setMaximumIterations(64);
      
    //输出 odom tf_s2k_error
    Eigen::Matrix4d odom=Eigen::Matrix4d::Identity();
    odom_velo=Eigen::Matrix4d::Identity();
    tf_s2k_error=Eigen::Matrix4d::Identity();
    fp_odom = fopen(odom_file.c_str(),"w+");
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    fp_scan_error = fopen(scan_error_file.c_str(),"w+");
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3)); 
  
    scan_count=0;
    key_id=0,key_interval=10;
    
    std::cout << "--- initialize_params end! ---" << std::endl;
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

    Eigen::Matrix4d pose = matching_s2k(cloud_msg->header.stamp, cloud);//base系在odom系下的变换（odom=第一帧keyframe的base）
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);
    scan_count++;
    
    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = "/velodyne_points";
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

  }

    /**
   * @brief callback for sync point clouds
   * @param no_ground_cloud_msg  point cloud msg
   * @param ground_cloud_msg  point cloud msg
   */
  void cloud_callback_sync(const sensor_msgs::PointCloud2ConstPtr& no_ground_cloud_msg,const sensor_msgs::PointCloud2ConstPtr& ground_cloud_msg) {
    if(!ros::ok()) {
      return;
    }

    pcl::PointCloud<PointT>::Ptr no_ground_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*no_ground_cloud_msg, *no_ground_cloud);
    pcl::fromROSMsg(*ground_cloud_msg, *ground_cloud);

    Eigen::Matrix4d pose = matching_ground(no_ground_cloud_msg->header.stamp, no_ground_cloud,ground_cloud);
    publish_odometry(no_ground_cloud_msg->header.stamp, no_ground_cloud_msg->header.frame_id, pose);
    scan_count++;
    
    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = "/velodyne_points";
    read_until->stamp = no_ground_cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

  }
    /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and last scan
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4d matching_s2k_bys2s(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    filtered=cloud;
    if(scan_count==0)
    {
      reg_s2s.setInputTarget(filtered);
      tf_s2s.setIdentity();tf_s2s(0,3)=1.5;
      
      key=filtered;
      reg_s2k.setInputTarget(key);
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose.setIdentity();
      keyframe_stamp=stamp;
      last_t = ros::WallTime::now();
      return Eigen::Matrix4d::Identity();
    }
    
    pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());
    //s2s
    reg_s2s.setInputSource(filtered); 
    reg_s2s.align(*matched, tf_s2s.cast<float>());
    tf_s2s = reg_s2s.getFinalTransformation().cast<double>();
    reg_s2s.setInputTarget(filtered); 
    
    //s2k
    std::cout << std::endl<<scan_count<<" to "<<key_id<<"  "<< std::endl;
    tf_s2k=tf_s2k*tf_s2s;
    odom_velo = key_pose * tf_s2k;
    
    Eigen::Matrix4d tf_s2k_gt=Eigen::Matrix4d::Identity();
    if(poses_cam.size()!=0)tf_s2k_gt=poses_cam[key_id].inverse()*poses_cam[scan_count];
    Eigen::Matrix4d tf_s2k_gt_velo=tf_velo2cam.inverse()*tf_s2k_gt*tf_velo2cam;
    std::cout<<"tf_s2k_gt_velo: \n"<<tf_s2k_gt_velo<<std::endl;
    std::cout<<"tf_s2k by pre_tf_s2k*tf_s2s: \n"<<tf_s2k<<std::endl;
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    
    reg_s2k.setInputSource(filtered); 
    reg_s2k.align(*matched, tf_s2k.cast<float>());
    tf_s2k = reg_s2k.getFinalTransformation().cast<double>();    
    std::cout<<"update tf_s2k per key_interval f: \n"<<tf_s2k<<std::endl;
       
    //refine    
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    odom_velo= key_pose * tf_s2k;
           
    //delta between current scan and keyframe
    double dx_s2k = tf_s2k.block<3, 1>(0, 3).norm();
    double da_s2k = 2*std::acos(Eigen::Quaternionf(tf_s2k.block<3, 3>(0, 0).cast<float>()).w());
    double dt_s2k = (stamp - keyframe_stamp).toSec();
    std::cout<<dx_s2k<<" "<<da_s2k<<" "<<dt_s2k<<std::endl;
    if(dx_s2k > keyframe_delta_trans || da_s2k > keyframe_delta_angle || dt_s2k > keyframe_delta_time) {    
    //if(scan_count%key_interval==0) {    
      key=filtered;
      reg_s2k.setInputTarget(key);   
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose=odom_velo;
      keyframe_stamp=stamp;
    }
    
    //elevation
    Sophus::SE3 SE3_Rt(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
    std::cout<<"tf_s2k_error_velo: "<<SE3_Rt.log().transpose()<<std::endl;  
    
    //output
    Eigen::Matrix4d odom=tf_velo2cam*odom_velo*tf_velo2cam.inverse();
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    
    if(poses_cam.size()!=0){
    Eigen::Matrix4d odom_error=poses_cam[scan_count].inverse()*odom;
    Sophus::SE3 SE3_Rt2(odom_error.block(0,0,3,3),odom_error.block(0,3,3,1));
    std::cout<<"odom_error_cam: "<<SE3_Rt2.log().transpose()<<std::endl;  
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3));
    }
 
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout << "--t: "<< (t2 - last_t).toSec() <<std::endl;
    last_t=t2;
    
    return odom;
  }

   /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and last scan
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4d matching_s2k(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    filtered=cloud;
    if(scan_count==0)
    {    
      key=filtered;
      reg_s2k.setInputTarget(key);
      key_id=scan_count;
      guess_trans.setIdentity();
      guess_trans(0,3)=1.5;
      pre_tf_s2k.setIdentity();
      key_pose.setIdentity();
      keyframe_stamp=stamp;
      last_t = ros::WallTime::now();
      return Eigen::Matrix4d::Identity();
    }
    
    pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());
    
    //s2k
    std::cout << std::endl<<scan_count<<" to "<<key_id<<"  "<< std::endl;
    
    Eigen::Matrix4d tf_s2k_gt=Eigen::Matrix4d::Identity();
    if(poses_cam.size()!=0)tf_s2k_gt=poses_cam[key_id].inverse()*poses_cam[scan_count];
    Eigen::Matrix4d tf_s2k_gt_velo=tf_velo2cam.inverse()*tf_s2k_gt*tf_velo2cam;
    std::cout<<"tf_s2k_gt_velo: \n"<<tf_s2k_gt_velo<<std::endl;
    std::cout<<"guess_trans: \n"<<guess_trans<<std::endl;
    
    reg_s2k.setInputSource(filtered); 
    reg_s2k.align(*matched, guess_trans.cast<float>());
    tf_s2k = reg_s2k.getFinalTransformation().cast<double>(); 
    if(scan_count==1)
    {
      reg_s2k.align(*matched, tf_s2k.cast<float>());
      tf_s2k = reg_s2k.getFinalTransformation().cast<double>(); 
    }
    std::cout<<"tf_s2k with reg_s2k: \n"<<tf_s2k<<std::endl;
    
    tf_s2s=pre_tf_s2k.inverse()*tf_s2k;
    //refine    
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    odom_velo= key_pose * tf_s2k;
           
    //delta between current scan and keyframe
    double dx_s2k = tf_s2k.block<3, 1>(0, 3).norm();
    double da_s2k = 2*std::acos(Eigen::Quaternionf(tf_s2k.block<3, 3>(0, 0).cast<float>()).w());
    double dt_s2k = (stamp - keyframe_stamp).toSec();
    std::cout<<dx_s2k<<" "<<da_s2k<<" "<<dt_s2k<<std::endl;
    if(dx_s2k > keyframe_delta_trans || da_s2k > keyframe_delta_angle || dt_s2k > keyframe_delta_time) {    
    //if(scan_count%key_interval==0) {    
      key=filtered;
      reg_s2k.setInputTarget(key);   
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose=odom_velo;
      keyframe_stamp=stamp;
    }
    pre_tf_s2k=tf_s2k;
    guess_trans=pre_tf_s2k*tf_s2s;
    
    //elevation
    Sophus::SE3 SE3_Rt(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
    std::cout<<"tf_s2k_error_velo: "<<SE3_Rt.log().transpose()<<std::endl;  
    
    //output
    Eigen::Matrix4d odom=tf_velo2cam*odom_velo*tf_velo2cam.inverse();
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    
    if(poses_cam.size()!=0){
    Eigen::Matrix4d odom_error=poses_cam[scan_count].inverse()*odom;
    Sophus::SE3 SE3_Rt2(odom_error.block(0,0,3,3),odom_error.block(0,3,3,1));
    std::cout<<"odom_error_cam: "<<SE3_Rt2.log().transpose()<<std::endl;  
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3));
    }
 
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout << "--t: "<< (t2 - last_t).toSec() <<std::endl;
    last_t=t2;
    
    return odom;
  }
  
   /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and last scan with ndt_ground optimizing
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4d matching_s2k_ground(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    filtered=cloud;
    if(scan_count==0)
    {    
      key=filtered;
      reg_s2k.setInputTarget(key);
      ground_s2k.setInputTarget(key);
      key_id=scan_count;
      guess_trans.setIdentity();
      guess_trans(0,3)=1.5;
      pre_tf_s2k.setIdentity();
      key_pose.setIdentity();
      keyframe_stamp=stamp;
      last_t = ros::WallTime::now();
      return Eigen::Matrix4d::Identity();
    }
    
    pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());
    
    //s2k
    std::cout << std::endl<<scan_count<<" to "<<key_id<<"  "<< std::endl;
    
    Eigen::Matrix4d tf_s2k_gt=Eigen::Matrix4d::Identity();
    if(poses_cam.size()!=0)tf_s2k_gt=poses_cam[key_id].inverse()*poses_cam[scan_count];
    Eigen::Matrix4d tf_s2k_gt_velo=tf_velo2cam.inverse()*tf_s2k_gt*tf_velo2cam;
    std::cout<<"tf_s2k_gt_velo: \n"<<tf_s2k_gt_velo<<std::endl;
    std::cout<<"guess_trans: \n"<<guess_trans<<std::endl;
    
    reg_s2k.setInputSource(filtered); 
    reg_s2k.align(*matched, guess_trans.cast<float>());
    tf_s2k = reg_s2k.getFinalTransformation().cast<double>(); 
    if(scan_count==1)
    {
      reg_s2k.align(*matched, tf_s2k.cast<float>());
      tf_s2k = reg_s2k.getFinalTransformation().cast<double>(); 
    }
    std::cout<<"tf_s2k with reg_s2k: \n"<<tf_s2k<<std::endl;
    
    //refine 
    if((pre_tf_s2k.inverse()*tf_s2k).block<3, 1>(0, 3).norm()>2)
    {
      ground_s2k.setInputSource(filtered); 
      ground_s2k.align(*matched, tf_s2k.cast<float>());
      tf_s2k = ground_s2k.getFinalTransformation().cast<double>();
      std::cout<<"tf_s2k with ground_s2k: \n"<<tf_s2k<<std::endl;
    }
    
    tf_s2s=pre_tf_s2k.inverse()*tf_s2k;   
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    odom_velo= key_pose * tf_s2k;
           
    //delta between current scan and keyframe
    double dx_s2k = tf_s2k.block<3, 1>(0, 3).norm();
    double da_s2k = 2*std::acos(Eigen::Quaternionf(tf_s2k.block<3, 3>(0, 0).cast<float>()).w());
    double dt_s2k = (stamp - keyframe_stamp).toSec();
    std::cout<<dx_s2k<<" "<<da_s2k<<" "<<dt_s2k<<std::endl;
    if(dx_s2k > keyframe_delta_trans || da_s2k > keyframe_delta_angle || dt_s2k > keyframe_delta_time) {    
    //if(scan_count%key_interval==0) {    
      key=filtered;
      reg_s2k.setInputTarget(key);
      ground_s2k.setInputTarget(key);
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose=odom_velo;
      keyframe_stamp=stamp;
    }
    pre_tf_s2k=tf_s2k;
    guess_trans=pre_tf_s2k*tf_s2s;
    
    //elevation
    Sophus::SE3 SE3_Rt(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
    std::cout<<"tf_s2k_error_velo: "<<SE3_Rt.log().transpose()<<std::endl;  
    
    //output
    Eigen::Matrix4d odom=tf_velo2cam*odom_velo*tf_velo2cam.inverse();
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    
    if(poses_cam.size()!=0){
    Eigen::Matrix4d odom_error=poses_cam[scan_count].inverse()*odom;
    Sophus::SE3 SE3_Rt2(odom_error.block(0,0,3,3),odom_error.block(0,3,3,1));
    std::cout<<"odom_error_cam: "<<SE3_Rt2.log().transpose()<<std::endl;  
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3));
    }
 
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout << "--t: "<< (t2 - last_t).toSec() <<std::endl;
    last_t=t2;
    
    return odom;
  }
  
      /**
   * @brief estimate the relative pose between an input cloud and the first keyframe cloud (odometry)
   *        matching between current scan and last scan
   * @param stamp  the timestamp of the input cloud
   * @param no_ground_cloud  the input cloud
   * @param ground_cloud  the input cloud
   * @return the relative pose between the input cloud and the first keyframe cloud (odometry)
   */
  Eigen::Matrix4d matching_ground(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& no_ground_cloud,
				  const pcl::PointCloud<PointT>::ConstPtr& ground_cloud) {
    filtered=no_ground_cloud;
    if(scan_count==0)
    {
      reg_s2s.setInputTarget(filtered);
      tf_s2s.setIdentity();tf_s2s(0,3)=1.5;
      
      key=filtered;
      reg_s2k.setInputTarget(key);
      //ground_s2k.setInputTarget(ground_cloud);
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose.setIdentity();
      last_t = ros::WallTime::now();
      return Eigen::Matrix4d::Identity();
    }
    
    pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());
    //s2s
    reg_s2s.setInputSource(filtered); 
    reg_s2s.align(*matched, tf_s2s.cast<float>());
    tf_s2s = reg_s2s.getFinalTransformation().cast<double>();
    reg_s2s.setInputTarget(filtered); 
    
    //s2k
    std::cout << std::endl<<scan_count<<" to "<<key_id<<"  "<< std::endl;
    tf_s2k=tf_s2k*tf_s2s;
    odom_velo = key_pose * tf_s2k;
    
    Eigen::Matrix4d tf_s2k_gt=poses_cam[key_id].inverse()*poses_cam[scan_count];
    Eigen::Matrix4d tf_s2k_gt_velo=tf_velo2cam.inverse()*tf_s2k_gt*tf_velo2cam;
    std::cout<<"tf_s2k_gt_velo: \n"<<tf_s2k_gt_velo<<std::endl;
    std::cout<<"tf_s2k by acculating s2s: \n"<<tf_s2k<<std::endl;
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    
    if(scan_count%key_interval==0) {   
      reg_s2k.setInputSource(filtered); 
      reg_s2k.align(*matched, tf_s2k.cast<float>());
      tf_s2k = reg_s2k.getFinalTransformation().cast<double>();    
      std::cout<<"update tf_s2k per key_interval f: \n"<<tf_s2k<<std::endl;
             
      //refine
      /*ground_s2k.setInputSource(ground_cloud);
      ground_s2k.align(*matched, tf_s2k.cast<float>());
      tf_s2k=ground_s2k.getFinalTransformation().cast<double>();
      std::cout<<"update tf_s2k with ground gicp: \n"<<tf_s2k<<std::endl;
      ground_s2k.setInputTarget(ground_cloud);*/
      
      tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
      odom_velo= key_pose * tf_s2k;
      
      key=filtered;
      reg_s2k.setInputTarget(key);   
      key_id=scan_count;
      tf_s2k.setIdentity();
      key_pose=odom_velo;
    }
    
    //elevation
    Sophus::SE3 SE3_Rt(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
    std::cout<<"tf_s2k_error_velo: "<<SE3_Rt.log().transpose()<<std::endl;  
    
    //output
    Eigen::Matrix4d odom=tf_velo2cam*odom_velo*tf_velo2cam.inverse();
    Eigen::Matrix4d odom_error=poses_cam[scan_count].inverse()*odom;
    Sophus::SE3 SE3_Rt2(odom_error.block(0,0,3,3),odom_error.block(0,3,3,1));
    std::cout<<"odom_error_cam: "<<SE3_Rt2.log().transpose()<<std::endl;  
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3));   
 
    //screet print 
    auto t2 = ros::WallTime::now();
    std::cout << "--t: "<< (t2 - last_t).toSec() <<std::endl;
    last_t=t2;
    
    return odom;
  }
  
  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4d& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose.cast<float>(), odom_frame_id, base_frame_id);
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
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> no_ground_cloud_sub; 
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> ground_cloud_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>> sync;
  
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
  
  // odometry calculation
  Eigen::Matrix4d prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4d keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud
  pcl::PointCloud<PointT>::ConstPtr lastframe;  // lastframe point cloud
  Eigen::Matrix4d lastframe_pose;               // lastframe pose

  //
  pcl::Registration<PointT, PointT>::Ptr registration;
  
  //odom pose file with KITTI calibration tf_cal
  pclpca::NormalDistributionsTransform<PointT, PointT> reg_s2s, reg_s2k;
  pclomp_ground::NormalDistributionsTransformGround<PointT, PointT> ground_s2k;
  FILE *fp_odom,*fp_scan_error;
  Eigen::Matrix4d tf_velo2cam;
  ros::WallTime last_t;
  Eigen::Matrix4d guess_trans;                  //init guess for mathing
  int scan_count,key_id,key_interval;
  pcl::PointCloud<PointT>::ConstPtr filtered,key;
  Eigen::Matrix4d tf_s2s,tf_s2k,key_pose,pre_tf_s2k;
  Eigen::Matrix4d odom_velo,tf_s2k_error;
  vector<Eigen::Matrix4d> poses_cam,poses_velo;
};

}

PLUGINLIB_EXPORT_CLASS(lidar_odometry::ScanMatchingOdomNodelet, nodelet::Nodelet)
