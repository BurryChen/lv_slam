#include <memory>
#include <iostream>
#include <string>
#include <mutex>

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

#include <global_graph/graph_slam.hpp>
#include <global_graph/keyframe.hpp>
#include <global_graph/keyframe_updater.hpp>
#include <global_graph/information_matrix_calculator.hpp>

using namespace std;

namespace lv_slam {

class LocalMappingNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalMappingNodelet() {}
  virtual  ~LocalMappingNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odom_nodelet...");
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    frame_updater.reset(new KeyframeUpdater(private_nh));
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    
    initialize_params();

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 256));
    sync.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 256));
    sync->registerCallback(boost::bind(&LocalMappingNodelet::cloud_callback, this, _1, _2));
    
    //points_sub = nh.subscribe("/filtered_points", 256, &LocalMappingNodelet::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odom/read_until", 256);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_after_local", 256);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    
    // load calib /ground truth file and build output file
    string res_dir = pnh.param<std::string>("res_dir", "/home/whu/data/ndt_odom_KITTI");
    string seq = pnh.param<std::string>("seq", "04");
    std::cout<<"res_dir= "<<res_dir<<std::endl;
    std::cout<<"seq= "<<seq<<std::endl;
    std::string calibdir="/media/whu/HD_CHEN_2T/02data/KITTI_odometry/dataset/sequences/"+seq+"/calib.txt";
    std::string gt_file="/home/whu/data/data_source_KITTI/devkit_old/cpp/data/poses/"+seq+".txt";
    std::string odom_file=res_dir+"/data/KITTI_"+seq+"_odom_after_local.txt";
    ifstream fin ( calibdir );
    string tmp;
    for(int i=0;i<4;i++)getline(fin,tmp);
    tf_velo2cam.setIdentity();
    fin>>tmp>>tf_velo2cam(0,0)>>tf_velo2cam(0,1)>>tf_velo2cam(0,2)>>tf_velo2cam(0,3)
    >>tf_velo2cam(1,0)>>tf_velo2cam(1,1)>>tf_velo2cam(1,2)>>tf_velo2cam(1,3)
    >>tf_velo2cam(2,0)>>tf_velo2cam(2,1)>>tf_velo2cam(2,2)>>tf_velo2cam(2,3); 
      
    //输出 odom tf_
    Eigen::Matrix4d odom=Eigen::Matrix4d::Identity();
    odom_velo=Eigen::Matrix4d::Identity();
    tf_s2k_error=Eigen::Matrix4d::Identity();
    fp_odom = fopen(odom_file.c_str(),"w+");
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    
    std::cout << "--- local_mapping initialize_params end! ---" << std::endl;
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const ros::Time& stamp = odom_msg->header.stamp;
    int seq=odom_msg->header.seq;
    //std::cout<<seq<<" in local_mapping"<<std::endl;
    Eigen::Isometry3d odom = lidar_odometry::odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    frame_updater->update(odom);
    double accum_d = frame_updater->get_accum_distance();
    // 利用了Keyfram的结构,这里是指所有帧的集合
    KeyFrame::Ptr frame(new KeyFrame(stamp, odom, accum_d, cloud));
    //std::lock_guard<std::mutex> lock(frame_queue_mutex);
    frame_queue.push_back(frame);

    //和pre keyframe 平移旋转量太小，跳过，不选为keyframe
    if(!frame_updater->update(odom)) {
      return;
    }

    local_graph_optimization();
    std::cout<<"-----------finished local_graph_optimization from "<<seq+1-frame_queue.size()<<" to "<<seq<<"-------------"<<std::endl;     
    //publish and output
    for(int i=1; i<frame_queue.size(); i++) { 
       const auto& frame=frame_queue[i];
       Eigen::Matrix4d odom_after_local=frame->odom_after_local.matrix();
       publish_odometry(frame->stamp, "velodyne2", odom_after_local); 
       //output
       Eigen::Matrix4d odom=tf_velo2cam*odom_after_local*tf_velo2cam.inverse();
       fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
       
    }
    frame_queue.erase(frame_queue.begin(), frame_queue.end()-1);
      
    /*Eigen::Matrix4d pose = local_graph(cloud_msg->header.stamp, cloud);//base系在odom系下的变换（odom=第一帧keyframe的base）
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);
    scan_count++;
    
    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = "/velodyne_points";
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);*/

  }

    /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool local_graph_optimization() {
    if(frame_queue.empty()) {
      return false;
    }
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
      
    //add node to local graph
    for(int i=0; i<frame_queue.size(); i++) {
      const auto& cur_frame = frame_queue[i];
      cur_frame->node=graph_slam->add_se3_node(cur_frame->odom);
      if(i==0) cur_frame->node->setFixed(true);
    }
    //add edge to local graph
    for(int i=0; i<frame_queue.size(); i++) {  
      const auto& cur_frame = frame_queue[i];
      int prev_seq;
      if(i==0) 
      {
	prev_seq=frame_queue.size()- 1;	
      }
      else 
      {
	prev_seq=i - 1;	
      }
      const auto& prev_frame =  frame_queue[prev_seq];     
      Eigen::Isometry3d relative_pose = cur_frame->odom.inverse() * prev_frame->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_frame->cloud, cur_frame->cloud, relative_pose);
      //std::cout<<i<<" "<<prev_seq<<std::endl<<relative_pose.matrix()<<std::endl<<information<<std::endl;
      auto edge = graph_slam->add_se3_edge(cur_frame->node, prev_frame->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
    }
    
    // optimize the pose graph
    int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);
    for(int i=1; i<frame_queue.size(); i++) {  
      frame_queue[i]->odom_after_local=frame_queue.front()->odom_after_local*(frame_queue.front()->node->estimate()).inverse()*frame_queue[i]->node->estimate();
    }
 
    return true;
  }
  
  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4d& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = lidar_odometry::matrix2transform(stamp, pose.cast<float>(), odom_frame_id, base_frame_id);
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
  
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub; 
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync;
 
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
  FILE *fp_odom;
  Eigen::Matrix4d tf_velo2cam;
  ros::WallTime last_t;
  Eigen::Matrix4d guess_trans;                  //init guess for mathing
  int scan_count,key_id,key_interval;
  pcl::PointCloud<PointT>::ConstPtr filtered,key;
  Eigen::Matrix4d tf_s2s,tf_s2k,key_pose,pre_tf_s2k;
  Eigen::Matrix4d odom_velo,tf_s2k_error;
  vector<Eigen::Matrix4d> poses_cam,poses_velo;
  
  pcl::PointCloud<PointT> localmap;            // localmap point cloud
  std::unique_ptr<KeyframeUpdater> frame_updater;
  std::mutex frame_queue_mutex;
  std::deque<KeyFrame::Ptr> frame_queue;       //vector会使内存线性增长
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<GraphSLAM> graph_slam;
  
};

}

PLUGINLIB_EXPORT_CLASS(lv_slam::LocalMappingNodelet, nodelet::Nodelet)