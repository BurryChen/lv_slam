#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <lv_slam/FloorCoeffs.h>

#include <lv_slam/SaveMap.h>
#include <lv_slam/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <global_graph/ros_utils.hpp>
#include <global_graph/ros_time_hash.hpp>

#include <global_graph/graph_slam.hpp>
#include <global_graph/keyframe.hpp>
#include <global_graph/keyframe_updater.hpp>
#include <global_graph/loop_detector.hpp>
#include <global_graph/information_matrix_calculator.hpp>
#include <global_graph/map_cloud_generator.hpp>
#include <global_graph/nmea_sentence_parser.hpp>

#include <image_transport/image_transport.h> //image_transport
#include <cv_bridge/cv_bridge.h>             //cv_bridge

#include <iostream>
#include <stdio.h>

namespace lv_slam
{

  class GlobalGraphNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZI PointT;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

    GlobalGraphNodelet() {}
    virtual ~GlobalGraphNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      mt_nh = getMTNodeHandle();
      private_nh = getPrivateNodeHandle();

      // init parameters
      map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
      odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
      map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
      trans_odom2map.setIdentity();

      max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

      //
      anchor_node = nullptr;
      anchor_edge = nullptr;
      floor_plane_node = nullptr;
      graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
      keyframe_updater.reset(new KeyframeUpdater(private_nh));
      loop_detector.reset(new LoopDetector(private_nh));
      map_cloud_generator.reset(new MapCloudGenerator());
      inf_calclator.reset(new InformationMatrixCalculator(private_nh));
      nmea_parser.reset(new NmeaSentenceParser());

      gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
      gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
      gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
      floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);

      imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
      enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
      enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
      imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
      imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

      points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

      // subscribers
      string odom_topic = private_nh.param<std::string>("odom_topic", "/odom");
      odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, odom_topic, 256));
      //odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/aft_mapped_to_init_high_frec", 256));
      cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 256));
      string img_topic = private_nh.param<std::string>("img_topic", "/camera/left/image_raw");
      img_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(mt_nh, img_topic, 256));
      //img_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(mt_nh, "/mynteye/left/image_color", 256));
      sync2.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(256), *odom_sub, *cloud_sub, *img_sub));
      sync2->registerCallback(boost::bind(&GlobalGraphNodelet::cloud_callback, this, _1, _2, _3));
      imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &GlobalGraphNodelet::imu_callback, this);
      floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &GlobalGraphNodelet::floor_coeffs_callback, this);

      if (private_nh.param<bool>("enable_gps", true))
      {
        gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &GlobalGraphNodelet::gps_callback, this);
        nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &GlobalGraphNodelet::nmea_callback, this);
        navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &GlobalGraphNodelet::navsat_callback, this);
      }

      // publishers
      markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/global_graph/markers", 16);
      odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/global_graph/odom2pub", 16);
      map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/global_graph/map_points", 1);
      read_until_pub = mt_nh.advertise<std_msgs::Header>("/global_graph/read_until", 32);

      dump_service_server = mt_nh.advertiseService("/global_graph/dump", &GlobalGraphNodelet::dump_service, this);
      save_map_service_server = mt_nh.advertiseService("/global_graph/save_map", &GlobalGraphNodelet::save_map_service, this);

      // 定时器，估计时间间隔定时闭环优化
      double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
      double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
      optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &GlobalGraphNodelet::optimization_timer_callback, this);
      //map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &GlobalGraphNodelet::map_points_publish_timer_callback, this);

      std::cout << "### global_graph init done! " << std::endl;
    }

  private:
    /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
    void cloud_callback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const sensor_msgs::Image::ConstPtr &img_msg)
    {
      const ros::Time &stamp = odom_msg->header.stamp;
      int seq = odom_msg->header.seq;
      //std::cout<<seq<<" "<<stamp<<std::endl;
      Eigen::Isometry3d odom = odom2isometry(odom_msg); //base系在odom系下的变换（map=第一帧keyframe的base,odom2map闭环检测修正值，realtime uodate）

      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*cloud_msg, *cloud);
      if (base_frame_id.empty())
      {
        base_frame_id = cloud_msg->header.frame_id;
      }

      odoms.insert(std::pair<int, Eigen::Isometry3d>(seq, odom));
      /*//和pre keyframe 平移旋转量太小，跳过，不选为keyframe
    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    //第一关键帧起的累计平移标量
    double accum_d = keyframe_updater->get_accum_distance();
    //KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));
    
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = cv_ptr->image.clone();
    std::cout<<"seq added to keyframs:"<<seq<<std::endl;
    Ptr<cv::Feature2D> detector=cv::ORB::create();
    std::vector<KeyPoint> keypoints;
    cv::Mat descriptor;
    detector->detectAndCompute(img, Mat(),keypoints,descriptor);
    KeyFrame::Ptr keyframe(new KeyFrame(stamp,seq,odom, accum_d, cloud,descriptor));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);   //关键帧队列*/

      // window map 作为keyframe
      if (keyframe_updater->is_first && keyframe_updater->update(odom, stamp))
      {
        w_odom = odom;
        w_cloud.clear();
        w_cloud = *cloud;
        w_img = *img_msg;
        w_seq = seq;
        accum_d = keyframe_updater->get_accum_distance();
        //std::cout<<seq<<" start "<<cloud->size()<<" "<<w_cloud.size()<<"-----------------------------------------------------------------------"<<std::endl;
      }
      else if (!keyframe_updater->is_first && keyframe_updater->update(odom, stamp))
      {
        pcl::VoxelGrid<PointT> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
        voxelgrid.setInputCloud(w_cloud.makeShared());
        voxelgrid.filter(*downsampled);
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(w_img, sensor_msgs::image_encodings::MONO8);
        cv::Mat img = cv_ptr->image.clone();
        std::cout << "seq added to keyframs:" << w_seq << std::endl;
        Ptr<cv::Feature2D> detector = cv::ORB::create();
        std::vector<KeyPoint> keypoints;
        cv::Mat descriptor;
        detector->detectAndCompute(img, Mat(), keypoints, descriptor);
        std::cout << w_seq << " " << w_cloud.size() << " " << downsampled->size() << "-----------------------------------------------------------------------" << std::endl;
        KeyFrame::Ptr keyframe(new KeyFrame(w_img.header.stamp, w_seq, w_odom, accum_d, downsampled, descriptor));
        std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
        keyframe_queue.push_back(keyframe);
        w_odom = odom;
        w_cloud.clear();
        w_cloud = *cloud;
        w_img = *img_msg;
        w_seq = seq;
        accum_d = keyframe_updater->get_accum_distance();
      }
      else if (!keyframe_updater->is_first && !keyframe_updater->update(odom, stamp))
      {
        std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud, *transformed, (w_odom.inverse() * odom).matrix());
        w_cloud += *transformed;
        //std::cout<<seq<<" "<<transformed->size()<<" "<<w_cloud.size()<<"-----------------------------------------------------------------------"<<std::endl;
      }
    }

    /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
    bool flush_keyframe_queue()
    {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

      if (keyframe_queue.empty())
      {
        return false;
      }

      trans_odom2map_mutex.lock();
      Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
      trans_odom2map_mutex.unlock();

      int num_processed = 0;
      for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++)
      {
        num_processed = i;

        const auto &keyframe = keyframe_queue[i];
        // new_keyframes will be tested later for loop closure
        new_keyframes.push_back(keyframe);

        // add pose node
        Eigen::Isometry3d odom = odom2map * keyframe->odom;
        keyframe->node = graph_slam->add_se3_node(odom); //map系下pose
        keyframe_hash[keyframe->stamp] = keyframe;

        // fix the first node
        if (keyframes.empty() && new_keyframes.size() == 1)
        {
          if (private_nh.param<bool>("fix_first_node", false))
          {
            anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
            anchor_node->setFixed(true);
            anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 6));
          }
        }

        if (i == 0 && keyframes.empty())
        {
          continue;
        }

        // add edge between consecutive keyframes
        const auto &prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
        Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
        auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
      }

      std_msgs::Header read_until;
      read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);

      keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
      return true;
    }

    void nmea_callback(const nmea_msgs::SentenceConstPtr &nmea_msg)
    {
      GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

      if (grmc.status != 'A')
      {
        return;
      }

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = nmea_msg->header;
      gps_msg->position.latitude = grmc.latitude;
      gps_msg->position.longitude = grmc.longitude;
      gps_msg->position.altitude = NAN;

      gps_callback(gps_msg);
    }

    void navsat_callback(const sensor_msgs::NavSatFixConstPtr &navsat_msg)
    {
      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = navsat_msg->header;
      gps_msg->position.latitude = navsat_msg->latitude;
      gps_msg->position.longitude = navsat_msg->longitude;
      gps_msg->position.altitude = navsat_msg->altitude;
      gps_callback(gps_msg);
    }

    /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
    void gps_callback(const geographic_msgs::GeoPointStampedPtr &gps_msg)
    {
      std::lock_guard<std::mutex> lock(gps_queue_mutex);
      gps_msg->header.stamp += ros::Duration(gps_time_offset);
      gps_queue.push_back(gps_msg);
    }

    /**
   * @brief
   * @return
   */
    bool flush_gps_queue()
    {
      std::lock_guard<std::mutex> lock(gps_queue_mutex);

      if (keyframes.empty() || gps_queue.empty())
      {
        return false;
      }

      bool updated = false;
      auto gps_cursor = gps_queue.begin();

      for (auto &keyframe : keyframes)
      {
        if (keyframe->stamp > gps_queue.back()->header.stamp)
        {
          break;
        }

        if (keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord)
        {
          continue;
        }

        // find the gps data which is closest to the keyframe
        auto closest_gps = gps_cursor;
        for (auto gps = gps_cursor; gps != gps_queue.end(); gps++)
        {
          auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
          auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
          if (std::abs(dt) < std::abs(dt2))
          {
            break;
          }

          closest_gps = gps;
        }

        // if the time residual between the gps and keyframe is too large, skip it
        gps_cursor = closest_gps;
        if (0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec()))
        {
          continue;
        }

        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg((*closest_gps)->position, utm);
        Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

        // the first gps data position will be the origin of the map
        if (!zero_utm)
        {
          zero_utm = xyz;
        }
        xyz -= (*zero_utm);

        keyframe->utm_coord = xyz;

        g2o::OptimizableGraph::Edge *edge;
        if (std::isnan(xyz.z()))
        {
          Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
          edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
        }
        else
        {
          Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
          information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
          information_matrix(2, 2) /= gps_edge_stddev_z;
          edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
        }
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));

        updated = true;
      }

      auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp,
                                         [=](const ros::Time &stamp, const geographic_msgs::GeoPointStampedConstPtr &geopoint)
                                         {
                                           return stamp < geopoint->header.stamp;
                                         });
      gps_queue.erase(gps_queue.begin(), remove_loc);
      return updated;
    }

    void imu_callback(const sensor_msgs::ImuPtr &imu_msg)
    {
      if (!enable_imu_orientation && !enable_imu_acceleration)
      {
        return;
      }

      std::lock_guard<std::mutex> lock(imu_queue_mutex);
      imu_msg->header.stamp += ros::Duration(imu_time_offset);
      imu_queue.push_back(imu_msg);
    }

    bool flush_imu_queue()
    {
      std::lock_guard<std::mutex> lock(imu_queue_mutex);
      if (keyframes.empty() || imu_queue.empty() || base_frame_id.empty())
      {
        return false;
      }

      bool updated = false;
      auto imu_cursor = imu_queue.begin();

      for (auto &keyframe : keyframes)
      {
        if (keyframe->stamp > imu_queue.back()->header.stamp)
        {
          break;
        }

        if (keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration)
        {
          continue;
        }

        // find imu data which is closest to the keyframe
        auto closest_imu = imu_cursor;
        for (auto imu = imu_cursor; imu != imu_queue.end(); imu++)
        {
          auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
          auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
          if (std::abs(dt) < std::abs(dt2))
          {
            break;
          }

          closest_imu = imu;
        }

        imu_cursor = closest_imu;
        if (0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec()))
        {
          continue;
        }

        const auto &imu_ori = (*closest_imu)->orientation;
        const auto &imu_acc = (*closest_imu)->linear_acceleration;

        geometry_msgs::Vector3Stamped acc_imu;
        geometry_msgs::Vector3Stamped acc_base;
        geometry_msgs::QuaternionStamped quat_imu;
        geometry_msgs::QuaternionStamped quat_base;

        quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
        quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
        acc_imu.vector = (*closest_imu)->linear_acceleration;
        quat_imu.quaternion = (*closest_imu)->orientation;

        try
        {
          tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
          if (enable_imu_orientation)
            tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
        }
        catch (std::exception &e)
        {
          std::cerr << "imu failed to find transform!!" << base_frame_id << acc_imu.header.frame_id << std::endl;
          return false;
        }

        keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
        keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
        keyframe->orientation = keyframe->orientation;
        if (keyframe->orientation->w() < 0.0)
        {
          keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
        }

        if (enable_imu_orientation)
        {
          Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
          auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
          graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
        }

        if (enable_imu_acceleration)
        {
          Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
          g2o::OptimizableGraph::Edge *edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
          graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
        }
        updated = true;
      }

      auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp,
                                         [=](const ros::Time &stamp, const sensor_msgs::ImuConstPtr &imu)
                                         {
                                           return stamp < imu->header.stamp;
                                         });
      imu_queue.erase(imu_queue.begin(), remove_loc);

      return true;
    }

    /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
    void floor_coeffs_callback(const lv_slam::FloorCoeffsConstPtr &floor_coeffs_msg)
    {
      if (floor_coeffs_msg->coeffs.empty())
      {
        return;
      }

      std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
      floor_coeffs_queue.push_back(floor_coeffs_msg);
    }

    /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
    bool flush_floor_queue()
    {
      std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

      if (keyframes.empty())
      {
        return false;
      }

      const auto &latest_keyframe_stamp = keyframes.back()->stamp;

      bool updated = false;
      for (const auto &floor_coeffs : floor_coeffs_queue)
      {
        if (floor_coeffs->header.stamp > latest_keyframe_stamp)
        {
          break;
        }

        auto found = keyframe_hash.find(floor_coeffs->header.stamp);
        if (found == keyframe_hash.end())
        {
          continue;
        }

        if (!floor_plane_node)
        {
          floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
          floor_plane_node->setFixed(true);
        }

        const auto &keyframe = found->second;

        Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
        auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

        keyframe->floor_coeffs = coeffs;

        updated = true;
      }

      auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
                                         [=](const ros::Time &stamp, const lv_slam::FloorCoeffsConstPtr &coeffs)
                                         {
                                           return stamp < coeffs->header.stamp;
                                         });
      floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

      return updated;
    }

    /**
   * @brief generate map point cloud and publish it
   * @param event
   */
    void map_points_publish_timer_callback(const ros::WallTimerEvent &event)
    {
      if (!map_points_pub.getNumSubscribers())
      {
        return;
      }

      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      snapshot = keyframes_snapshot;
      keyframes_snapshot_mutex.unlock();

      auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
      if (!cloud)
      {
        return;
      }

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud, *cloud_msg);

      std::vector<KeyFrameSnapshot::Ptr>().swap(snapshot);
      cout << "snapshot.capacity()=" << snapshot.capacity() << endl;
      cloud->clear();
      cout << "cloud.capacity()=" << cloud->size() << endl;

      map_points_pub.publish(cloud_msg);
    }

    /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
    void optimization_timer_callback(const ros::WallTimerEvent &event)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      // add keyframes and floor coeffs in the queues to the pose graph
      bool keyframe_updated = flush_keyframe_queue();

      if (!keyframe_updated)
      {
        std_msgs::Header read_until;
        read_until.stamp = ros::Time::now() + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      if (!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue())
      {
        return;
      }

      // loop detection
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
      for (const auto &loop : loops)
      {
        Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
        auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
      }

      //local 关键帧加入到global关键帧中
      std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
      new_keyframes.clear();

      // optimize the pose graph
      int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
      graph_slam->optimize(num_iterations);

      //BA is for all keyframes, 把误差均分给每一个node,为了保证相对关系会造成绝对pose的偏差。以first keyframe为基准，消除绝对偏差。
      auto trans_absolute = (keyframes.front()->node->estimate()).inverse();
      for (auto keyframe : keyframes)
      {
        keyframe->node->setEstimate(trans_absolute * keyframe->node->estimate());
      }

      // publish tf,tf_odom2map
      const auto &keyframe = keyframes.back();
      Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse(); //tf_base2map*tf_base2odom.inverse()=tf_odom2map
      trans_odom2map_mutex.lock();
      trans_odom2map = trans.matrix().cast<float>(); //两种不同类型的Eigen矩阵相加，或者赋值，需要用到cast函数：
      trans_odom2map_mutex.unlock();

      //if(map_points_pub.getNumSubscribers()) {//如果有结果mapping订阅就发布
      if (1)
      { //无结果mapping订阅也发布
        std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
        std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
                       [=](const KeyFrame::Ptr &k)
                       {
                         return std::make_shared<KeyFrameSnapshot>(k);
                       });

        keyframes_snapshot_mutex.lock();
        keyframes_snapshot.swap(snapshot);
        keyframes_snapshot_mutex.unlock();

        auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
        if (!cloud)
        {
          return;
        }

        cloud->header.frame_id = map_frame_id;
        cloud->header.stamp = snapshot.back()->cloud->header.stamp;
        cout << "map_points.size=" << cloud->size() << endl;

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud, *cloud_msg);
        map_points_pub.publish(cloud_msg);
      }

      if (odom2map_pub.getNumSubscribers())
      { //pub tf_odom2map
        geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
        odom2map_pub.publish(ts);
      }

      if (markers_pub.getNumSubscribers())
      {
        auto markers = create_marker_array(ros::Time::now());
        markers_pub.publish(markers);
      }
    }

    /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
    visualization_msgs::MarkerArray create_marker_array(const ros::Time &stamp) const
    {
      visualization_msgs::MarkerArray markers;
      markers.markers.resize(5);

      // node markers
      visualization_msgs::Marker &traj_marker = markers.markers[0];
      traj_marker.header.frame_id = "map";
      traj_marker.header.stamp = stamp;
      traj_marker.ns = "nodes";
      traj_marker.id = 0;
      traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

      traj_marker.pose.orientation.w = 1.0;
      traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 1.0;

      visualization_msgs::Marker &imu_marker = markers.markers[4];
      imu_marker.header = traj_marker.header;
      imu_marker.ns = "imu";
      imu_marker.id = 4;
      imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

      imu_marker.pose.orientation.w = 1.0;
      imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

      traj_marker.points.resize(keyframes.size());
      traj_marker.colors.resize(keyframes.size());
      for (int i = 0; i < keyframes.size(); i++)
      {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        traj_marker.points[i].x = pos.x();
        traj_marker.points[i].y = pos.y();
        traj_marker.points[i].z = pos.z();

        double p = static_cast<double>(i) / keyframes.size(); //从起始由红变绿
        traj_marker.colors[i].r = 0.0;
        traj_marker.colors[i].g = p;
        traj_marker.colors[i].b = 1.0 - p;
        traj_marker.colors[i].a = 1.0;

        if (keyframes[i]->acceleration)
        {
          Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
          geometry_msgs::Point point;
          point.x = pos.x();
          point.y = pos.y();
          point.z = pos.z();

          std_msgs::ColorRGBA color;
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;

          imu_marker.points.push_back(point);
          imu_marker.colors.push_back(color);
        }
      }

      // edge markers
      visualization_msgs::Marker &edge_marker = markers.markers[1];
      edge_marker.header.frame_id = "map";
      edge_marker.header.stamp = stamp;
      edge_marker.ns = "edges";
      edge_marker.id = 1;
      edge_marker.type = visualization_msgs::Marker::LINE_LIST;

      edge_marker.pose.orientation.w = 1.0;
      edge_marker.scale.x = 0.05;

      edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
      edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

      auto edge_itr = graph_slam->graph->edges().begin();
      for (int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++)
      {
        g2o::HyperGraph::Edge *edge = *edge_itr;
        g2o::EdgeSE3 *edge_se3 = dynamic_cast<g2o::EdgeSE3 *>(edge);
        if (edge_se3)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[0]);
          g2o::VertexSE3 *v2 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[1]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = v2->estimate().translation();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z();
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
          double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
          edge_marker.colors[i * 2].b = 1.0 - p1;
          edge_marker.colors[i * 2].g = p1;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].b = 1.0 - p2;
          edge_marker.colors[i * 2 + 1].g = p2;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          if (std::abs(v1->id() - v2->id()) > 2)
          {
            edge_marker.points[i * 2].z += 0.5;
            edge_marker.points[i * 2 + 1].z += 0.5;
          }

          continue;
        }

        g2o::EdgeSE3Plane *edge_plane = dynamic_cast<g2o::EdgeSE3Plane *>(edge);
        if (edge_plane)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_plane->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z();
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          edge_marker.colors[i * 2].b = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].b = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }

        g2o::EdgeSE3PriorXY *edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY *>(edge);
        if (edge_priori_xy)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xy->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
          pt2.head<2>() = edge_priori_xy->measurement();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z() + 0.5;
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

          edge_marker.colors[i * 2].r = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].r = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }

        g2o::EdgeSE3PriorXYZ *edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ *>(edge);
        if (edge_priori_xyz)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xyz->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z() + 0.5;
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          edge_marker.colors[i * 2].r = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].r = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }
      }

      // sphere
      visualization_msgs::Marker &sphere_marker = markers.markers[3];
      sphere_marker.header.frame_id = "map";
      sphere_marker.header.stamp = stamp;
      sphere_marker.ns = "loop_close_radius";
      sphere_marker.id = 0;
      sphere_marker.type = visualization_msgs::Marker::SPHERE;

      if (!keyframes.empty())
      {
        Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();
      }
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

      sphere_marker.color.r = 1.0;
      sphere_marker.color.a = 0.3;

      return markers;
    }

    /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
    bool dump_service(lv_slam::DumpGraphRequest &req, lv_slam::DumpGraphResponse &res)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      std::string directory = req.destination;

      if (directory.empty())
      {
        std::array<char, 64> buffer;
        buffer.fill(0);
        time_t rawtime;
        time(&rawtime);
        const auto timeinfo = localtime(&rawtime);
        strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
        std::string directory(buffer.data());
      }

      if (!boost::filesystem::is_directory(directory))
      {
        boost::filesystem::create_directory(directory);
      }

      std::cout << "all data dumped to:" << directory << std::endl;

      graph_slam->save(directory + "/graph.g2o");
      for (int i = 0; i < keyframes.size(); i++)
      {
        std::stringstream sst;
        sst << boost::format("%s/%06d") % directory % i;

        keyframes[i]->save(sst.str());
      }

      if (zero_utm)
      {
        std::ofstream zero_utm_ofs(directory + "/zero_utm");
        zero_utm_ofs << *zero_utm << std::endl;
      }

      std::ofstream ofs(directory + "/special_nodes.csv");
      ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
      ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
      ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

      save_pose(directory);

      res.success = true;
      return true;
    }

    /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
    bool save_map_service(lv_slam::SaveMapRequest &req, lv_slam::SaveMapResponse &res)
    {
      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      snapshot = keyframes_snapshot;
      keyframes_snapshot_mutex.unlock();

      auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
      if (!cloud)
      {
        res.success = false;
        return true;
      }

      if (zero_utm && req.utm)
      {
        for (auto &pt : cloud->points)
        {
          pt.getVector3fMap() += (*zero_utm).cast<float>();
        }
      }

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      if (zero_utm)
      {
        std::ofstream ofs(req.destination + ".utm");
        ofs << (*zero_utm).transpose() << std::endl;
      }
      int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
      res.success = ret == 0;

      return true;
    }

    /**
   * @brief save pose
   * @param directory
   * @return
   */
    void save_pose(std::string directory)
    {
      // load calib  file
      string calib_file = private_nh.param<std::string>("calib_file", " ");
      std::cout << "calib_file= " << calib_file << std::endl;
      ifstream fin(calib_file);
      string tmp;
      for (int i = 0; i < 4; i++)
        getline(fin, tmp);
      tf_velo2cam.setIdentity();
      fin >> tmp >> tf_velo2cam(0, 0) >> tf_velo2cam(0, 1) >> tf_velo2cam(0, 2) >> tf_velo2cam(0, 3) >> tf_velo2cam(1, 0) >> tf_velo2cam(1, 1) >> tf_velo2cam(1, 2) >> tf_velo2cam(1, 3) >> tf_velo2cam(2, 0) >> tf_velo2cam(2, 1) >> tf_velo2cam(2, 2) >> tf_velo2cam(2, 3);

      std::ofstream pose_keyframe_ofs(directory + "/ggo_kf_odom.txt");
      for (int i = 0; i < keyframes.size(); i++)
      {
        Eigen::Isometry3d pose = keyframes[i]->node->estimate();
        // pose:the pose of left camera coordinate system in the i'th frame with respect to the first(=0th) frame
        Eigen::Matrix4d data = (tf_velo2cam * pose * tf_velo2cam.inverse()).matrix();
        //std::cout<<"pose=\n"<<pose.matrix()<<std::endl;
        pose_keyframe_ofs << boost::format("%.9u %le %le %le %le %le %le %le %le %le %le %le %le\n") % keyframes[i]->stamp % data(0, 0) % data(0, 1) % data(0, 2) % data(0, 3) % data(1, 0) % data(1, 1) % data(1, 2) % data(1, 3) % data(2, 0) % data(2, 1) % data(2, 2) % data(2, 3);
      }

      std::ofstream pose_global_ofs(directory + "/ggo_wf_odom.txt");
      Eigen::Isometry3d pose_align = keyframes[0]->node->estimate().inverse();
      for (int i = 0; i < keyframes.size(); i++)
      {
        int seq0 = keyframes[i]->seq;
        int seq1 = odoms.size();
        Eigen::Isometry3d kf_pose = pose_align * keyframes[i]->node->estimate();
        Eigen::Isometry3d odom0 = odoms[seq0];
        Eigen::Isometry3d d_pose_odom = Eigen::Isometry3d::Identity();

        if (i < keyframes.size() - 1)
        {
          Eigen::Isometry3d kf_pose_next = pose_align * keyframes[i + 1]->node->estimate();
          Eigen::Isometry3d d_pose = kf_pose.inverse() * kf_pose_next;
          seq1 = keyframes[i + 1]->seq;

          Eigen::Isometry3d odom1 = odoms[seq1];
          Eigen::Isometry3d d_odom = odom0.inverse() * odom1;

          d_pose_odom = d_odom.inverse() * d_pose;
          Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
          Eigen::Quaterniond q1 = Eigen::Quaterniond(d_pose_odom.linear().matrix());
          Eigen::Quaterniond q_slerp = q0.slerp(seq1 - seq0, q1);
          //std::cout<<"d_pose_odom "<<q_slerp.coeffs().transpose()<<std::endl<<d_pose_odom.matrix()<<std::endl;
          d_pose_odom.linear() = q_slerp.toRotationMatrix();
          d_pose_odom.translation() *= 1.0 / (seq1 - seq0);
        }
        //d_pose_odom=Eigen::Isometry3d::Identity();
        //std::cout<<"key/scan:"<<seq0<<"/"<<seq1<<std::endl;
        //std::cout<<"d_pose_odom after interpolation "<<std::endl<<d_pose_odom.matrix()<<std::endl;
        for (int j = seq0; j < seq1; j++)
        {
          Eigen::Isometry3d pose_new;
          Eigen::Isometry3d pose_s2k = odom0.inverse() * odoms[j];
          if (j == seq0)
            pose_new = kf_pose * pose_s2k;
          else
            pose_new = kf_pose * pose_s2k * d_pose_odom;
          //std::cout<<"pose_new "<<std::endl<<pose_new.matrix()<<std::endl;
          Eigen::Matrix4d data = (tf_velo2cam * pose_new * tf_velo2cam.inverse()).matrix();
          pose_global_ofs << boost::format("%le %le %le %le %le %le %le %le %le %le %le %le\n") % data(0, 0) % data(0, 1) % data(0, 2) % data(0, 3) % data(1, 0) % data(1, 1) % data(1, 2) % data(1, 3) % data(2, 0) % data(2, 1) % data(2, 2) % data(2, 3);
        }
      }
    }

  private:
    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;
    ros::WallTimer optimization_timer;
    ros::WallTimer map_publish_timer;

    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub; //订阅者过滤器
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2, sensor_msgs::Image>> sync; //时间同步器
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync2;

    ros::Subscriber gps_sub;
    ros::Subscriber nmea_sub;
    ros::Subscriber navsat_sub;

    ros::Subscriber imu_sub;
    ros::Subscriber floor_sub;

    ros::Publisher markers_pub;

    std::string map_frame_id;
    std::string odom_frame_id;

    std::mutex trans_odom2map_mutex; //h互斥锁
    Eigen::Matrix4f trans_odom2map;
    ros::Publisher odom2map_pub;

    std::string points_topic;
    ros::Publisher read_until_pub;
    ros::Publisher map_points_pub;

    tf::TransformListener tf_listener;

    ros::ServiceServer dump_service_server;
    ros::ServiceServer save_map_service_server;

    // keyframe queue
    std::string base_frame_id;
    std::mutex keyframe_queue_mutex;
    std::deque<KeyFrame::Ptr> keyframe_queue;

    // gps queue
    double gps_time_offset;
    double gps_edge_stddev_xy;
    double gps_edge_stddev_z;
    boost::optional<Eigen::Vector3d> zero_utm;
    std::mutex gps_queue_mutex;
    std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

    // imu queue
    double imu_time_offset;
    bool enable_imu_orientation;
    double imu_orientation_edge_stddev;
    bool enable_imu_acceleration;
    double imu_acceleration_edge_stddev;
    std::mutex imu_queue_mutex;
    std::deque<sensor_msgs::ImuConstPtr> imu_queue;

    // floor_coeffs queue
    double floor_edge_stddev;
    std::mutex floor_coeffs_queue_mutex;
    std::deque<lv_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

    // for map cloud generation
    double map_cloud_resolution;
    std::mutex keyframes_snapshot_mutex;
    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;

    // graph slam
    // all the below members must be accessed after locking main_thread_mutex
    std::mutex main_thread_mutex;

    int max_keyframes_per_update;
    std::deque<KeyFrame::Ptr> new_keyframes; //双端队列

    g2o::VertexSE3 *anchor_node;
    g2o::EdgeSE3 *anchor_edge;
    g2o::VertexPlane *floor_plane_node;
    std::vector<KeyFrame::Ptr> keyframes;
    std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash; //无序图

    std::unique_ptr<GraphSLAM> graph_slam; //智能指针
    std::unique_ptr<LoopDetector> loop_detector;
    std::unique_ptr<KeyframeUpdater> keyframe_updater;
    std::unique_ptr<NmeaSentenceParser> nmea_parser;

    std::unique_ptr<InformationMatrixCalculator> inf_calclator;

    //pose file with KITTI calibration tf_cal
    FILE *fp;
    Eigen::Isometry3d tf_velo2cam;
    std::map<int, Eigen::Isometry3d> odoms;
    pcl::PointCloud<PointT> w_cloud;
    Eigen::Isometry3d w_odom;
    double accum_d;
    sensor_msgs::Image w_img;
    int w_seq;
  };

}

PLUGINLIB_EXPORT_CLASS(lv_slam::GlobalGraphNodelet, nodelet::Nodelet)
