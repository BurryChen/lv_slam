#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/gicp_omp.h>
#include <sophus/se3.h>

/**************************************************************************
* match home/whu/data/loam_KITTI/velobag/1000000013016889.pcd   /home/whu/data/loam_KITTI/velobag/1000000013121139.pcd
****************************************************************************************************************/

pcl::PointCloud<pcl::PointXYZI>::Ptr distance_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,int distance_near_thresh, int distance_far_thresh) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const pcl::PointXYZI& p) {
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
  
int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: match target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }
  
  Eigen::Matrix4d tf_velo2cam;
  tf_velo2cam<<      
     4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,-1.198459927713e-02,
    -7.210626507497e-03,  8.081198471645e-03, -9.999413164504e-01,-5.403984729748e-02, 
     9.999738645903e-01,  4.859485810390e-04, -7.206933692422e-03,-2.921968648686e-01,
      0,0,0,1;
      
  // downsampling
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  *source_cloud = *downsampled;
  
  std::cout<<"target size:"<<target_cloud->size()<<" source size:"<<source_cloud->size()<<std::endl;

  ros::Time::init();
  
  Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();
  guess<<
  /*  0.999997, -0.000512974,  -0.00239359,       1.3324,
  0.00052067,     0.999995,   0.00321544,   0.00767631,
  0.00239192,  -0.00321668,     0.999992,     0.019645,
           0,          0,          0,          1;*/
 0.999999, -0.000230269,  -0.00131241,       1.3645,
 0.000236686,     0.999988,   0.00489182,   -0.0187628,
  0.00131126,  -0.00489212,     0.999987 ,  0.00355705,
           0,            0,            0,            1;

  
  Sophus::SE3 SE3_Rt_guess(guess.block(0,0,3,3).cast<double>(),guess.block(0,3,3,1).cast<double>());
  Eigen::Matrix<double, 6, 1> p_guess=SE3_Rt_guess.log();
  //p_guess(2)=p_guess(3)=p_guess(4)=0;
  std::cout<<"p_guess: \n"<<p_guess.transpose()<<std::endl;
  //guess=Sophus::SE3::exp(p_guess).matrix().cast<float>();
  std::cout<<"guess: \n"<<guess<<std::endl;
  
  Eigen::Matrix4d tf_s2s_gt;
  tf_s2s_gt<<
    0.999988,   0.00489403, -0.000211991,    0.0195194,
  -0.0048943,     0.999987,   -0.0013073,   -0.0139887,
 0.000205591,   0.00130833,     0.999999,       1.3645,
           0,          0,          0,          1;
  tf_s2s_gt=tf_velo2cam.inverse()*tf_s2s_gt*tf_velo2cam;
  // T(R,t)->李群SE(3)->李代数se(3)
  Sophus::SE3 SE3_Rt_gt(tf_s2s_gt.block(0,0,3,3).cast<double>(),tf_s2s_gt.block(0,3,3,1).cast<double>());
  Eigen::Matrix<double, 6, 1> p_gt=SE3_Rt_gt.log();
  std::cout<<"p_gt: \n"<<p_gt.transpose()<<std::endl;
  std::cout<<"tf_s2s_gt: \n"<<tf_s2s_gt<<std::endl;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr matched_gt(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud (*source_cloud, *matched_gt,tf_s2s_gt );
  pcl::io::savePCDFileASCII ("matched_gt.pcd", *matched_gt);
  pcl::PointCloud<pcl::PointXYZI>::Ptr matched_guess(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud (*source_cloud, *matched_guess,guess );
  pcl::io::savePCDFileASCII ("matched_guess.pcd", *matched_guess);
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr distance_filted(new pcl::PointCloud<pcl::PointXYZI>());
  distance_filted=distance_filter(target_cloud,2,50);
  *target_cloud = *distance_filted;
  distance_filted=distance_filter(source_cloud,2,50);
  *source_cloud=*distance_filted;
  pcl::io::savePCDFileASCII ("target_cloud.pcd", *target_cloud);
  pcl::io::savePCDFileASCII ("source_cloud.pcd", *source_cloud);
  
  std::cout << "--- pcl::NDT_OMP ---" << std::endl;
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setInputTarget(target_cloud);
  ndt_omp->setInputSource(source_cloud);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  ndt_omp->setStepSize(0.1);
  ndt_omp->setTransformationEpsilon(0.01);
  ndt_omp->setMaximumIterations(64);
  ndt_omp->setOulierRatio(0.55);
  pcl::PointCloud<pcl::PointXYZI>::Ptr matched(new pcl::PointCloud<pcl::PointXYZI>());
  auto t1 = ros::WallTime::now();
  ndt_omp->align(*matched,guess);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << "DIRECT1--nr_iterations_:" <<ndt_omp->getFinalNumIteration()<<std::endl; 
  //std::cout<<"Transform: \n"<<ndt_omp->getFinalTransformation()<<std::endl;
  Eigen::Matrix4d tf_s2s=ndt_omp->getFinalTransformation().cast<double>();
  Eigen::Matrix4d tf_s2s_cam=tf_velo2cam*tf_s2s*tf_velo2cam.inverse();
  std::cout<<"tf_s2s: \n"<<tf_s2s<<std::endl;
  std::cout<<"tf_s2s_cam: \n"<<tf_s2s_cam<<std::endl;
  
  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setInputSource(source_cloud);
  gicp.setInputTarget(target_cloud);
  gicp.align(*matched,guess);    

  std::cout <<"tf_gicp=\n"<< gicp.getFinalTransformation() << std::endl;
  
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("matched.pcd", *matched);
  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_handler(source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> matched_handler(matched, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(matched, matched_handler, "matched");
  vis.spin();

  return 0;
}