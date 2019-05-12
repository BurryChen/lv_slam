#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include <Eigen/Geometry> // for eulerAngles
#include <iostream>
#include <ndt_d2d/ndt_matcher_d2d.h>
#include <ndt_d2d/ndt_matcher_p2d.h>
#include <string>

/***********************************************************************************
 * workfolw for Huang's Handed scanner.
 ***********************************************************************************/

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: match target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }
  float scale=1.0;  // 1m=1000mm
  // downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.001f*scale, 0.001f*scale, 0.001f*scale);

  /*voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  *source_cloud = *downsampled;*/
  
  std::cout<<"target size:"<<target_cloud->size()<<" source size:"<<source_cloud->size()<<std::endl;

  ros::Time::init();
  
  Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr matched(new pcl::PointCloud<pcl::PointXYZ>());
	   
  /*std::cout << "--- pcl::NDT_OMP shoes1 ---" << std::endl;
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(0.01*scale);
  ndt_omp->setInputTarget(target_cloud);
  ndt_omp->setInputSource(source_cloud);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  ndt_omp->setStepSize(0.005*scale);
  ndt_omp->setTransformationEpsilon(0.0005*scale);
  ndt_omp->setMaximumIterations(64);
  ndt_omp->setOulierRatio(0.55);
  pcl::PointCloud<pcl::PointXYZ>::Ptr matched(new pcl::PointCloud<pcl::PointXYZ>());
  auto t1 = ros::WallTime::now();
  ndt_omp->align(*matched,guess);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << "DIRECT1" <<std::endl; 
  std::cout<<"Transform: \n"<<ndt_omp->getFinalTransformation()<<std::endl;*/
  
  /*std::cout << "--- pcl::NDT drugon---" << std::endl;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt->setTransformationEpsilon(0.0000001);
  //为More-Thuente线搜索设置最大步长
  ndt->setStepSize(1.0);
  //设置NDT网格结构的分辨率（VoxelGridCovariance）
  ndt->setResolution(0.005);
  //设置匹配迭代的最大次数
  ndt->setMaximumIterations(400);
  ndt->setInputTarget(target_cloud);
  ndt->setInputSource(source_cloud);

  auto t1 = ros::WallTime::now();
  ndt->align(*matched);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;  
  pcl::io::savePCDFileASCII ("matched_pcl.pcd", *matched);*/
  
  //NDT_d2d
  double __res[] ={0.005};
  std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));
  
  lslgeneric::NDTMatcherD2D matcherD2D(true, false, resolutions);
  lslgeneric::NDTMatcherP2D matcherP2D(resolutions);
  matcherD2D.ITR_MAX = 400;
  matcherD2D.n_neighbours = 1;
  
  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout;
  Tout.matrix()=guess.cast<double>();
  
  //bool ret = matcherD2D.match(*target_cloud,*source_cloud,Tout,true);
  bool ret = matcherP2D.match(*target_cloud,*source_cloud,Tout);
  std::cout<<ret<<"Transform_d2d: \n"<<Tout.matrix()<<std::endl;
  lslgeneric::transformPointCloudInPlace(Tout,*matched);

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("matched.pcd", *matched);
  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> matched_handler(matched, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(matched, matched_handler, "matched");
  vis.spin();

  return 0;
}
