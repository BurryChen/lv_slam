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
#include <ndt_pca/ndt_pca.h>

#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <Eigen/Dense>
#include <string>  
#include <vector>  
#include <fstream>
#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace Eigen;
using namespace std;

typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr distance_filter(const pcl::PointCloud<PointT>::Ptr& cloud,double distance_near_thresh, double distance_far_thresh) {
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
  

pcl::PointCloud<PointT>::Ptr vertical_angle_calibration(const pcl::PointCloud<PointT>::ConstPtr& cloud) { 
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>()); 
    filtered->reserve(cloud->size()); 
 
    for (size_t cp = 0; cp < cloud->points.size (); ++cp) 
    { 
      PointT p=cloud->points[cp]; 
      double detal=0.22*M_PI/180;  //delta_vertical_angle=0.22 
           
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
  
int pcdfilter(const struct dirent *filename)    //文件筛选器
{
    size_t len;
 
    len = strlen(filename->d_name);
    if (len >= 4
        && filename->d_name[len - 4] == '.'
        && filename->d_name[len - 3] == 'p'
        && filename->d_name[len - 2] == 'c'
        && filename->d_name[len - 1] == 'd')
        return 1;
 
    return 0;
}

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: odom_kitti res_dir sequence" << std::endl;
    return 0;
  }
  
  std::string res_dir=argv[1];
  std::string seq=argv[2];
  std::string pcdsdir="/media/whu/HD_CHEN_2T/02data/KITTI_odometry/velobag/velo_"+seq+".bag_pcd";
  std::string calibdir="/media/whu/HD_CHEN_2T/02data/KITTI_odometry/dataset/sequences/"+seq+"/calib.txt";
  std::string gt_file="/home/whu/data/data_source_KITTI/devkit_old/cpp/data/poses/"+seq+".txt";
  std::string odom_file=res_dir+"/data/KITTI_"+seq+"_odom.txt";
  std::string scan_error_file=res_dir+"/errors/KITTI_"+seq+"_scan_error.txt";
  std::string odom_error_file=res_dir+"/errors/KITTI_"+seq+"_odom_error.txt";
  ifstream fin ( calibdir );
  string tmp;
  for(int i=0;i<4;i++) getline(fin,tmp);
 
  // load ground truth file
  //世界坐标系map,以第一帧velo为基准建立，而kitti ground truth 是以第一帧camera为世界坐标系的velo pose，需要世界系calibration参数
  Eigen::Matrix4d tf_velo2cam=Eigen::Matrix4d::Identity();
  fin>>tmp>>tf_velo2cam(0,0)>>tf_velo2cam(0,1)>>tf_velo2cam(0,2)>>tf_velo2cam(0,3)
  >>tf_velo2cam(1,0)>>tf_velo2cam(1,1)>>tf_velo2cam(1,2)>>tf_velo2cam(1,3)
  >>tf_velo2cam(2,0)>>tf_velo2cam(2,1)>>tf_velo2cam(2,2)>>tf_velo2cam(2,3);
  vector<Eigen::Matrix4d> poses_cam,poses_velo;
  FILE *fp = fopen(gt_file.c_str(),"r");
  if (!fp) printf("Can't open gt_file!");
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
    
  // downsampling&ndt configure 
  pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr source_cloud(new pcl::PointCloud<PointT>());
  
  pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
  
  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  *source_cloud = *downsampled;
  
  pcl::PointCloud<PointT>::Ptr distance_filted(new pcl::PointCloud<PointT>());
  
  ros::Time::init(); 
  
  std::cout << "--- pcl::NDT_OMP ---" << std::endl;
  pclpca::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_omp(new pclpca::NormalDistributionsTransform<PointT, PointT>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclpca::DIRECT1);
  ndt_omp->setStepSize(0.1);
  ndt_omp->setTransformationEpsilon(0.01);
  ndt_omp->setMaximumIterations(64);
  ndt_omp->setOulierRatio(0.55);
  pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());

  // pcd file
  chdir(pcdsdir.c_str());
  struct dirent **namelist;
  int n=scandir(pcdsdir.c_str(),&namelist,pcdfilter,alphasort);
  if(n < 0)
  {
    cout << "scandir return "<< n  << endl;
    return 0;
  }
  
  // match
  Eigen::Matrix4d tf_s2s=Eigen::Matrix4d::Identity();
  tf_s2s(0,3)=1.5;
  Eigen::Matrix4d tf_s2s_cam=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_s2s_error=Eigen::Matrix4d::Identity();
  FILE *fp_odom = fopen(odom_file.c_str(),"w+");
  fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
  FILE *fp_scan_error = fopen(scan_error_file.c_str(),"w+");
  fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2s_error(0,0),tf_s2s_error(0,1),tf_s2s_error(0,2),tf_s2s_error(0,3),
	    tf_s2s_error(1,0),tf_s2s_error(1,1),tf_s2s_error(1,2),tf_s2s_error(1,3),
	    tf_s2s_error(2,0),tf_s2s_error(2,1),tf_s2s_error(2,2),tf_s2s_error(2,3)); 
  
  for(int i= 0; i <n-1; i ++)
  {
    //scan_match(namelist[i]->d_name,namelist[i+1]->d_name);
    std::string target_pcd = namelist[i]->d_name;
    std::string source_pcd = namelist[i+1]->d_name;
    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
      std::cerr << "failed to load " << target_pcd << std::endl;
      return 0;
    }
    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
      std::cerr << "failed to load " << source_pcd << std::endl;
      return 0;
    }
    
    /*voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    *source_cloud = *downsampled;

    distance_filted=distance_filter(target_cloud,1,50);
    *target_cloud = *distance_filted;
    distance_filted=distance_filter(source_cloud,1,50);
    *source_cloud=*distance_filted;*/
  
    distance_filted=vertical_angle_calibration(target_cloud);
    *target_cloud = *distance_filted;
    distance_filted=vertical_angle_calibration(source_cloud);
    *source_cloud=*distance_filted;
    
    ndt_omp->setInputTarget(target_cloud);
    ndt_omp->setInputSource(source_cloud);
    auto t1 = ros::WallTime::now();
    ndt_omp->align(*matched,tf_s2s.cast<float>());
    auto t2 = ros::WallTime::now();
    std::cout <<i<<"  "<<target_pcd<<" /"<<source_pcd<< " t: " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
    tf_s2s=ndt_omp->getFinalTransformation().cast<double>();
    tf_s2s_cam=tf_velo2cam*tf_s2s*tf_velo2cam.inverse();
    std::cout<<"tf_s2s: \n"<<tf_s2s<<std::endl;
    std::cout<<"tf_s2s_cam: \n"<<tf_s2s_cam<<std::endl;
    odom=odom*tf_s2s_cam;
    
    //elevation 
    Eigen::Matrix4d tf_s2s_gt=poses_cam[i].inverse()*poses_cam[i+1];
    std::cout<<"tf_s2s_gt: \n"<<tf_s2s_gt<<std::endl;
    tf_s2s_error=tf_s2s_gt.inverse()*tf_s2s_cam;
    //std::cout<<"tf_s2s_error: \n"<<tf_s2s_error<<std::endl;
    Sophus::SE3 SE3_Rt(tf_s2s_error.block(0,0,3,3),tf_s2s_error.block(0,3,3,1));
    std::cout<<"tf_s2s_error: "<<SE3_Rt.log().transpose()<<std::endl;
    
    fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
    fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2s_error(0,0),tf_s2s_error(0,1),tf_s2s_error(0,2),tf_s2s_error(0,3),
	    tf_s2s_error(1,0),tf_s2s_error(1,1),tf_s2s_error(1,2),tf_s2s_error(1,3),
	    tf_s2s_error(2,0),tf_s2s_error(2,1),tf_s2s_error(2,2),tf_s2s_error(2,3));
  
  }
  fclose(fp_odom);
  fclose(fp_scan_error);
    
  return 0;
}