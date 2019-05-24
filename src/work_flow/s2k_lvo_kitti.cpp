#include <iostream>
#include <ros/ros.h>
#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <Eigen/Dense>
#include <string>  
#include <vector>  
#include <fstream>
#include <chrono>
#include <ctime>
#include <climits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace Eigen;
using namespace std;

#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/gicp_omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <lidar_odometry/direct_spase.hpp>
#include <lidar_odometry/epipolar_geometry.hpp>

using namespace cv;

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
  
/*pcl::PointCloud<PointT>::Ptr label_filter(const pcl::PointCloud<PointT>::Ptr& cloud,cv::Mat label_img,Eigen::Matrix4d T,Eigen::Matrix3d K) {
    std::map<uint32_t, cv::Vec3b> label_rgb;
    label_rgb[7]=cv::Vec3b(128, 64,128);
    label_rgb[8]=cv::Vec3b(244, 35, 232);
    label_rgb[11]=cv::Vec3b(70, 70, 70);
    label_rgb[12]=cv::Vec3b(102, 102, 156);
    label_rgb[13]=cv::Vec3b(190, 153, 153);
    label_rgb[17]=cv::Vec3b(153, 153, 153);
    label_rgb[19]=cv::Vec3b(250, 170, 30);
    label_rgb[20]=cv::Vec3b(220, 220, 0);
    label_rgb[21]=cv::Vec3b(107, 142, 35);
    label_rgb[22]=cv::Vec3b(152, 251, 152);
    label_rgb[23]=cv::Vec3b(70, 130, 180);
    label_rgb[24]=cv::Vec3b(220, 20, 60);
    label_rgb[25]=cv::Vec3b(255, 0, 0);
    label_rgb[26]=cv::Vec3b(0, 0, 142);
    label_rgb[27]=cv::Vec3b(0, 0, 70);
    label_rgb[28]=cv::Vec3b(0, 60, 100);
    label_rgb[29]=cv::Vec3b(0, 80, 100);
    label_rgb[32]=cv::Vec3b(0, 0, 230);
    label_rgb[33]=cv::Vec3b(119, 11, 32);
    label_rgb[0]=cv::Vec3b(0, 0, 0);
    
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());
     
    for(int i=0;i<cloud->size();i++)
    {
      PointT p=cloud->points[i];
      Eigen::Vector4d p3d_velo(p.x,p.y,p.z,1);
      // body（cam）系下3d点
      Eigen::Vector4d p3d_body=T*p3d_velo;
      if(p3d_body[2]<0)continue;
      double u,v;
      u=K(0,0)*p3d_body[0] / p3d_body[2]+K(0,2);
      v=K(1,1)*p3d_body[1] / p3d_body[2]+K(1,2);
      // 去掉邻近边缘处的点
      if ( u < 1 || v < 1 || ( u+1 ) >label_img.cols || ( v+1 ) >label_img.rows )
	continue;
      Vec3b pixel=label_img.at<Vec3b>(v,u);
      p.r=(int)pixel[2];p.g=(int)pixel[1];p.b=(int)pixel[0];
      for(std::map<uint32_t, cv::Vec3b>::iterator it = label_rgb.begin();it!=label_rgb.end();it++) 
      {
	if(it->second==pixel)
	  p.label=it->first;
      }
      if(p.label>=11&&p.label<=20)
	filtered->points.push_back(p);    
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    filtered->header = cloud->header;
    
    return filtered;
}*/
  
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
  std::string imgsdir="/media/whu/HD_CHEN_2T/02data/KITTI_odometry/dataset/sequences/"+seq+"/image_2";
  std::string gt_file="/home/whu/data/data_source_KITTI/devkit_old/cpp/data/poses/"+seq+".txt";
  std::string odom_file=res_dir+"/data/KITTI_"+seq+"_odom.txt";
  std::string scan_error_file=res_dir+"/errors/KITTI_"+seq+"_scan_error.txt";
  std::string odom_error_file=res_dir+"/errors/KITTI_"+seq+"_odom_error.txt";
  ifstream fin ( calibdir );
  string tmp;
 
  // load ground truth file
  //世界坐标系map,以第一帧velo为基准建立，而kitti ground truth 是以第一帧camera为世界坐标系的velo pose，需要世界系calibration参数 
  Eigen::Matrix4d P_rect_00=Eigen::Matrix4d::Identity();
  fin>>tmp>>P_rect_00(0,0)>>P_rect_00(0,1)>>P_rect_00(0,2)>>P_rect_00(0,3)
  >>P_rect_00(1,0)>>P_rect_00(1,1)>>P_rect_00(1,2)>>P_rect_00(1,3)
  >>P_rect_00(2,0)>>P_rect_00(2,1)>>P_rect_00(2,2)>>P_rect_00(2,3);
  
  getline(fin,tmp);getline(fin,tmp);
  Eigen::Matrix4d P_rect_02=Eigen::Matrix4d::Identity();
  fin>>tmp>>P_rect_02(0,0)>>P_rect_02(0,1)>>P_rect_02(0,2)>>P_rect_02(0,3)
  >>P_rect_02(1,0)>>P_rect_02(1,1)>>P_rect_02(1,2)>>P_rect_02(1,3)
  >>P_rect_02(2,0)>>P_rect_02(2,1)>>P_rect_02(2,2)>>P_rect_02(2,3);
  
  getline(fin,tmp);getline(fin,tmp);
  Eigen::Matrix4d tf_velo2cam0=Eigen::Matrix4d::Identity();
  fin>>tmp>>tf_velo2cam0(0,0)>>tf_velo2cam0(0,1)>>tf_velo2cam0(0,2)>>tf_velo2cam0(0,3)
  >>tf_velo2cam0(1,0)>>tf_velo2cam0(1,1)>>tf_velo2cam0(1,2)>>tf_velo2cam0(1,3)
  >>tf_velo2cam0(2,0)>>tf_velo2cam0(2,1)>>tf_velo2cam0(2,2)>>tf_velo2cam0(2,3);
  
  Eigen::Matrix3d K =P_rect_00.block<3, 3>(0, 0); 

  Eigen::Matrix4d tf_cam0tocam2=Eigen::Matrix4d::Identity();
  tf_cam0tocam2.block<3, 1>(0, 3)=K.inverse()*P_rect_02.block<3, 1>(0, 3);

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
    poses_velo.push_back(tf_velo2cam0.inverse()*p*tf_velo2cam0);
    }
  }
  fclose(fp);
    
  cv::Mat color,gray,source_color;
  
  // downsampling&ndt configure 
  pcl::PointCloud<PointT>::Ptr matched(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
  ros::Time::init(); 
  
  std::cout << "--- pcl::NDT_OMP ---" << std::endl;
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr reg_s2s(new pclomp::NormalDistributionsTransform<PointT, PointT>());
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr reg_s2k(new pclomp::NormalDistributionsTransform<PointT, PointT>());
  reg_s2s->setResolution(1.0);
  reg_s2s->setNumThreads(8);
  reg_s2s->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  reg_s2s->setTransformationEpsilon(0.01);
  reg_s2s->setMaximumIterations(64);
  *reg_s2k=*reg_s2s;
  
  // pcd file
  chdir(pcdsdir.c_str());
  struct dirent **namelist;
  int n=scandir(pcdsdir.c_str(),&namelist,pcdfilter,alphasort);
  if(n < 0)
  {
    cout << "scandir return "<< n  << endl;
    return 0;
  }
   
  Eigen::Matrix4d odom=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_velo=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_s2k_error=Eigen::Matrix4d::Identity();
  FILE *fp_odom = fopen(odom_file.c_str(),"w+");
  fprintf(fp_odom,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    odom(0,0),odom(0,1),odom(0,2),odom(0,3),
	    odom(1,0),odom(1,1),odom(1,2),odom(1,3),
	    odom(2,0),odom(2,1),odom(2,2),odom(2,3));
  FILE *fp_scan_error = fopen(scan_error_file.c_str(),"w+");
  fprintf(fp_scan_error,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	    tf_s2k_error(0,0),tf_s2k_error(0,1),tf_s2k_error(0,2),tf_s2k_error(0,3),
	    tf_s2k_error(1,0),tf_s2k_error(1,1),tf_s2k_error(1,2),tf_s2k_error(1,3),
	    tf_s2k_error(2,0),tf_s2k_error(2,1),tf_s2k_error(2,2),tf_s2k_error(2,3)); 
  
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr filtered,key;
  Eigen::Matrix4d tf_s2s,tf_s2k,key_pose;
  int key_id=0,key_interval=10;
  for(int i= 0; i <n; i++)
  {
    std::string pcd = namelist[i]->d_name;
    if(pcl::io::loadPCDFile<PointT>(pcd, *cloud)) {
      std::cerr << "failed to load " << pcd << std::endl;
      return 0;
    } 
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*downsampled);   
    filtered = distance_filter(downsampled,0.5,100);
            
    if(i==0)
    {
      reg_s2s->setInputTarget(filtered);
      tf_s2s.setIdentity();tf_s2s(0,3)=1.5;
      
      key=filtered;
      reg_s2k->setInputTarget(key);
      key_id=i;
      tf_s2k.setIdentity();
      key_pose.setIdentity(); 
      continue;
    }
    
    //s2s
    reg_s2s->setInputSource(filtered); 
    reg_s2s->align(*matched, tf_s2s.cast<float>());
    tf_s2s = reg_s2s->getFinalTransformation().cast<double>();
    reg_s2s->setInputTarget(filtered); 
    
    //s2k
    std::cout << std::endl<<i<<" to "<<key_id<<"  "<<pcd  << std::endl;
    tf_s2k=tf_s2k*tf_s2s;
    odom_velo = key_pose * tf_s2k;
    
    Eigen::Matrix4d tf_s2k_gt=poses_cam[key_id].inverse()*poses_cam[i];
    Eigen::Matrix4d tf_s2k_gt_velo=tf_velo2cam0.inverse()*tf_s2k_gt*tf_velo2cam0;
    std::cout<<"tf_s2k_gt_velo: \n"<<tf_s2k_gt_velo<<std::endl;
    std::cout<<"tf_s2k by acculating s2s: \n"<<tf_s2k<<std::endl;
    tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
    
    //std::cout<<"tf_s2k_gt_cam2: \n"<<tf_cam0tocam2*tf_s2k_gt*tf_cam0tocam2.inverse()<<std::endl;
    
    if(i%key_interval==0) {   
      reg_s2k->setInputSource(filtered); 
      reg_s2k->align(*matched, tf_s2k.cast<float>());
      tf_s2k = reg_s2k->getFinalTransformation().cast<double>();    
      std::cout<<"update tf_s2k per key_interval f: \n"<<tf_s2k<<std::endl;
      
      tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
      Sophus::SE3 SE3_Rt0(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
       
      //refine roll/pitch with images
      /*char temp[7],temp2[7];
      sprintf(temp,"%06d",i-key_interval);
      sprintf(temp2,"%06d",i);
      //reg_direct_sparse(cloud,imgsdir+"/"+temp2+".png",imgsdir+"/"+temp+".png",tf_cam0tocam2*tf_velo2cam0,K,tf_s2k);
      if(reg_epipolar_geometry(imgsdir+"/"+temp2+".png",imgsdir+"/"+temp+".png",tf_cam0tocam2*tf_velo2cam0,K,tf_s2k))
      {
	std::cout<<"tf_s2k_error_velo: "<<SE3_Rt0.log().transpose()<<std::endl; 
	std::cout<<"update tf_s2k with reg_epipolar_geometry successfully: \n"<<tf_s2k<<std::endl;
      }
      else
	std::cout<<"update tf_s2k with reg_epipolar_geometry unsuccessfully!"<<std::endl;*/
  
      //俯仰角设为0
      /*if(0){
      Sophus::SE3 SE3_s2k(tf_s2k.block(0,0,3,3),tf_s2k.block(0,3,3,1));
      Eigen::Matrix<double, 6, 1> p=SE3_s2k.log();
      p(4,0)=0;
      tf_s2k=Sophus::SE3::exp(p).matrix();
      std::cout<<"update tf_s2k (pitch=0): \n"<<tf_s2k<<std::endl;
      }*/
      
      tf_s2k_error=tf_s2k_gt_velo.inverse()*tf_s2k;
      odom_velo= key_pose * tf_s2k;
      
      key=filtered;
      reg_s2k->setInputTarget(key);   
      key_id=i;
      tf_s2k.setIdentity();
      key_pose=odom_velo;
    }
    
    //elevation
    Sophus::SE3 SE3_Rt(tf_s2k_error.block(0,0,3,3),tf_s2k_error.block(0,3,3,1));
    std::cout<<"tf_s2k_error_velo: "<<SE3_Rt.log().transpose()<<std::endl;  
    
    //output
    odom=tf_velo2cam0*odom_velo*tf_velo2cam0.inverse();
    Eigen::Matrix4d odom_error=poses_cam[i].inverse()*odom;
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
  
  }
  fclose(fp_odom);
  fclose(fp_scan_error);
    
  return 0;
}
