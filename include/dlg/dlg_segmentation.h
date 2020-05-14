// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// 
// #include <pcl/io/obj_io.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJPolylineMesh.h所属头文件；

#include "PolylineMesh.h"


using namespace pcl;
using namespace pcl::io;
using namespace std;

//功能：pointcloud preprocess
//  输入：const PCLPointCloud2ConstPtr &input_cloud, 地址值不能改，指向的值不能修改   
//  参数为常对象const PCLPointCloud2ConstPtr的引用&input_cloud，
//  智能指针PCLPointCloud2ConstPtr指向常对象::pcl::PCLPointCloud2 const
//  输出：PCLPointCloud2Ptr output_cloud  地址值不能改，指向的值可以修改   
//  常引用作为函数参数，1）安全，2）提高效率
int preprocess(const PCLPointCloud2ConstPtr &input_cloud, const PCLPointCloud2Ptr &output_cloud)
{
  pcl::PLYWriter plywriter;
  
  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pcl::PCLPointCloud2::Ptr cloud_filtered_pass (new pcl::PCLPointCloud2);
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.10, 0.20);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered_pass);
  std::cout << "PointCloud after filtering cloud_filtered_pass has: " << cloud_filtered_pass->width  << " data points." << std::endl; //*
  //plywriter.write("cloud_filtered_pass.ply",*cloud_filtered_pass,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);
  
  //剔除离群点
  pcl::StatisticalOutlierRemoval< pcl::PCLPointCloud2 > sor;
  pcl::PCLPointCloud2::Ptr cloud_filtered_sor (new pcl::PCLPointCloud2);
  sor.setInputCloud(cloud_filtered_pass); 
  sor.setMeanK(50);                             //对每个点分析的临近点个数
  sor.setStddevMulThresh(1);                    //一个点距离超出平均距离2倍中误差
  sor.filter(*cloud_filtered_sor);
  std::cout << "PointCloud after filtering StatisticalOutlierRemoval has: " << cloud_filtered_sor->width  << " data points." << std::endl; //*
  //plywriter.write("cloud_filtered_sor.ply",*cloud_filtered_sor,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);
    
  //平面参数化模型投影点云
  // Create a set of planar coefficients with X=Y=0,Z=0.05, 安装参数
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1;
  coefficients->values[3] = 0;

  // Create the filtering object  
  pcl::ProjectInliers<pcl::PCLPointCloud2> proj;
  pcl::PCLPointCloud2::Ptr cloud_projected(new pcl::PCLPointCloud2);
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered_sor);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cout << "PointCloud after filtering ProjectInliers has: " << cloud_projected->width   << " data points." << std::endl; //*
  //plywriter.write("cloud_projected.ply",*cloud_projected,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //体素化网格滤波器降采样
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  pcl::PCLPointCloud2::Ptr cloud_filter(new pcl::PCLPointCloud2);
  vg.setInputCloud (cloud_projected);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*output_cloud);
  std::cout << "PointCloud after filtering VoxelGrid has: " << output_cloud->width  << " data points." << std::endl; 
  plywriter.write("cloud_preprocessed.ply",*output_cloud,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);
    
  return output_cloud->width;
}


//功能：点云到线段的定点
int CloudtoSegment(const pcl::PointCloud<pcl::PointWithRange>::Ptr &cloud_line, const pcl::ModelCoefficients::Ptr &coefficients,const pcl::PointCloud<pcl::PointWithRange>::Ptr &cloud_vertice)
{

    std::cerr << "Model coefficients: " << std::endl;
    for(size_t i=0;i<coefficients->values.size();i++)
      std::cerr<<coefficients->values[i]<<"   ";
    std:cout<<std::endl;
    Eigen::Vector4f pt(0,0,0,0) , line_pt(0,0,0,0), line_dir(0,0,0,0);
    Eigen::Vector4f point2line_foot(0,0,0,0);

    line_pt<<coefficients->values[0],coefficients->values[1],coefficients->values[2],0;
    line_dir<<coefficients->values[3],coefficients->values[4],coefficients->values[5],0;
    
    float point2line_ratio;
    pcl::PointCloud<pcl::PointWithRange>::Ptr  cloud_foots(new pcl::PointCloud<pcl::PointWithRange>);
    for (size_t i = 0; i < cloud_line->points.size(); ++i)
    {     

      pt<<cloud_line->points[i].x,cloud_line->points[i].y,cloud_line->points[i].z,0;
           
      //if(sqrPointToLineDistance(pt,line_pt,line_dir)>0.2) continue;
           
      point2line_ratio=line_dir.dot(pt - line_pt) / line_dir.squaredNorm ();            //垂足到起点距离
      
      point2line_foot=point2line_ratio*line_dir+line_pt;                                
      point2line_foot(3)=point2line_ratio;
      
      pcl::PointWithRange foot;                          //垂足坐标
      foot.x=point2line_foot(0);
      foot.y=point2line_foot(1);
      foot.z=point2line_foot(2);
      foot.range=point2line_foot(3);
      
      cloud_foots->points.push_back(foot);
      //std::cout<<"    " <<point2line_foot(0)<<"    " <<point2line_foot(1)<<"    " <<point2line_foot(2)<<"    " <<point2line_foot(3)<<std::endl;     
    }
   
    cloud_foots->width = (uint32_t) cloud_line->points.size();
    cloud_foots->height = 1;
    //cloud_foots.reserve();
    pcl::PLYWriter plywriter;
    //plywriter.write<pcl::PointWithRange> ("cloud_foots.ply", *cloud_foots, false,false);
    std::cout <<"cloud_foots:"<<cloud_foots->points.size()<< std::endl;
    
       
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointWithRange>::Ptr tree (new pcl::search::KdTree<pcl::PointWithRange>);
  tree->setInputCloud (cloud_foots);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointWithRange> ec;
  ec.setClusterTolerance (0.10); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_foots);
  ec.extract (cluster_indices);

  int i = 0;
  Eigen::Vector4f point1,point2;

  //pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_vertice (new pcl::PointCloud<pcl::PointWithRange>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointWithRange>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_foots->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    float minRange=99999,maxRange=-99999,tempRange;
    int index_begin=0,index_end=0;
    for (size_t j = 0; j < cloud_cluster->points.size(); ++j)
    {
      tempRange=cloud_cluster->points[j].range;
      if(tempRange<minRange)
      {
	minRange=tempRange;  
        index_begin=j;	
      }
      if(tempRange>maxRange) 
      {
	maxRange=tempRange;  
	index_end=j;	
      }
    }
      
    float error_range=0.025;
    
    Eigen::Vector4f v_begin(0,0,0,0),v_end(0,0,0,0);
    v_begin=(minRange+error_range)*line_dir+line_pt; 
    v_end=(maxRange-error_range)*line_dir+line_pt;                                
      
    pcl::PointWithRange pt1,pt2;   
    pt1.x=v_begin(0);
    pt1.y=v_begin(1);
    pt1.z=v_begin(2);
    pt1.range=minRange+error_range;      
    cloud_vertice->points.push_back(pt1);
    pt2.x=v_end(0);
    pt2.y=v_end(1);
    pt2.z=v_end(2);
    pt2.range=maxRange-error_range;      
    cloud_vertice->points.push_back(pt2);
//     cloud_vertice->points.push_back(cloud_cluster->points[index_begin]);
//     cloud_vertice->points.push_back(cloud_cluster->points[index_end]);
   
    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << index_begin<<"   "<<index_end <<std::endl;
    std::cout<<"segment_"<<i<<":  "<<pt1<<",   "<<pt2 <<std::endl;

    std::stringstream ss;
    ss << "cloud_cluster_" << i << ".ply";
   // plywriter.write<pcl::PointWithRange> (ss.str (), *cloud_cluster, false,false); //*
    i++;
  }
 
}
