#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJPolylineMesh.h所属头文件；

#include <dlg/PolylineMesh.h>
#include <dlg/dlg_segmentation.h>

#include <pcl/filters/clipper3D.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;


int 
main (int argc, char** argv)
{
  std::string fn;
  fn=argv[1];  
  //输出线段
  PolylineMesh linemesh;   
  //1、Read in the cloud data
  pcl::PLYReader reader;
  pcl::PLYWriter plywriter;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  //reader.read ("cloud_all_lines.ply", *cloud);
  reader.read(fn,*cloud); 
  std::cout << "PointCloud before filtering has: " << cloud->width << " data points." << std::endl; //*
    
  //2、preprocess
  pcl::PCLPointCloud2::Ptr cloud_processed(new pcl::PCLPointCloud2);
  preprocess(cloud,cloud_processed);
  
  //reader.read("cloud_preprocessed.ply",*cloud_processed); 
  std::cout <<std::endl<< "PointCloud after filtering VoxelGrid has: " << cloud_processed->width  << " data points." << std::endl; 
   
  //3、line segmentation detection (LSG) and DLG product
  
  // Create the segmentation object for the liner model and set all the parameters
  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointWithRange>);
  pcl::fromPCLPointCloud2(*cloud_processed,*cloud_filtered);
  pcl::SACSegmentation<pcl::PointWithRange> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_line (new pcl::PointCloud<pcl::PointWithRange>);
  
  seg.setOptimizeCoefficients (true);                      //设置优化参数
  //seg.setModelType (pcl::SACMODEL_PLANE);                  //设置分割模型为平面
  seg.setModelType (pcl::SACMODEL_LINE);                   //设置分割模型为直线
  //seg.setModelType(pcl::SACMODEL_STICK);
  seg.setMethodType (pcl::SAC_RANSAC);                     //参数估计方法
  //seg.setMaxIterations (100);                              //最大迭代次数
  seg.setDistanceThreshold (0.05);                         //内点到模型的距离
  
  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_all_lines (new pcl::PointCloud<pcl::PointWithRange>),cloud_temp (new pcl::PointCloud<pcl::PointWithRange>);
  //pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_vertices (new pcl::PointCloud<pcl::PointWithRange>);
  int linenum=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 100)
  {
    //std::cout << "PointCloud before line segmentation: "<< cloud_filtered->points.size() << " data points." << std::endl;
    // Segment the largest liner component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a liner model for the given dataset." << std::endl;
      break;
    }
   
    std::cerr <<std::endl<< "Model inliers: " << inliers->indices.size () << std::endl;
    
    // Extract the liner inliers from the input cloud
    pcl::ExtractIndices<pcl::PointWithRange> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the liner surface
    extract.filter (*cloud_line);
    
    //std::cout << "PointCloud representing the line component: " << cloud_line->points.size() << " data points." << std::endl;
    *cloud_all_lines+=*cloud_line;
    
    std::stringstream ss;
    ss << "cloud_line_" << linenum << ".ply";
    //plywriter.write<pcl::PointWithRange> (ss.str (), *cloud_line, false,false);
    std::cout << ss.str() <<"_"<<cloud_line->points.size()<< std::endl;
    
    //if( cloud_line->points.size() > 100)
    CloudtoSegment(cloud_line,coefficients,linemesh.cloud_vertices);
/*    for (size_t i = 0; i < cloud_line->points.size(); ++i)
      std::cerr << "    " << cloud_line->points[i].x << " "
                                               << cloud_line->points[i].y << " "
                                               << cloud_line->points[i].z << std::endl;   */ 
    // Remove the liner inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_temp);    
    
    *cloud_filtered = *cloud_temp; 
    linenum++;
    //writer.write<pcl::PointWithRange> ("cloud_line", *cloud_filtered, false);   
  } 
  //plywriter.write<pcl::PointWithRange> ("cloud_all_lines.ply", *cloud_all_lines,false,false);
  //plywriter.write<pcl::PointWithRange> ("cloud_rest.ply", *cloud_filtered,false,false);


  for (size_t i = 0; i <linemesh.cloud_vertices->points.size()/2; ++i)
  {
    pcl::Vertices temp;
    temp.vertices.push_back(i*2),temp.vertices.push_back(i*2+1);  
    linemesh.polylines.push_back(temp); 
  }
  
  float resolution = 0.01f;

  pcl::octree::OctreePointCloudSearch<pcl::PointWithRange> octree (resolution);

  octree.setInputCloud (linemesh.cloud_vertices);
  octree.addPointsFromInputCloud ();
    
  //线段相连
  pcl::PointWithRange pt1,pt2;
  for (size_t i = 0; i < linemesh.cloud_vertices->points.size(); ++i)
  {  
    int num_end=0;
    double x_diff = 0, y_diff = 0, z_diff = 0,dist=999; 
    pt1=linemesh.cloud_vertices->points[i];
    for (size_t j = i+1; j < linemesh.cloud_vertices->points.size(); ++j)
    {
       pt2=linemesh.cloud_vertices->points[j];
       x_diff = pt1.x- pt2.x;  
       y_diff = pt1.x- pt2.x;
       z_diff = pt1.y- pt2.y;
       if(sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff)<dist)
       {
	 dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff); 
	 num_end=j;	 
       }
    }
    //裁剪
//     pcl::PointXYZRGB p3,p4;
//     pcl::BoxClipper3D<pcl::PointXYZRGB> cp;
//     cp.setTransformation();
    
  
    //pcl::PCLPointCloud2::Ptr cloud_filter(new pcl::PCLPointCloud2);
    //clip.setInputCloud (cloud_projected);
    //cp.clipLineSegment3D(pt1,pt2);
    //cp.clipLineSegment3D(pt1,pt2);
    //clip.filter (*output_cloud);
    //std::cout << "PointCloud after filtering VoxelGrid has: " << output_cloud->width  << " data points." << std::endl; 
    //plywriter.write("cloud_preprocessed.ply",*output_cloud,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);

    if(dist<0.50)
    {
      std::cout<<dist<<" "<<i<<" "<<num_end<<std::endl;
      pcl::Vertices temp;
      temp.vertices.push_back(i),temp.vertices.push_back(num_end);
      linemesh.polylines.push_back(temp); 
    }
  }
    
  std::cout << "dlg_segments: " << linemesh.cloud_vertices->points.size() << " vertices," << linemesh.polylines.size()<<" segments." <<std::endl;
  fn+="_dlg_segments.obj";
  pcl::io::saveOBJFile3(fn, linemesh);
  //pcl::io::saveOBJFile3("dlg_segments2.obj", linemesh);
  
  return (0);
}
