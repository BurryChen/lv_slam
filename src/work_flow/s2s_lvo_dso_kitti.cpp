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

#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/gicp_omp.h>

#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace Eigen;
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace g2o;
using namespace cv;

typedef pcl::PointXYZI PointT;

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

// 点云投影到图像，获取对应像素值
vector<Measurement> project_cloud2img(const pcl::PointCloud<PointT>::Ptr& cloud,cv::Mat gray,Eigen::Matrix4d T,Eigen::Matrix3d K)
{
  vector<Measurement> measurements;
  for ( int j=0; j<cloud->size(); j++ )
    {
      Eigen::Vector4d p3d_velo(cloud->points[j].x,cloud->points[j].y,cloud->points[j].z,1);
      // body（cam）系下3d点
      Eigen::Vector4d p3d_body=T*p3d_velo;
      double u,v;
      u=K(0,0)*p3d_body[0] / p3d_body[2]+K(0,2);
      v=K(1,1)*p3d_body[1] / p3d_body[2]+K(1,2);
      // 去掉邻近边缘处的点
      if ( u < 1 || v < 1 || ( u+1 ) >gray.cols || ( v+1 ) >gray.rows )
	continue;
      float grayscale = float ( gray.ptr<uchar> ( cvRound ( v ) ) [ cvRound ( u ) ] );
      measurements.push_back ( Measurement ( p3d_body.block<3,1>(0,0), grayscale ) );
    }
    return measurements;
}

inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}

// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参； 输出：相机位姿
// 返回：true为成功，false失败
bool poseEstimationDirect ( const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3d& intrinsics, Eigen::Isometry3d& Tcw );

// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect ( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image )
        : x_world_ ( point ), fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), image_ ( image )
    {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v  =static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d x_local = v->estimate().map ( x_world_ );
        float x = x_local[0]*fx_/x_local[2] + cx_;
        float y = x_local[1]*fy_/x_local[2] + cy_;
        // check x,y is in the image
        if ( x-1<0 || ( x+1 ) >image_->cols || ( y-1 ) <0 || ( y+1 ) >image_->rows )
        {
            _error ( 0,0 ) = 0.0;
            this->setLevel ( 1 );
        }
        else
        {
            _error ( 0,0 ) = getPixelValue ( x,y ) - _measurement;
        }
    }

    // plus in manifold
    virtual void linearizeOplus( )
    {
        if ( level() == 1 )
        {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d xyz_trans = vtx->estimate().map ( x_world_ );   // q in book

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        float u = x*fx_*invz + cx_;
        float v = y*fy_*invz + cy_;

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx_;
        jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
        jacobian_uv_ksai ( 0,2 ) = - y*invz *fx_;
        jacobian_uv_ksai ( 0,3 ) = invz *fx_;
        jacobian_uv_ksai ( 0,4 ) = 0;
        jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx_;

        jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 ) *fy_;
        jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy_;
        jacobian_uv_ksai ( 1,2 ) = x*invz *fy_;
        jacobian_uv_ksai ( 1,3 ) = 0;
        jacobian_uv_ksai ( 1,4 ) = invz *fy_;
        jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
        jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

        _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;
    }

    // dummy read and write functions because we don't care...
    virtual bool read ( std::istream& in ) {}
    virtual bool write ( std::ostream& out ) const {}

protected:
    // get a gray scale value from reference image (bilinear interpolated)
    inline float getPixelValue ( float x, float y )
    {
        uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
        float xx = x - floor ( x );
        float yy = y - floor ( y );
        return float (
                   ( 1-xx ) * ( 1-yy ) * data[0] +
                   xx* ( 1-yy ) * data[1] +
                   ( 1-xx ) *yy*data[ image_->step ] +
                   xx*yy*data[image_->step+1]
               );
    }
public:
    Eigen::Vector3d x_world_;   // 3D point in world frame
    float cx_=0, cy_=0, fx_=0, fy_=0; // Camera intrinsics
    cv::Mat* image_=nullptr;    // reference image
};

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
  vector<Measurement> measurements;
  
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
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<PointT, PointT>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
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
  int step=1;
  Eigen::Matrix4d tf_s2s=Eigen::Matrix4d::Identity();
  tf_s2s(0,3)=1.5;
  Eigen::Matrix4d tf_s2s_cam=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_s2s_cam2=Eigen::Matrix4d::Identity();
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
    std::string source_pcd = namelist[i+step]->d_name;
    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
      std::cerr << "failed to load " << target_pcd << std::endl;
      return 0;
    }
    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
      std::cerr << "failed to load " << source_pcd << std::endl;
      return 0;
    }
    
    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    *source_cloud = *downsampled;

    distance_filted=distance_filter(target_cloud,1,50);
    *target_cloud = *distance_filted;
    distance_filted=distance_filter(source_cloud,1,50);
    *source_cloud=*distance_filted;
  
    ndt_omp->setInputTarget(target_cloud);
    ndt_omp->setInputSource(source_cloud);
    auto t1 = ros::WallTime::now();
    ndt_omp->align(*matched,tf_s2s.cast<float>());
    auto t2 = ros::WallTime::now();
    std::cout <<i<<"  "<<target_pcd<<" /"<<source_pcd<< " t: " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
    tf_s2s=ndt_omp->getFinalTransformation().cast<double>();
    tf_s2s_cam=tf_velo2cam0*tf_s2s*tf_velo2cam0.inverse();
    std::cout<<"tf_s2s: \n"<<tf_s2s<<std::endl;
    std::cout<<"tf_s2s_cam: \n"<<tf_s2s_cam<<std::endl;
    
    //elevation 
    Eigen::Matrix4d tf_s2s_gt=poses_cam[i].inverse()*poses_cam[i+step];
    std::cout<<"tf_s2s_gt: \n"<<tf_s2s_gt<<std::endl;
    tf_s2s_error=tf_s2s_gt.inverse()*tf_s2s_cam;
    Sophus::SE3 SE3_Rt(tf_s2s_error.block(0,0,3,3),tf_s2s_error.block(0,3,3,1));
    std::cout<<"tf_s2s_error: "<<SE3_Rt.log().transpose()<<std::endl;
    
    //refine with direct matching in cam2
    char temp[7];
    sprintf(temp,"%06d",i+step);
    string temp2=imgsdir+"/"+temp+".png";
    color= cv::imread ( temp2 ,1);
    if ( color.data==nullptr )
      continue; 
    cv::cvtColor ( color, gray, cv::COLOR_BGR2GRAY );
    measurements=project_cloud2img(source_cloud,gray,tf_cam0tocam2*tf_velo2cam0,K);
    // 使用直接法计算相机运动
    source_color = color.clone();
    sprintf(temp,"%06d",i);
    temp2=imgsdir+"/"+temp+".png";
    color= cv::imread ( temp2 ,1);
    if ( color.data==nullptr )
      continue; 
    cv::cvtColor ( color, gray, cv::COLOR_BGR2GRAY );
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    Eigen::Isometry3d Tcw(tf_cam0tocam2*tf_s2s_cam*tf_cam0tocam2.inverse());
    poseEstimationDirect ( measurements, &gray, K, Tcw );
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t4-t3 );
    cout<<"direct method costs time: "<<time_used.count() <<" seconds."<<endl;
    cout<<"Tcw="<<Tcw.matrix() <<endl;	
    tf_s2s_cam=tf_cam0tocam2.inverse()*Tcw.matrix()*tf_cam0tocam2;
    //elevation 
    tf_s2s_error=tf_s2s_gt.inverse()*tf_s2s_cam;
    Sophus::SE3 SE3_Rt2(tf_s2s_error.block(0,0,3,3),tf_s2s_error.block(0,3,3,1));
    std::cout<<"tf_s2s_error2: "<<SE3_Rt2.log().transpose()<<std::endl;
    
    // plot the feature points
    cv::Mat img_show ( color.rows*2, color.cols, CV_8UC3 );
    source_color.copyTo ( img_show ( cv::Rect ( 0,0,color.cols, color.rows ) ) );
    color.copyTo ( img_show ( cv::Rect ( 0,color.rows,color.cols, color.rows ) ) );
    int k=0;
    for ( Measurement m:measurements )
    {
      k++;
      if(k%50!=0)continue;
      if ( rand() > RAND_MAX/5 )
	continue;
      Eigen::Vector3d p = m.pos_world;
      Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), K(0,0), K(1,1), K(0,2), K(1,2) );
      Eigen::Vector3d p2 = Tcw*m.pos_world;
      Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), K(0,0), K(1,1), K(0,2), K(1,2) );
      if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color.rows )
	continue;

      float b = 255*float ( rand() ) /RAND_MAX;
      float g = 255*float ( rand() ) /RAND_MAX;
      float r = 255*float ( rand() ) /RAND_MAX;
      cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 2 );
      cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), 8, cv::Scalar ( b,g,r ), 2 );
      cv::line ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), cv::Scalar ( b,g,r ), 1 );
    }
    cv::imshow ( "result", img_show );
    cv::waitKey ( 1 );
    
    odom=odom*tf_s2s_cam;
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

bool poseEstimationDirect ( const vector< Measurement >& measurements, cv::Mat* gray, Eigen::Matrix3d& K, Eigen::Isometry3d& Tcw )
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( true );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // 添加边
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
            m.pos_world,
            K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), gray
        );
        edge->setVertex ( 0, pose );
        edge->setMeasurement ( m.grayscale );
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        edge->setId ( id++ );
        optimizer.addEdge ( edge );
    }
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    Tcw = pose->estimate();
}