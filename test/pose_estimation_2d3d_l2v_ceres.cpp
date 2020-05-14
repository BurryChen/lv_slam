#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <chrono>

#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <iostream>
#include <sys/types.h>
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 
#include <unistd.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
using namespace cv;
using namespace Eigen;

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

// 世界坐标转像素坐标
Point2d world2pixel ( const Point3d& p, const Mat& K, const Mat& R, const Mat& t);

void bundleAdjustment_2d3d(
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t,Matrix<double,6,6> &Cov
);


int main ( int argc, char** argv )
{   
    //string workdir="./data/calib";
    string workdir=argv[1];
    chdir(workdir.c_str());
   
    ifstream inFile; 
    inFile.open("corresponces_2d3d.csv",ios::in);
    string line; 
    //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取 ,第一行跳过
    //getline(inFile, line);
    int id=0;
    vector<Point3f> pts_3d,pts_3d_check;
    vector<Point2f> pts_2d,pts_2d_check;
    int num=0;
    while (getline(inFile, line))   
    {  
        //cout <<"原始字符串："<< line << endl; //整行输出  
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中  
        vector<string> fields; //声明一个字符串向量  
        string field;  
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
        {  
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
        }  
        float x=stof(fields[1].c_str()),y = atof(fields[2].c_str());
	float X=stof(fields[3].c_str()),Y = atof(fields[4].c_str()),Z = atof(fields[5].c_str());

	Point2f p1(x,y);
        Point3f p2(X,Y,Z);
	num++;
	
	if(num%3!=3)
	{pts_2d.push_back ( p1);  pts_3d.push_back ( p2); }
        else
	{pts_2d_check.push_back ( p1);  pts_3d_check.push_back ( p2);} 
	
        //cout <<"correspondence-"<<id++<<":\t"<< p1 << "\t" << p2  << endl; 
    }
    Mat K = ( Mat_<double> ( 3,3 ) << 2814.26885863355, 0, 2003.80380836663, 0, 2804.58176740019, 1450.29021239786, 0, 0, 1);
    
    Mat r, t;
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;
    
    ofstream outFile;
    outFile.open("result.log", ios::out);  
    outFile<<" PNP results: "<<endl;
    outFile<<"R = "<<R<<endl;
    outFile<<"t = "<<t<<endl;
    Matrix<double,6,6> Cov;
    bundleAdjustment_2d3d( pts_3d, pts_2d, K,R,t,Cov);
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;
    
    outFile<<endl<<" Ceres bundleAdjustment results: "<<endl;
    outFile<<"R = "<<R<<endl;
    outFile<<"t = "<<t<<endl;
    outFile<<"R_inv = "<<R.t() <<endl;
    outFile<<"t_inv = "<<-R.t() *t<<endl;
    
    //2.residual_control: // verify pts_2d = R*p2 + t
    
    outFile<<"-----------------------residual_control-----------------------------"<<endl;
    ofstream resFile;
    resFile.open("residual_control.csv", ios::out);
    vector<double> res[3];
    double VTV=0;
    for ( int i=0; i<pts_2d.size(); i++ )
    {
        Point2d pts_3d2pixel=world2pixel(pts_3d[i],K,R,t);
	Point2d error_proj(pts_3d2pixel.x-pts_2d[i].x ,pts_3d2pixel.y-pts_2d[i].y);
        
	/*cout<<"pts_2d = "<<pts_2d[i]<<endl;
        cout<<"pts_3d = "<<pts_3d[i]<<endl;
        cout<<"pts_3d2pixel = "<< pts_3d2pixel <<endl;	    
	cout<<"error_proj = "<< error_proj<<endl;
        cout<<endl;*/
	
	outFile<<"pts_2d = "<<pts_2d[i]<<endl;
        outFile<<"pts_3d = "<<pts_3d[i]<<endl;
        outFile<<"pts_3d2pixel = "<< pts_3d2pixel <<endl;	    
	outFile<<"error_proj = "<< error_proj<<endl;
        outFile<<endl;
	
	resFile<<
	pts_2d[i].x<<","<<pts_2d[i].y<<","<<error_proj.x<<","<<error_proj.y<<","<<i<<endl
	<<pts_2d[i].x+20*error_proj.x<<","<<pts_2d[i].y+20*error_proj.y<<endl<<endl;
	//error_proj.x<<","<<error_proj.y<<","<<norm(error_proj)<<endl;
	//fprintf(fp, "%lf\t%lf\n%lf\t%lf\n\n", m_Xtmp[i], m_Ytmp[i], m_Xtmp[i] + 50 * dx, m_Ytmp[i]+50*dy);
	res[0].push_back(error_proj.x);
	res[1].push_back(error_proj.y);
	res[2].push_back(norm(error_proj));
	VTV+=error_proj.x*error_proj.x+error_proj.y*error_proj.y;
    }
    
    double mean[3],rmse[3];
    for(int i=0;i<3;i++)
    {
      double sum = std::accumulate(std::begin(res[i]), std::end(res[i]), 0.0);
      mean[i] =  sum / res[i].size(); //均值
      double accum  = 0.0;  
      for (vector<int>::size_type j = 0; j != res[i].size(); j ++){
      accum  += res[i][j]*res[i][j];   
      }
      rmse[i] = sqrt(accum/(res[i].size()-1)); //RMS 
    }
    
    cout<<"mean"<<endl;;
    cout<<mean[0]<<","<<mean[1]<<","<<mean[2]<<endl;
    cout<<"rmse"<<endl;;
    cout<<rmse[0]<<","<<rmse[1]<<","<<rmse[2]<<endl; 
    
    outFile<<"mean"<<endl;;
    outFile<<mean[0]<<","<<mean[1]<<","<<mean[2]<<endl;
    outFile<<"rmse"<<endl;;
    outFile<<rmse[0]<<","<<rmse[1]<<","<<rmse[2]<<endl;
    
    double sigma2=VTV/(2*pts_2d.size()-6);
    double sigma=sqrt(sigma2);
    outFile<<endl<<"VTV="<<VTV<<endl;
    outFile<<"sigma="<<sqrt(sigma2)<<"\nsigma2="<<sigma2<<endl;
    //精度评定
    outFile<<"Covariance=\n"<<Cov<<endl;
    outFile<<"D=sigma2*Cov=\n"<<sigma2*Cov<<endl;
    outFile<<"sigma_X=\n";
    for ( int i=0; i<6; i++ )
    {
      outFile<<sigma*sqrt(Cov(i,i))<<endl;
    }
    outFile<<"sigma_angle=\n";
    for ( int i=0; i<3; i++ )
    {
      outFile<<sigma*sqrt(Cov(i,i))*180/3.1415926<<endl;
    }
    
    outFile<<"-----------------------residual_check----------------------------"<<endl;
    //检查点残差
    ofstream resFile2;
    resFile2.open("residual_check.csv", ios::out);  
    vector<double> res2[3];
    for ( int i=0; i<pts_2d_check.size(); i++ )
    {
        Point2d pts_3d2pixel=world2pixel(pts_3d_check[i],K,R,t);
	Point2d error_proj(pts_3d2pixel.x-pts_2d_check[i].x ,pts_3d2pixel.y-pts_2d_check[i].y);
        
	/*cout<<"pts_2d_check = "<<pts_2d_check[i]<<endl;
        cout<<"pts_3d_check = "<<pts_3d_check[i]<<endl;
        cout<<"pts_3d_check2pixel = "<< pts_3d2pixel <<endl;	    
	cout<<"error_proj = "<< error_proj<<endl;
        cout<<endl;*/
	
	outFile<<"pts_2d_check = "<<pts_2d_check[i]<<endl;
        outFile<<"pts_3d_check = "<<pts_3d_check[i]<<endl;
        outFile<<"pts_3d2pixel = "<< pts_3d2pixel <<endl;	    
	outFile<<"error_proj = "<< error_proj<<endl;
        outFile<<endl;
	
	resFile2<<
	//error_proj.x<<","<<error_proj.y<<","<<norm(error_proj)<<endl;
	pts_2d_check[i].x<<","<<pts_2d_check[i].y<<","<<error_proj.x<<","<<error_proj.y<<endl
	<<pts_2d_check[i].x+20*error_proj.x<<","<<pts_2d_check[i].y+20*error_proj.y<<endl<<endl;
	res2[0].push_back(error_proj.x);
	res2[1].push_back(error_proj.y);
	res2[2].push_back(norm(error_proj));
    }
    
    for(int i=0;i<3;i++)
    {
      double sum = std::accumulate(std::begin(res2[i]), std::end(res2[i]), 0.0);
      mean[i] =  sum / res2[i].size(); //均值
      double accum  = 0.0;  
      for (vector<int>::size_type j = 0; j != res2[i].size(); j ++){
      accum  += res2[i][j]*res2[i][j];   
      }
      rmse[i] = sqrt(accum/(res2[i].size()-1)); //RMS 
    }
    
    cout<<"mean"<<endl;;
    cout<<mean[0]<<","<<mean[1]<<","<<mean[2]<<endl;
    cout<<"rmse"<<endl;;
    cout<<rmse[0]<<","<<rmse[1]<<","<<rmse[2]<<endl; 
    
    outFile<<"mean"<<endl;;
    outFile<<mean[0]<<","<<mean[1]<<","<<mean[2]<<endl;
    outFile<<"rmse"<<endl;;
    outFile<<rmse[0]<<","<<rmse[1]<<","<<rmse[2]<<endl;
   
    outFile<<endl;
    resFile2<<endl;   
    
    return 1;   
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

// 世界坐标转像素坐标
Point2d world2pixel ( const Point3d& p, const Mat& K, const Mat& R, const Mat& t)
{
  Mat KTP=K*(R * (Mat_<double>(3,1)<<p.x, p.y, p.z) + t);
  return Point2d(KTP.at<double>(0,0) / KTP.at<double>(0,2),KTP.at<double>(0,1) / KTP.at<double>(0,2));
}

// 代价函数,res=p_2d-K*T*p_3d
// 已知量：p_3d(u,v);p_3d(x,y,z);
// 待估参数：pose
struct CORRESPONDING_2D3D_COST
{
    CORRESPONDING_2D3D_COST (double u,double v,
			double x,double y,double z) 
    : _u(u),_v(v),_x(x),_y(y),_z(z) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const pose,
        T* residual ) const     //  残差   2维
    {	
        //RES;
	T P[3],TP[3],KTP[3],R2[9];
	P[0]=T(_x),P[1]=T(_y),P[2]=T(_z);
        // Rodrigues' formula
        ceres::AngleAxisRotatePoint(pose, P, TP);
        TP[0] += pose[3]; TP[1] += pose[4]; TP[2] += pose[5];
        // 归一化
        KTP[0]=K[0]*TP[0]+K[1]*TP[1]+K[2]*TP[2];
	KTP[1]=K[3]*TP[0]+K[4]*TP[1]+K[5]*TP[2];
	KTP[2]=K[6]*TP[0]+K[7]*TP[1]+K[8]*TP[2];
        T predicted_u = KTP[0] / KTP[2];
        T predicted_v = KTP[1] / KTP[2];
	residual[0] =  predicted_u-_u ;
        residual[1] =  predicted_v-_v ;
        return true;
    }
    const double _u,_v,_x,_y,_z; 
    const double K[9]={2814.26885863355, 0, 2003.80380836663, 0, 2804.58176740019, 1450.29021239786, 0, 0, 1};
};


void bundleAdjustment_2d3d (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t,Matrix<double,6,6> &Cov)
{
    Eigen::Matrix4d  T;
    T<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
       R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
       R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0),
       0,0,0,1;
    /*double R2[9]={R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                  R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                  R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2)};
    double ang[3];
    ceres::RotationMatrixToAngleAxis(R2,ang);*/  //为什么是负的？
    Eigen::Matrix<double, 6, 1> se3;
      // T(R,t)->李群SE(3)->李代数se(3)
    Sophus::SE3 SE3_Rt(T.block(0,0,3,3).cast<double>(),T.block(0,3,3,1).cast<double>());
    //平移在前，旋转在后
    se3=SE3_Rt.log();
    //旋转在前，平移在后,求导函数中R用旋转向量，t并没有换算为Jp
    double pose[6]={se3(3,0),se3(4,0),se3(5,0),t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0)};
   
    //build problem
    ceres::Problem problem;
    for ( int i=0; i<points_2d.size(); i++ )
    {
        double p_2d[2],p_3d[3];
	p_2d[0]=points_2d[i].x;p_2d[1]=points_2d[i].y;
	p_3d[0]=points_3d[i].x;p_3d[1]=points_3d[i].y;p_3d[2]=points_3d[i].z;
	//代价函数,优化参数pose
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，误差项维度，优化参数维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CORRESPONDING_2D3D_COST, 2, 6> ( 
                new CORRESPONDING_2D3D_COST (p_2d[0],p_2d[1],p_3d[0],p_3d[1],p_3d[2])
            ),
            nullptr,            // 核函数，这里不使用，为空
	    pose
        );	
    }
            
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_SCHUR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果,旋转向量，平移量
    cout<<summary.FullReport() <<endl;
    cout<<"estimated pose= ";
    for(auto a:pose) cout<<a<<" ";
    cout<<endl;
    
    Eigen::Matrix<double, 6, 1> se3_new;
    se3_new<<pose[3],pose[4],pose[5],pose[0],pose[1],pose[2];
    T=Sophus::SE3::exp(se3_new).matrix().cast<double>();
    T(0,3)=pose[3],T(1,3)=pose[4],T(2,3)=pose[5];
    cout<<"T="<<endl<<T<<endl;
    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) << T(0,0),T(0,1),T(0,2),
	                          T(1,0),T(1,1),T(1,2),
				  T(2,0),T(2,1),T(2,2));
    t = ( Mat_<double> ( 3,1 ) << pose[3],pose[4],pose[5]);       
    
    //output covariance
    ceres::Covariance::Options options2;
    options2.algorithm_type=ceres::DENSE_SVD;
    options2.apply_loss_function=false;
    options2.min_reciprocal_condition_number=1e-5;
    ceres::Covariance covariance(options2);

    vector<pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back(make_pair(pose, pose));

    CHECK(covariance.Compute(covariance_blocks, &problem));

    double covariance_posepose[6 * 6];
    covariance.GetCovarianceBlock(pose, pose, covariance_posepose);   
    
    //协因数阵及法方程系数阵
    Eigen:Map<Matrix<double,6,6> > Covtemp(covariance_posepose);
    Cov=Covtemp;
}
