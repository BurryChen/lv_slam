#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
/****************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * **************************************************/

int find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

void pose_estimation_2d2d (
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Eigen::Matrix3d K,
    Mat& R, Mat& t );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Eigen::Matrix3d& K );

int find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& good_matches )
{
  //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
  Ptr<SIFT> detector = SIFT::create();
  //detector->setHessianThreshold(minHessian);
  //std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
  detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );
  //-- Step 2: Matching descriptor vectors using FLANN matcher
  BFMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );
  double min_dist = matches[0].distance, max_dist = matches[0].distance;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  std::cout<<"-- Max dist :"<<std::endl<<max_dist<<std::endl;
  std::cout<<"-- Min dist :"<<std::endl<<min_dist<<std::endl;
  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  //std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(4*min_dist, 60.0) )
    { good_matches.push_back( matches[i]); }
  }
  //for( int i = 0; i < (int)good_matches.size(); i++ )
  //{ printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
}


Point2d pixel2cam ( const Point2d& p, const Eigen::Matrix3d& K )
{
    return Point2d
           (
               ( p.x - K( 0,2 ) ) / K( 0,0 ),
               ( p.y - K( 1,2 ) ) / K( 1,1 )
           );
}


void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
			    Eigen::Matrix3d K,
                            Mat& R, Mat& t )
{
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    //fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    //fundamental_matrix = findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
    //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( K(0,2), K(1,2) );     	//相机光心, 
    double focal_length = K(1,1);			//相机焦距, 
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point);
    //cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    //cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    //cout<<"R is "<<endl<<R<<endl;
    //cout<<"t is "<<endl<<t<<endl;
    
}

// 对极几何估计位姿
bool reg_epipolar_geometry ( string source_img,string target_img, Eigen::Matrix4d tf_velo2cam2,Eigen::Matrix3d K,Eigen::Matrix4d& pose_velo )
{
    //-- 读取图像
    Mat img_1 = imread ( source_img, CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( target_img, CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    
    //画关键点
    //Mat img_keypoints_1, img_keypoints_2;
    //drawKeypoints(img_1,keypoints_1,img_keypoints_1,Scalar::all(-1),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //imshow(source_img,img_keypoints_1);
    //imshow(target_img,img_keypoints_2);
    cout<<"一共找到了good_matches "<<matches.size() <<"组匹配点"<<endl;
    //if(matches.size()<100) return 0;
    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
     //-- Show detected matches
    imshow( source_img+" to "+target_img+" Good Matches", img_matches ); 
    
    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, K, R, t );

    //-- 验证E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );

    //cout<<"R="<<endl<<R<<endl;
    //cout<<"t="<<endl<<t<<endl;
    //cout<<"t^R="<<endl<<t_x*R<<endl;
    
    //-- 验证对极约束,
    for ( DMatch m: matches )
    {
      //类似探元指向角
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
	cout << "epipolar constraint = " << d*K(0,0)<<keypoints_1[ m.queryIdx ].pt<<keypoints_2[ m.trainIdx ].pt<< endl;
    }
        
    Eigen::Matrix3d R_cam2,R_velo;
    R_cam2<<R.at<double> ( 0,0 ),R.at<double> ( 0,1 ),R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ),R.at<double> ( 1,1 ),R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ),R.at<double> ( 2,1 ),R.at<double> ( 2,2 );
    R_velo=tf_velo2cam2.block<3,3>(0,0).inverse()*R_cam2*tf_velo2cam2.block<3,3>(0,0);
    Sophus::SO3 SO3_velo(R_velo); 
    Eigen::Matrix<double, 3, 1> p_new=SO3_velo.log();
    Sophus::SE3 SE3_s2k(pose_velo.block(0,0,3,3),pose_velo.block(0,3,3,1));
    Eigen::Matrix<double, 6, 1> p=SE3_s2k.log();
    waitKey(0);
    //差异小于0.01rad（0.57°），认为有效
    std::cout<<"p "<<p.transpose()<<std::endl;
    std::cout<<"p_new: "<<p_new.transpose()<<std::endl;
    if(p_new(1,0)-p(4,0)<0&&p_new(1,0)-p(4,0)>-0.005)
    {
      p(4,0)=p_new(1,0);
      pose_velo=Sophus::SE3::exp(p).matrix();
      return 1;
    }
    else 
      return 0;

    //cv::destroyWindow(source_img+" to "+target_img+" Good Matches");
    
};
