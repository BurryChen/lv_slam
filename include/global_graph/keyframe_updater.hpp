#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <sys/types.h>
#include <string>
#include <vector>
#include <fstream>
using namespace std;

namespace lv_slam
{

  /**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
  class KeyframeUpdater
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * @brief constructor
   * @param pnh
   */
    KeyframeUpdater(ros::NodeHandle &pnh)
        : is_first(true),
          prev_keypose(Eigen::Isometry3d::Identity())
    {
      keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 2.0);
      keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 2.0);

      accum_distance = 0.0;

      //semantic label file read
      label_path = pnh.param<std::string>("label_path", "$(find lv_slam)/data/05_label.txt");
      std::cout << "label_path=" << label_path << std::endl;
      std::ifstream inFile;
      inFile.open(label_path, ios::in);
      if (!inFile.is_open())
      {
        cout << "label_path open failed" << endl;
      }
      std::string line;
      while (getline(inFile, line))
      {
        //cout <<"原始字符串："<< line << endl; //整行输出
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        vector<string> fields;   //声明一个字符串向量
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
          fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        double time = stod(fields[0].c_str());
        int label = atoi(fields[1].c_str());
        // printf("%f %d \n", time, label);
        label_frames.push_back(make_pair(time, label));
      }
      cout << "KeyframeUpdater label_frames done!" << endl;
    }

    /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
    bool update(const Eigen::Isometry3d &pose)
    {
      // first frame is always registered to the graph
      if (is_first)
      {
        is_first = false;
        prev_keypose = pose;
        return true;
      }

      // calculate the delta transformation from the previous keyframe
      Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
      double dx = delta.translation().norm();
      double da = std::acos(Eigen::Quaterniond(delta.linear()).w());

      // too close to the previous frame
      if (dx < keyframe_delta_trans && da < keyframe_delta_angle)
      {
        return false;
      }

      accum_distance += dx;
      prev_keypose = pose;
      return true;
    }

    bool update(const Eigen::Isometry3d &pose, const ros::Time &stamp)
    {
      // first frame is always registered to the graph
      if (is_first)
      {
        is_first = false;
        prev_keypose = pose;
        return true;
      }
      // calculate the delta transformation from the previous keyframe
      Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
      double dx = delta.translation().norm();
      double da = std::acos(Eigen::Quaterniond(delta.linear()).w());

      for (auto label_frame : label_frames)
      {
        if (std::abs(stamp.toSec() - label_frame.first) < 0.05)
        {
          printf("-------------------------------------------------KeyframeUpdater%f %f \n",
                 stamp.toSec(), label_frame.first);
          accum_distance += dx;
          prev_keypose = pose;
          return true;
        }
      }

      // too close to the previous frame
      if (dx < keyframe_delta_trans && da < keyframe_delta_angle)
      {
        return false;
      }

      accum_distance += dx;
      prev_keypose = pose;
      return true;
    }

    /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
    double get_accum_distance() const
    {
      return accum_distance;
    }

  private:
    // parameters
    double keyframe_delta_trans; //
    double keyframe_delta_angle; //

    //bool is_first;
    double accum_distance;
    Eigen::Isometry3d prev_keypose;

  public:
    bool is_first;

    std::string label_path;
    std::vector<std::pair<double, int>> label_frames;
  };

}

#endif // KEYFRAME_UPDATOR_HPP
