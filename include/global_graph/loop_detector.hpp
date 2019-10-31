#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <boost/format.hpp>
#include <global_graph/keyframe.hpp>
#include <global_graph/registrations.hpp>
#include <global_graph/graph_slam.hpp>

#include <g2o/types/slam3d/vertex_se3.h>

#include <DBoW3/DBoW3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;

namespace lv_slam {

struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose)
    : key1(key1),
      key2(key2),
      relative_pose(relpose)
  {}

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector(ros::NodeHandle& pnh) {
    distance_thresh = pnh.param<double>("distance_thresh", 5.0);
    accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
    distance_from_last_edge_thresh = pnh.param<double>("min_edge_interval", 5.0);

    fitness_score_max_range = pnh.param<double>("fitness_score_max_range", std::numeric_limits<double>::max());
    fitness_score_thresh = pnh.param<double>("fitness_score_thresh", 0.5);

    registration = select_registration_method(pnh);
    last_edge_accum_distance = 0.0;
    
    //loop dectection param
    voc_path=pnh.param<std::string>("voc_path", "/home/whu/slam_ws/src/lv_slam/config/vocab_larger.yml.gz");
    voc = new DBoW3::Vocabulary(voc_path);
    if(voc->empty())
    {
      std::cout<<"Vocabulary dose not exit."<<std::endl;
      return;
    }
  }

  /**
   * @brief detect loops and add them to the pose graph
   * @param keyframes       keyframes
   * @param new_keyframes   newly registered keyframes
   * @param graph_slam      pose graph
   */
  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes, lv_slam::GraphSLAM& graph_slam) {
    std::vector<Loop::Ptr> detected_loops;
    for(const auto& new_keyframe : new_keyframes) {
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching_and_bow(candidates, new_keyframe, graph_slam);
      if(loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  double get_distance_thresh() const {
    return distance_thresh;
  }

private:
  /**
   * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
   * @param keyframes      candidate keyframes of loop start
   * @param new_keyframe   loop end keyframe
   * @return loop candidates
   */
  std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if(new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh) {
      return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);//预留空间

    for(const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if(new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if(dist > distance_thresh) {
        continue;
      }

      candidates.push_back(k);
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe, lv_slam::GraphSLAM& graph_slam) {
    if(candidate_keyframes.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    std::cout << "matching" << std::flush;
    auto t1 = ros::Time::now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(const auto& candidate : candidate_keyframes) {
      registration->setInputSource(candidate->cloud);
      Eigen::Matrix4f guess = (new_keyframe->node->estimate().inverse() * candidate->node->estimate()).matrix().cast<float>();
      guess(2, 3) = 0.0;
      registration->align(*aligned, guess);
      std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if(!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;   //取最低匹配误差（得分）
      best_matched = candidate;
      relative_pose = registration->getFinalTransformation();
    }

    auto t2 = ros::Time::now();
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

    if(best_score > fitness_score_thresh) {
      std::cout << "loop not found..." << std::endl;
      return nullptr;
    }

    std::cout << "loop found!!" << std::endl;//block 表示返回从矩阵的(i, j)开始，每行取p个元素，每列取q个元素
    std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }
 
   /**
   * @brief To validate a loop candidate this function applies img bow loop detection and a scan matching between keyframes. If true, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching_and_bow(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe, lv_slam::GraphSLAM& graph_slam) {
    if(candidate_keyframes.empty()) {
      std::cout << "no candidate, then loop not found..." << std::endl;
      return nullptr;
    }
      
    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    auto t1 = ros::Time::now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    DBoW3::Database db_temp;
    db_temp.setVocabulary(*voc, false, 0);
    for(const auto& candidate : candidate_keyframes) {
      db_temp.add(candidate->descriptor);
    }
    DBoW3::QueryResults ret;
    db_temp.query(new_keyframe->descriptor,ret,5);
    std::cout<<candidate_keyframes.size()<<"loop dectection for "<<new_keyframe->seq<<std::endl<<std::endl;
    
    for(int i=0;i<ret.size();i++)
    {
      if(ret[i]<0.05||best_score<=fitness_score_thresh) break;
      std::cout<<ret[i]<<ret[i].Id<<std::endl;
      int best_id=ret[0].Id;
      best_matched=candidate_keyframes[best_id];
      registration->setInputSource(best_matched->cloud);
      Eigen::Matrix4f guess = (new_keyframe->node->estimate().inverse() * best_matched->node->estimate()).matrix().cast<float>();
      guess(2, 3) = 0.0;
      registration->align(*aligned, guess);
      std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if(!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;   //取最低匹配误差（得分）
      relative_pose = registration->getFinalTransformation();
    }
 
    if(best_score > fitness_score_thresh) {
      std::cout << "loop not found..." << std::endl;
      return nullptr;
    }
    
    auto t2 = ros::Time::now();
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

    std::cout << "loop found!!" << std::endl;//block 表示返回从矩阵的(i, j)开始，每行取p个元素，每列取q个元素
    std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }
  
private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance

  double fitness_score_max_range;         // maximum allowable distance between corresponding points
  double fitness_score_thresh;            // threshold for scan matching

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
  
  DBoW3::Database db;
  DBoW3::Vocabulary* voc;
  std::string voc_path;
};

}

#endif // LOOP_DETECTOR_HPP
