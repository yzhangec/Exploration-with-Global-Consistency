#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <memory>
#include <mutex>
#include <queue>
#include <random>

using std::default_random_engine;
using std::normal_distribution;
using std::queue;
using std::shared_ptr;

namespace fast_planner {
class SDFMap;

class MapROS {
 public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap *map);
  void init();

 private:
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void depthPoseCorrectCallback(const sensor_msgs::ImageConstPtr &img,
                                const visualization_msgs::MarkerConstPtr &pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose);
  void poseDriftCallback(const geometry_msgs::PoseStampedConstPtr &pose);
  void poseGroundTruthCallback(const visualization_msgs::MarkerConstPtr &pose);
  void poseLoopClosurePathCallback(const nav_msgs::Path &path);
  void poseOptimizationTriggerCallback(const nav_msgs::Path &msg);
  void extrinsicCallback(const nav_msgs::Odometry::ConstPtr &pose_msg);
  void cloudGtCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void updateESDFCallback(const ros::TimerEvent & /*event*/);
  void reintegrateCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);

  void publishOccu();
  void publishOccuLocal();
  void publishESDF();
  void publishTSDF();
  void publishTSDFSlice();
  void publishTSDFOccu();
  void publishUpdateRange();
  void publishUnknown();
  void publishDepth(const std_msgs::Header &head);
  void publishALCCandidates();
  void publishLoopClosurePoints();

  void processDepthImage();
  void processDepthImageCorrect(const Eigen::Vector3d &camera_pos_correct,
                                const Eigen::Quaterniond &camera_q_correct,
                                pcl::PointCloud<pcl::PointXYZ> &point_cloud_correct);

  SDFMap *map_;
  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          visualization_msgs::Marker>
      SyncPolicyImagePoseCorrect;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePoseCorrect>>
      SynchronizerImagePoseCorrect;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          geometry_msgs::PoseStamped>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<visualization_msgs::Marker>> pose_correct_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImagePoseCorrect sync_image_pose_correct_;
  SynchronizerCloudPose sync_cloud_pose_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
      update_range_pub_, depth_pub_, tsdf_all_pub_, tsdf_slice_pub_, tsdf_occu_pub_,
      loop_closure_pub_, path_pose_pub_, alc_candidates_pub_, alc_candidates_selected_pub_,
      loop_path_before_pub_, loop_path_after_pub_, cloud_gt_low_pub_;
  ros::Subscriber pose_gt_sub_, pose_drift_sub_, pose_loop_sub_, path_opt_sub_, extrinsic_sub_,
      pose_no_loop_sub_, cloud_gt_sub_;
  ros::Timer esdf_timer_, vis_timer_, tsdf_reintegration_timer_;

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  int image_width_, image_height_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  std::string frame_id_;
  // msg publication
  double tsdf_slice_height_, esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  double occu_pub_period_, tsdf_pub_period_, esdf_pub_period_;
  bool show_esdf_time_, show_occ_time_, show_set_cover_time_;
  bool show_occu_map_, show_occu_local_, show_tsdf_map_, show_tsdf_slice_, show_esdf_map_,
      show_keyframe_, show_depth_cloud_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  std::unique_ptr<cv::Mat> depth_image_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_cam_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  ros::Time map_start_time_, last_loop_time;

  std::mutex deintegration_lock_, extrinsic_mutex_, pose_mutex_, pose_no_loop_mutex_;
  int frame_count_, frame_count_correct_;
  Eigen::Matrix4d cam02body;

  ros::Time enter_alc_cluster_time_, no_alc_time_, old_pose_time_, new_pose_time_, last_loop_time_;
  bool is_in_alc_cluster_, no_alc_flag_, handle_mock_loop_, new_loop_closure_flag_, is_simulation_;

  int input_pointcloud_mode_;

  nav_msgs::Path loop_path_before_;
  nav_msgs::Path loop_path_after_;

  Eigen::Vector3d tic;
  Eigen::Quaterniond qic;

  Eigen::Vector3d t_cur_;
  Eigen::Quaterniond q_cur_;

  Eigen::Vector3d pos_no_loop_;
  Eigen::Quaterniond q_no_loop_;

  friend SDFMap;
};
}  // namespace fast_planner

#endif