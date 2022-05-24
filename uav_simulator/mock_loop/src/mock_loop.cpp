#include <vector>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

ros::Publisher loop_odom_pub_, loop_pose_pub_, path_pub_, path_gt_pub_, loop_trigger_pub_,
    pg_T_vio_pub_;
ros::Subscriber odom_sub_, odom_gt_sub_, pose_sub_;
ros::ServiceServer mock_loop_ser_;
ros::Time last_loop_time_, t_loop_old_, t_loop_cur_, start_time_;
nav_msgs::Path path_, path_gt_;

Eigen::Vector3d w_t_vio_, P_cur;
Eigen::Matrix3d w_R_vio_, R_cur;
Eigen::Matrix4d cam02body;

std::vector<Eigen::Vector3d> w_t_vio_vec_;
std::vector<Eigen::Matrix3d> w_R_vio_vec_;

int sequence_cnt = 0;
double loop_range_th_, loop_angle_th_;
bool handle_mock_loop_ = false, is_start_ = false, trigger_loop_ = false;

bool getPoseByTime(const ros::Time& t, const nav_msgs::Path& path, Eigen::Vector3d& p,
                   Eigen::Matrix3d& R) {
  if (path.poses.size() < 2) return false;

  if (t.toSec() < path.poses.front().header.stamp.toSec() ||
      t.toSec() > path.poses.back().header.stamp.toSec()) {
    return false;
  }

  geometry_msgs::PoseStamped last_pose = path.poses.front();
  for (const geometry_msgs::PoseStamped& pose : path.poses) {
    if (pose.header.stamp.toSec() >= t.toSec() && last_pose.header.stamp.toSec() <= t.toSec()) {
      double t_old = t.toSec() - last_pose.header.stamp.toSec();
      double t_cur = pose.header.stamp.toSec() - t.toSec();
      double t_diff = t_old / (t_cur + t_old);

      // Interpolation
      Eigen::Vector3d pos_cur, pos_last;
      Eigen::Quaterniond q_cur, q_last;

      tf::pointMsgToEigen(pose.pose.position, pos_cur);
      tf::pointMsgToEigen(last_pose.pose.position, pos_last);
      tf::quaternionMsgToEigen(pose.pose.orientation, q_cur);
      tf::quaternionMsgToEigen(last_pose.pose.orientation, q_last);

      // Interpolation to find p and R
      p = (1 - t_diff) * pos_last + t_diff * pos_cur;
      R = q_last.slerp(t_diff, q_cur).toRotationMatrix();

      return true;
    }
    last_pose = pose;
  }

  return false;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  Eigen::Vector3d t_cur, t_rect;
  Eigen::Quaterniond q_cur, q_rect;
  nav_msgs::Odometry odom_rect;

  tf::pointMsgToEigen(msg->pose.pose.position, t_cur);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, q_cur);

  t_rect = t_cur;
  q_rect = q_cur;

  Eigen::Vector3d pg_t_vio(0, 0, 0);
  Eigen::Quaterniond pg_q_vio(Eigen::Matrix3d::Identity());
  double pg_yaw_vio;

  for (int i = 0; i < w_t_vio_vec_.size(); i++) {
    t_rect = w_R_vio_vec_[i] * t_rect + w_t_vio_vec_[i];
    q_rect = w_R_vio_vec_[i] * q_rect.toRotationMatrix();

    pg_t_vio = w_R_vio_vec_[i] * pg_t_vio + w_t_vio_vec_[i];
    pg_q_vio = w_R_vio_vec_[i] * pg_q_vio.toRotationMatrix();
  }

  odom_rect.header = msg->header;
  tf::pointEigenToMsg(t_rect, odom_rect.pose.pose.position);
  tf::quaternionEigenToMsg(q_rect, odom_rect.pose.pose.orientation);
  loop_odom_pub_.publish(odom_rect);

  geometry_msgs::Pose pg_T_vio;
  tf::pointEigenToMsg(pg_t_vio, pg_T_vio.position);
  tf::quaternionEigenToMsg(pg_q_vio, pg_T_vio.orientation);
  pg_T_vio_pub_.publish(pg_T_vio);
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!is_start_) {
    start_time_ = msg->header.stamp;
    is_start_ = true;
  }

  Eigen::Vector3d t_cur, t_rect;
  Eigen::Quaterniond q_cur, q_rect;
  geometry_msgs::PoseStamped pose_rect;

  tf::pointMsgToEigen(msg->pose.position, t_cur);
  tf::quaternionMsgToEigen(msg->pose.orientation, q_cur);

  t_rect = t_cur;
  q_rect = q_cur;

  for (int i = 0; i < w_t_vio_vec_.size(); i++) {
    t_rect = w_R_vio_vec_[i] * t_rect + w_t_vio_vec_[i];
    q_rect = w_R_vio_vec_[i] * q_rect.toRotationMatrix();
  }

  pose_rect.header = msg->header;
  tf::pointEigenToMsg(t_rect, pose_rect.pose.position);
  tf::quaternionEigenToMsg(q_rect, pose_rect.pose.orientation);

  if (path_.poses.empty() || msg->header.stamp - path_.header.stamp > ros::Duration(0.1)) {
    if (!path_.poses.empty()) {
      Eigen::Vector3d pt_cur, pt_last;
      tf::pointMsgToEigen(msg->pose.position, pt_cur);
      tf::pointMsgToEigen(path_.poses.back().pose.position, pt_last);
      if ((pt_cur - pt_last).norm() > 0.1) {
        path_.header = msg->header;
        path_.header.seq = sequence_cnt;
        path_.poses.push_back(pose_rect);
        path_pub_.publish(path_);
      }
    } else {
      path_.header = msg->header;
      path_.header.seq = sequence_cnt;
      path_.poses.push_back(pose_rect);
      path_pub_.publish(path_);
    }
  }

  loop_pose_pub_.publish(pose_rect);

  if (handle_mock_loop_) {
    // sequence_cnt++;
    path_.header.seq = sequence_cnt;

    ROS_INFO("[Mock loop] Start handle loop closure");
    Eigen::Vector3d vio_P_cur;
    Eigen::Matrix3d vio_R_cur;

    if (!getPoseByTime(t_loop_cur_, path_, vio_P_cur, vio_R_cur)) {
      // ROS_ERROR("[Mock loop] Cannot get cur loop pose");
      // ROS_ERROR("[Mock loop] t_loop_cur_: %f, path_ t_last: %f", t_loop_cur_.toSec(),
      //           path_.poses.back().header.stamp.toSec());
      // handle_mock_loop_ = false;
      return;
    }

    w_t_vio_ = P_cur - vio_P_cur;
    w_R_vio_ = R_cur * vio_R_cur.transpose();

    w_t_vio_vec_.push_back(w_t_vio_);
    w_R_vio_vec_.push_back(w_R_vio_);

    Eigen::Vector3d t_old, t_rect;
    Eigen::Matrix3d R_old, R_rect;
    Eigen::Quaterniond q_old, q_rect;
    for (auto pose_it = path_.poses.begin(); pose_it != path_.poses.end();) {
      // tf::pointMsgToEigen(pose.pose.position, t_old);
      // tf::quaternionMsgToEigen(pose.pose.orientation, q_old);

      // t_rect = w_R_vio_ * t_old + w_t_vio_;
      if (!getPoseByTime(pose_it->header.stamp, path_gt_, t_rect, R_rect)) {
        ROS_ERROR("[Mock loop] Cannot find pose from path_gt_, pose t: %f",
                  pose_it->header.stamp.toSec());
        pose_it = path_.poses.erase(pose_it);
        continue;
      }

      q_rect = R_rect;
      tf::pointEigenToMsg(t_rect, pose_it->pose.position);
      tf::quaternionEigenToMsg(q_rect, pose_it->pose.orientation);

      pose_it++;
    }

    handle_mock_loop_ = false;

    std_msgs::Time time_msg;
    time_msg.data = path_.header.stamp;
    loop_trigger_pub_.publish(path_);
  }
}

void odomGTCallback(const visualization_msgs::MarkerConstPtr& msg) {
  if (!is_start_ || msg->header.stamp.toSec() <= start_time_.toSec()) {
    ROS_WARN_THROTTLE(1, "[Mock loop] Ground truth pose arrive too early");
    return;
  }

  ros::Time t_cur = msg->header.stamp;

  // trigger loop using ground truth loop closure
  Eigen::Vector3d pos_recieved;
  Eigen::Quaterniond q_recieved;
  Eigen::Matrix4d w_T_i, w_T_c;
  geometry_msgs::PoseStamped pose_recieved;

  tf::pointMsgToEigen(msg->pose.position, pos_recieved);
  tf::quaternionMsgToEigen(msg->pose.orientation, q_recieved);

  w_T_i(0, 3) = pos_recieved[0];
  w_T_i(1, 3) = pos_recieved[1];
  w_T_i(2, 3) = pos_recieved[2];
  w_T_i.block<3, 3>(0, 0) = q_recieved.toRotationMatrix();
  w_T_c = w_T_i * cam02body;

  pose_recieved.header = msg->header;
  tf::pointEigenToMsg(Eigen::Vector3d(w_T_c(0, 3), w_T_c(1, 3), w_T_c(2, 3)),
                      pose_recieved.pose.position);
  tf::quaternionEigenToMsg(Eigen::Quaterniond(w_T_c.block<3, 3>(0, 0)),
                           pose_recieved.pose.orientation);

  // Init, directly push back recieved odom
  if (path_gt_.poses.empty()) {
    path_gt_.header = msg->header;
    path_gt_.poses.push_back(pose_recieved);
    return;
  }

  Eigen::Vector3d p_last, p_old, p_cur;
  Eigen::Quaterniond q_old, q_cur;
  tf::pointMsgToEigen(path_gt_.poses.back().pose.position, p_last);
  tf::pointMsgToEigen(pose_recieved.pose.position, p_cur);
  tf::quaternionMsgToEigen(pose_recieved.pose.orientation, q_cur);

  // if ((p_cur - p_last).norm() > 0.1) {
  if ((t_cur - path_gt_.header.stamp) > ros::Duration(0.1)) {
    path_gt_.header = msg->header;
    path_gt_.poses.push_back(pose_recieved);
  }

  // find connections in hist path
  for (geometry_msgs::PoseStamped pose : path_gt_.poses) {
    tf::pointMsgToEigen(pose.pose.position, p_old);
    tf::quaternionMsgToEigen(pose.pose.orientation, q_old);
    double pos_diff = (p_cur - p_old).norm();
    double angle_diff =
        acos(((q_cur.toRotationMatrix() * q_old.toRotationMatrix().transpose()).trace() - 1) / 2);
    if (pos_diff < loop_range_th_ && angle_diff < loop_angle_th_ &&
            t_cur - pose.header.stamp > ros::Duration(15.0) &&  // avoid loop closure in situ
            t_cur - last_loop_time_ > ros::Duration(5.0) ||
        trigger_loop_) {  // avoid burst loop closure
      //   map_->handleLoopClosureSimulation();
      trigger_loop_ = false;
      last_loop_time_ = t_cur;
      //   map_->loop_closure_points_.push_back(pose->pose.position);
      //   publishLoopClosurePoints();
      handle_mock_loop_ = true;
      t_loop_old_ = pose.header.stamp;
      t_loop_cur_ = t_cur;
      P_cur = p_cur;
      R_cur = q_cur.toRotationMatrix();

      ROS_WARN("[Mock loop] Find loop closure points, t1: %f, t2: %f", t_loop_old_.toSec(),
               t_loop_cur_.toSec());
    }
  }

  path_gt_pub_.publish(path_gt_);
  return;
}

bool triggerLoopCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  trigger_loop_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_loop_node");
  ros::NodeHandle nh("~");

  loop_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mock_loop/odometry", 100);
  loop_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mock_loop/camera_pose", 100);
  path_pub_ = nh.advertise<nav_msgs::Path>("/loop_fusion/pose_graph_path", 100);
  path_gt_pub_ = nh.advertise<nav_msgs::Path>("/mock_loop/path_gt", 100);
  pg_T_vio_pub_ = nh.advertise<geometry_msgs::Pose>("/loop_fusion/pg_T_vio", 100);
  loop_trigger_pub_ = nh.advertise<nav_msgs::Path>("/loop_fusion/path_optimization_trigger", 100);

  odom_sub_ = nh.subscribe("/state_ukf/odom", 100, odomCallback);
  odom_gt_sub_ = nh.subscribe("/odom_visualization/robot", 100, odomGTCallback);
  pose_sub_ = nh.subscribe("/pcl_render_node/sensor_pose", 100, poseCallback);

  // Service for debug
  mock_loop_ser_ = nh.advertiseService("/mock_loop/trigger_loop", triggerLoopCallback);

  nh.getParam("loop_range_th", loop_range_th_);
  nh.getParam("loop_angle_th", loop_angle_th_);

  // Simulator extrinsic parameter
  cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  w_t_vio_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  w_R_vio_ = Eigen::Matrix3d::Identity();

  ros::spin();
}