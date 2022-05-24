#include <plan_env/map_ros.h>
#include <plan_env/sdf_map.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>

namespace fast_planner {
MapROS::MapROS() {}

MapROS::~MapROS() {}

void MapROS::setMap(SDFMap *map) { this->map_ = map; }

void MapROS::init() {
  node_.param("map_ros/fx", fx_, -1.0);
  node_.param("map_ros/fy", fy_, -1.0);
  node_.param("map_ros/cx", cx_, -1.0);
  node_.param("map_ros/cy", cy_, -1.0);
  node_.param("map_ros/image_width", image_width_, 640);
  node_.param("map_ros/image_height", image_height_, 480);
  node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  node_.param("map_ros/tsdf_slice_height", tsdf_slice_height_, -0.1);
  node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  node_.param("map_ros/show_occ_time", show_occ_time_, false);
  node_.param("map_ros/show_set_cover_time", show_set_cover_time_, false);
  node_.param("map_ros/show_occu_map", show_occu_map_, false);
  node_.param("map_ros/show_occu_local", show_occu_local_, false);
  node_.param("map_ros/show_tsdf_map", show_tsdf_map_, false);
  node_.param("map_ros/show_tsdf_slice", show_tsdf_slice_, false);
  node_.param("map_ros/show_esdf_map", show_esdf_map_, false);
  node_.param("map_ros/show_keyframe", show_keyframe_, false);
  node_.param("map_ros/show_depth_cloud", show_depth_cloud_, false);
  node_.param("map_ros/occu_pub_period", occu_pub_period_, 1.0);
  node_.param("map_ros/tsdf_pub_period", tsdf_pub_period_, 1.0);
  node_.param("map_ros/esdf_pub_period", esdf_pub_period_, 1.0);
  node_.param("map_ros/is_simulation", is_simulation_, false);
  node_.param("map_ros/frame_id", frame_id_, string("world"));
  node_.param("map_ros/input_pointcloud_mode", input_pointcloud_mode_, 0);

  proj_points_.resize(image_width_ * image_height_ / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(image_width_ * image_height_ / (skip_pixel_ * skip_pixel_));
  point_cloud_cam_.points.resize(image_width_ * image_height_ / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ /
  // map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

  map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 1);
  map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 1);
  map_local_inflate_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 1);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 1);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 1);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 1);
  // depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 1);
  tsdf_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/tsdf_all", 1);
  tsdf_slice_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/tsdf_slice", 1);
  tsdf_occu_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/tsdf_occu", 1);
  loop_closure_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/loop_closure", 1);
  path_pose_pub_ = node_.advertise<geometry_msgs::PoseArray>("/sdf_map/poses_on_map", 1);
  alc_candidates_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/alc_candidates", 1);
  alc_candidates_selected_pub_ =
      node_.advertise<visualization_msgs::Marker>("/sdf_map/alc_candidates_selected", 1);
  loop_path_before_pub_ = node_.advertise<nav_msgs::Path>("/sdf_map/loop_path_before", 1);
  loop_path_after_pub_ = node_.advertise<nav_msgs::Path>("/sdf_map/loop_path_after", 1);
  cloud_gt_low_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/cloud_gt_low_", 1);

  // For simulation, camera pose is drifted by a noise
  pose_drift_sub_ = node_.subscribe("/map_ros/pose", 1, &MapROS::poseCallback, this);
  // pose_gt_sub_ = node_.subscribe("/map_ros/pose", 1, &MapROS::poseGroundTruthCallback, this);
  // pose_no_loop_sub_ =
  //     node_.subscribe("/map_ros/pose_sim_no_loop", 1, &MapROS::poseDriftCallback, this);
  pose_loop_sub_ = node_.subscribe("/loop_fusion/pose_graph_path", 100,
                                   &MapROS::poseLoopClosurePathCallback, this);
  path_opt_sub_ =
      node_.subscribe("/map_ros/loop_result", 100, &MapROS::poseOptimizationTriggerCallback, this);
  extrinsic_sub_ =
      node_.subscribe("/vins_estimator/extrinsic", 100, &MapROS::extrinsicCallback, this);
  cloud_gt_sub_ =
      node_.subscribe("/map_generator/global_cloud", 100, &MapROS::cloudGtCallback, this);

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      node_, "/map_ros/depth", 10, ros::TransportHints().tcpNoDelay()));
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 10));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 10));

  sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
      MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));

  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = ros::Time::now();
  frame_count_ = 0;
  frame_count_correct_ = 0;
  cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  is_in_alc_cluster_ = false;
  no_alc_flag_ = false;
  new_loop_closure_flag_ = false;
  // handle_mock_loop_ = false;
}

void MapROS::visCallback(const ros::TimerEvent &e) {
  if (show_occu_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > occu_pub_period_) {
      publishOccu();
      tpass = 0.0;
    }
  }

  if (show_occu_local_) {
    publishOccuLocal();
  }

  if (show_tsdf_map_) {
    static double tpass_tsdf = 0.0;
    tpass_tsdf += (e.current_real - e.last_real).toSec();
    if (tpass_tsdf > tsdf_pub_period_) {
      publishTSDF();
      tpass_tsdf = 0.0;
    }
  }

  if (show_tsdf_slice_) {
    publishTSDFSlice();
  }

  if (show_esdf_map_) {
    static double tpass_esdf = 0.0;
    tpass_esdf += (e.current_real - e.last_real).toSec();
    if (tpass_esdf > esdf_pub_period_) {
      publishESDF();
      tpass_esdf = 0.0;
    }
  }

  // map_->publishAllBlocks();
  // publishUnknown();
  // publishUpdateRange();
}

void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/) {
  if (!esdf_need_update_) return;
  auto t1 = ros::Time::now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_)
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
             max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                               const geometry_msgs::PoseStampedConstPtr &pose) {
  // ROS_WARN("Received depth pose synced pair: img: %f, pose: %f, diff: %f",
  //          img->header.stamp.toSec(), pose->header.stamp.toSec(),
  //          img->header.stamp.toSec() - pose->header.stamp.toSec());

  camera_pos_ =
      Eigen::Vector3d(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  // generate point cloud, update map
  processDepthImage();

  if (show_depth_cloud_) publishDepth(img->header);

  // Frame frame;
  // frame.point_cloud_ = point_cloud_;
  // frame.camera_pos_ = camera_pos_;

  // cout << "Image size: " << depth_image_->cols << " x " << depth_image_->rows
  // << endl; cout << "point_cloud_ size: " << proj_points_cnt << endl;
  auto t1 = ros::Time::now();

  switch (input_pointcloud_mode_) {
    case 0:
      map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
      break;
    case 1:
      // using the pointcloud under camera frame for the convenience of reintegration
      map_->inputPointCloudSetCover(point_cloud_cam_, proj_points_cnt, camera_pos_, camera_q_,
                                    img->header.stamp);
      break;
    default:
      map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  }

  auto t_inf = ros::Time::now();

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = ros::Time::now();

  fuse_time_ += (t_inf - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t_inf - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_) {
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t_inf - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
    ROS_WARN("Inflate t: cur: %lf", (t2 - t_inf).toSec());
  }
}

// cloud is under camera frame
void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                               const geometry_msgs::PoseStampedConstPtr &pose) {
  // camera_pos_(0) = pose->pose.position.x;
  // camera_pos_(1) = pose->pose.position.y;
  // camera_pos_(2) = pose->pose.position.z;
  // camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
  //                                pose->pose.orientation.y, pose->pose.orientation.z);
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // pcl::fromROSMsg(*msg, cloud);
  // int num = cloud.points.size();

  // map_->inputPointCloud(cloud, num, camera_pos_);

  // if (local_updated_) {
  //   map_->clearAndInflateLocalMap();
  //   esdf_need_update_ = true;
  //   local_updated_ = false;
  // }

  camera_pos_ =
      Eigen::Vector3d(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  point_cloud_.clear();

  for (auto point : cloud.points) {
    Eigen::Vector3d pt_w =
        camera_q_.toRotationMatrix() * Eigen::Vector3d(point.x, point.y, point.z) + camera_pos_;
    pcl::PointXYZ pt;
    pt.x = pt_w[0];
    pt.y = pt_w[1];
    pt.z = pt_w[2];

    point_cloud_.points.push_back(pt);
    publishDepth(cloud_msg->header);
  }

  // cout << "Cloud size: " << cloud_msg->width << endl;
  // cout << "CloudPCL size: " << cloud_msg->width << endl;
  // cout << "CloudVis size: " << point_cloud_.points.size() << endl;
  // << endl; cout << "point_cloud_ size: " << proj_points_cnt << endl;
  auto t1 = ros::Time::now();

  map_->inputPointCloudSetCover(cloud, cloud.points.size(), camera_pos_, camera_q_,
                                cloud_msg->header.stamp);

  auto t_inf = ros::Time::now();

  if (local_updated_) {
    // comment for speed up, what it is used for?
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = ros::Time::now();

  fuse_time_ += (t_inf - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t_inf - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_) {
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t_inf - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
    ROS_WARN("Inflate t: cur: %lf", (t2 - t_inf).toSec());
  }
}

// Record the poses and do active loop closure
void MapROS::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {
  static double last_tick = 0.0;
  if (pose->header.stamp.toSec() - last_tick < 0.02) return;
  last_tick = pose->header.stamp.toSec();

  ros::Time received_time_stamp(pose->header.stamp);
  Eigen::Vector3d received_position(pose->pose.position.x, pose->pose.position.y,
                                    pose->pose.position.z);
  Eigen::Quaterniond received_q(pose->pose.orientation.w, pose->pose.orientation.x,
                                pose->pose.orientation.y, pose->pose.orientation.z);
  // map_->pose_graph_drift_.push_back(Pose(received_time_stamp, received_position, received_q));

  pose_mutex_.lock();
  t_cur_ = received_position;
  q_cur_ = received_q;
  pose_mutex_.unlock();

  // if (map_->active_loop_closure_candidates_.empty() ||
  //     (received_position - map_->active_loop_closure_candidates_.back().position_).norm() > 0.5)
  //     {
  //   map_->active_loop_closure_candidates_.push_back(
  //       Pose(pose->header.stamp, received_position, received_q));
  // }

  // int cluster_count = 0;
  // bool last_flag = false;

  // std::unique_lock<std::mutex> lock(map_->alc_mutex_);
  // map_->active_loop_closure_clusters_.clear();
  // if (!no_alc_flag_ && (ros::Time::now() - last_loop_time_ > ros::Duration(15.0))) {
  //   // if (!no_alc_flag_) {
  //   for (Pose &pose_hist : map_->active_loop_closure_candidates_) {
  //     double pos_diff = (pose_hist.position_ - received_position).norm();
  //     double angle_diff = acos(
  //         ((received_q.toRotationMatrix() * pose_hist.quaternion_.toRotationMatrix().transpose())
  //              .trace() -
  //          1) /
  //         2);
  //     if (pos_diff < 3.0 && received_time_stamp - pose_hist.time_stamp_ > ros::Duration(30.0)) {
  //       if (!last_flag) cluster_count++;
  //       map_->active_loop_closure_candidates_selected_.push_back(pose_hist);

  //       // if (map_->active_loop_closure_candidates_selected_.size() > 5)
  //       //   last_flag = false;
  //       // else
  //       last_flag = true;
  //     } else {
  //       if (last_flag) {
  //         // DO not add this cluster if it is close to a history cluster
  //         bool is_history = false;
  //         for (auto cluster : map_->active_loop_closure_clusters_) {
  //           if ((map_->active_loop_closure_candidates_selected_
  //                    [map_->active_loop_closure_candidates_selected_.size() / 2]
  //                        .position_ -
  //                cluster.center_position_)
  //                   .norm() < 2.0)
  //             is_history = true;
  //         }

  //         if (!is_history) {
  //           if (map_->active_loop_closure_candidates_selected_.size() < 6)
  //             map_->active_loop_closure_clusters_.push_back(ActiveLoopClosureCluster(
  //                 received_time_stamp, map_->active_loop_closure_candidates_selected_));
  //           else {
  //             vector<Pose> tmp;
  //             int cnt = 0;
  //             for (int i = 0; i < map_->active_loop_closure_candidates_selected_.size(); i++) {
  //               tmp.push_back(map_->active_loop_closure_candidates_selected_[i]);
  //               cnt++;

  //               if (cnt == 5) {
  //                 map_->active_loop_closure_clusters_.push_back(
  //                     ActiveLoopClosureCluster(received_time_stamp, tmp));
  //                 tmp.clear();
  //               }
  //             }

  //             if (tmp.size() > 0)
  //               map_->active_loop_closure_clusters_.push_back(
  //                   ActiveLoopClosureCluster(received_time_stamp, tmp));
  //           }
  //         }
  //         map_->active_loop_closure_candidates_selected_.clear();
  //         last_flag = false;
  //       }
  //     }
  //   }
  // }
  // lock.unlock();

  // publishALCCandidates();

  // bool is_found_cluster = false;
  // for (auto cluster : map_->active_loop_closure_clusters_) {
  //   for (auto pose : cluster.active_loop_closure_candidates_) {
  //     if ((received_position - pose.position_).norm() < 0.5) {
  //       if (!is_in_alc_cluster_) enter_alc_cluster_time_ = ros::Time::now();
  //       is_in_alc_cluster_ = true;
  //       is_found_cluster = true;
  //       break;
  //     }
  //   }
  // }

  // if (!is_found_cluster) {
  //   is_in_alc_cluster_ = false;
  //   enter_alc_cluster_time_ = ros::Time::now();
  // }

  // if ((ros::Time::now() - enter_alc_cluster_time_).toSec() > 5.0 && !no_alc_flag_) {
  //   ROS_ERROR("Exceed time limit for alc cluster, exit and raise no alc flag");
  //   no_alc_flag_ = true;
  //   no_alc_time_ = ros::Time::now();
  // }

  // if ((ros::Time::now() - no_alc_time_).toSec() > 10.0 && no_alc_flag_) {
  //   ROS_ERROR("normally find alc clusters");
  //   no_alc_flag_ = false;
  // }

  // ALC above

  // if (cluster_count > 0)
  //   ROS_WARN_STREAM("cluster num: " << cluster_count << ", cluster size: "
  //                                   << map_->active_loop_closure_clusters_.size());

  // if (loop_path_before_.poses.empty()) {
  //   loop_path_before_.header = pose->header;
  //   loop_path_after_.header = pose->header;

  //   geometry_msgs::PoseStamped p;
  //   p.header = pose->header;
  //   p.pose = pose->pose;
  //   loop_path_before_.poses.push_back(p);
  //   // loop_path_after_.poses.push_back(p);
  // } else {
  //   Eigen::Vector3d last_pos(loop_path_before_.poses.back().pose.position.x,
  //                            loop_path_before_.poses.back().pose.position.y,
  //                            loop_path_before_.poses.back().pose.position.z);
  //   if ((received_position - last_pos).norm() > 0.1) {
  //     loop_path_before_.header = pose->header;
  //     loop_path_after_.header = pose->header;

  //     geometry_msgs::PoseStamped p;
  //     p.header = pose->header;
  //     p.pose = pose->pose;
  //     loop_path_before_.poses.push_back(p);
  //     // loop_path_after_.poses.push_back(p);
  //   }
  // }

  // if (handle_mock_loop_) {
  //   Eigen::Vector3d w_P_old, w_P_cur, vio_P_cur, relative_t;
  //   Eigen::Matrix3d w_R_old, w_R_cur, vio_R_cur, relative_q;

  //   Eigen::Vector3d P_old, P_cur;
  //   Eigen::Quaterniond q_old, q_cur;

  //   Pose pose_old, pose_cur;
  //   pose_old.time_stamp_ = old_pose_time_;
  //   if (!map_->getPoseByTime(old_pose_time_, pose_old.position_, pose_old.quaternion_)) {
  //     ROS_ERROR("Cannot get pose old by time from loop_path_after");
  //     handle_mock_loop_ = false;
  //     return;
  //   }

  //   // drift
  //   w_P_old = pose_old.position_;
  //   w_R_old = pose_old.quaternion_.toRotationMatrix();

  //   // drift
  //   vio_P_cur = received_position;
  //   vio_R_cur = received_q.toRotationMatrix();

  //   // True relative position?
  //   if (!map_->getCorrectPoseByTime(old_pose_time_, P_old, q_old)) {
  //     ROS_ERROR("Cannot get pose by old_pose_time_ from pose graph");
  //     handle_mock_loop_ = false;
  //     return;
  //   }
  //   if (!map_->getCorrectPoseByTime(received_time_stamp, P_cur, q_cur)) {
  //     ROS_ERROR("Cannot get pose by received_time_stamp from pose graph");
  //     handle_mock_loop_ = false;
  //     return;
  //   }

  //   if (q_cur.norm() < 0.5) {
  //     ROS_ERROR("q_cur wrong value");
  //     return;
  //   }

  //   shift_t = P_cur - vio_P_cur;                                 // w_t_vio
  //   shift_r = q_cur.toRotationMatrix() * vio_R_cur.transpose();  // w_R_vio

  //   Eigen::Vector3d P, Pi;
  //   Eigen::Quaterniond q, qi;
  //   loop_path_after_.poses.clear();
  //   for (const geometry_msgs::PoseStamped &p : loop_path_before_.poses) {
  //     // update
  //     // vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
  //     // vio_R_cur = w_r_vio * vio_R_cur;

  //     Pi[0] = p.pose.position.x;
  //     Pi[1] = p.pose.position.y;
  //     Pi[2] = p.pose.position.z;

  //     qi.w() = p.pose.orientation.w;
  //     qi.x() = p.pose.orientation.x;
  //     qi.y() = p.pose.orientation.y;
  //     qi.z() = p.pose.orientation.z;

  //     P = shift_r * Pi + shift_t;
  //     q = shift_r * qi.toRotationMatrix();

  //     geometry_msgs::PoseStamped res;
  //     res.pose.position.x = P[0];
  //     res.pose.position.y = P[1];
  //     res.pose.position.z = P[2];

  //     res.pose.orientation.w = q.w();
  //     res.pose.orientation.x = q.x();
  //     res.pose.orientation.y = q.y();
  //     res.pose.orientation.z = q.z();

  //     loop_path_after_.poses.push_back(res);
  //   }

  //   loop_path_after_pub_.publish(loop_path_after_);

  //   handle_mock_loop_ = false;
  // }

  // loop_path_before_pub_.publish(loop_path_before_);

  // // w_P_old = pose_hist.position_;
  // // w_R_old = pose_hist.quaternion_.toRotationMatrix();

  // // vio_P_cur = pos_correct;
  // // vio_R_cur = camera_q_correct.toRotationMatrix();

  // // cur_kf->getVioPose(vio_P_cur, vio_R_cur);
  // // for (auto p : loop_path_after_.poses) {
  // //   if ((*it)->sequence == cur_kf->sequence) {
  // //     Vector3d vio_P_cur;
  // //     Matrix3d vio_R_cur;
  // //     (*it)->getVioPose(vio_P_cur, vio_R_cur);
  // //     vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
  // //     vio_R_cur = w_r_vio * vio_R_cur;
  // //     (*it)->updateVioPose(vio_P_cur, vio_R_cur);
  // //   }
  // // }
}

void MapROS::poseDriftCallback(const geometry_msgs::PoseStampedConstPtr &pose) {
  pose_no_loop_mutex_.lock();
  tf::pointMsgToEigen(pose->pose.position, pos_no_loop_);
  tf::quaternionMsgToEigen(pose->pose.orientation, q_no_loop_);
  pose_no_loop_mutex_.unlock();
}

void MapROS::poseGroundTruthCallback(const visualization_msgs::MarkerConstPtr &pose) {
  static double path_len = 0.0;
  static Eigen::Vector3d last_pos(0.0, 0.0, 0.0);
  // static double last_yaw = 0.0;
  // pose here is body2world
  ros::Time received_time_stamp = pose->header.stamp;
  Eigen::Matrix4d Pose_receive = Eigen::Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = pose->pose.position.x;
  request_position.y() = pose->pose.position.y;
  request_position.z() = pose->pose.position.z;
  request_pose.x() = pose->pose.orientation.x;
  request_pose.y() = pose->pose.orientation.y;
  request_pose.z() = pose->pose.orientation.z;
  request_pose.w() = pose->pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Eigen::Matrix4d cam2world = Pose_receive * cam02body;

  // map_->correct_pose_vector_.push_back(
  //     std::make_pair(pose->header.stamp.toSec(), cam2world));
  // ROS_WARN_THROTTLE(
  //     0.1, "Start time of pose gt: %f", map_->correct_pose_vector_[0].first);
  // ROS_WARN_THROTTLE(
  //     0.1, "End time of pose gt: %f",
  //     map_->correct_pose_vector_[map_->correct_pose_vector_.size() -
  //     1].first);

  Eigen::Vector3d pos_correct;
  pos_correct.x() = cam2world(0, 3);
  pos_correct.y() = cam2world(1, 3);
  pos_correct.z() = cam2world(2, 3);

  Eigen::Quaterniond camera_q_correct(cam2world.block<3, 3>(0, 0));

  path_len += (pos_correct - last_pos).norm();

  if (path_len > 100) {
    ROS_WARN("length exceed 100");
    pose_no_loop_mutex_.lock();
    // double pos_error
    pose_no_loop_mutex_.unlock();
  }
}
//   map_->pose_graph_.push_back(Pose(pose->header.stamp, pos_correct, camera_q_correct));
//   map_->pose_graph_gt_.header = pose->header;
//   geometry_msgs::PoseStamped pose_stamped;
//   pose_stamped.pose.position.x = pos_correct[0];
//   pose_stamped.pose.position.y = pos_correct[1];
//   pose_stamped.pose.position.z = pos_correct[2];

//   pose_stamped.pose.orientation.w = camera_q_correct.w();
//   pose_stamped.pose.orientation.x = camera_q_correct.x();
//   pose_stamped.pose.orientation.y = camera_q_correct.y();
//   pose_stamped.pose.orientation.z = camera_q_correct.z();
//   map_->pose_graph_gt_.poses.push_back(pose_stamped);
//   map_->pose_gt_pub_.publish(map_->pose_graph_gt_);

//   if (map_->pose_graph_loop_.empty() ||
//       (pos_correct - map_->pose_graph_loop_.back().position_).norm() > 0.2) {
//     map_->pose_graph_loop_.push_back(Pose(pose->header.stamp, pos_correct, camera_q_correct));
//   }

//   for (Pose &pose_hist : map_->pose_graph_loop_) {
//     double pos_diff = (pose_hist.position_ - pos_correct).norm();
//     double angle_diff = acos(((camera_q_correct.toRotationMatrix() *
//                                pose_hist.quaternion_.toRotationMatrix().transpose())
//                                   .trace() -
//                               1) /
//                              2);
//     if (pos_diff < 0.5 && angle_diff < 15.0 * M_PI / 180.0 &&  // set mock loop closure range
//         received_time_stamp - pose_hist.time_stamp_ >
//             ros::Duration(1.0) &&                                     // avoid loop closure in
//             situ
//         received_time_stamp - last_loop_time > ros::Duration(5.0)) {  // avoid burst loop closure
//       map_->handleLoopClosureSimulation();
//       last_loop_time = received_time_stamp;
//       map_->loop_closure_points_.push_back(pose->pose.position);
//       publishLoopClosurePoints();
//       handle_mock_loop_ = true;
//       old_pose_time_ = pose_hist.time_stamp_;
//       new_pose_time_ = received_time_stamp;
//     }
//   }

//   // save the drifted pose if the distance between it and the previous pose is above some
//   // threshold
//   // if (map_->pose_graph_loop_.empty() ||
//   //     (pos_correct - map_->pose_graph_loop_.back().position_).norm() > 0.2) {
//   //   map_->pose_graph_loop_.push_back(Pose(pose->header.stamp, pos_correct, camera_q_correct));
//   // }
// }

void MapROS::poseLoopClosurePathCallback(const nav_msgs::Path &path) {
  // ROS_WARN_STREAM("Received loop closure pose graph path, size of path: " << path.poses.size());

  ros::Time t1 = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ pt, search_pt;
  vector<geometry_msgs::Quaternion> q_vector;
  float radius = 3.0;

  cloud->width = path.poses.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  q_vector.resize(path.poses.size());

  for (auto pose : path.poses) {
    if ((t1 - pose.header.stamp) < ros::Duration(20.0)) break;
    pt.x = pose.pose.position.x;
    pt.y = pose.pose.position.y;
    pt.z = pose.pose.position.z;
    cloud->points.emplace_back(pt);
    q_vector.emplace_back(pose.pose.orientation);
  }

  if (cloud->points.empty()) return;

  kdtree.setInputCloud(cloud);

  pose_mutex_.lock();
  search_pt.x = t_cur_[0];
  search_pt.y = t_cur_[1];
  search_pt.z = t_cur_[2];
  pose_mutex_.unlock();

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  map_->alc_mutex_.lock();
  map_->active_loop_closure_clusters_.clear();
  map_->active_loop_closure_candidates_selected_.clear();
  map_->alc_mutex_.unlock();

  // ROS_WARN("ros::Time::now() - last_loop_time_ = %f", (ros::Time::now() -
  // last_loop_time_).toSec());

  if (kdtree.radiusSearch(search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
          0 &&
      !no_alc_flag_ &&
      (ros::Time::now() - last_loop_time_ > ros::Duration(map_->mp_->loop_max_duration_))) {
    int idx, last_idx = -10;
    geometry_msgs::Pose pose;
    // std::cout << "t_cur_: " << t_cur_.transpose() << std::endl;

    std::unique_lock<std::mutex> lock(map_->alc_mutex_);
    map_->active_loop_closure_clusters_.clear();
    map_->active_loop_closure_candidates_selected_.clear();

    std::sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      if ((*cloud)[pointIdxRadiusSearch[i]].z < 1e-3) continue;
      // std::cout << "Idx: " << pointIdxRadiusSearch[i] << "    "
      //           << (*cloud)[pointIdxRadiusSearch[i]].x << " " <<
      //           (*cloud)[pointIdxRadiusSearch[i]].y
      //           << " " << (*cloud)[pointIdxRadiusSearch[i]].z
      //           << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
      idx = pointIdxRadiusSearch[i];

      Eigen::Vector3d pos((*cloud)[pointIdxRadiusSearch[i]].x, (*cloud)[pointIdxRadiusSearch[i]].y,
                          (*cloud)[pointIdxRadiusSearch[i]].z);
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(q_vector[pointIdxRadiusSearch[i]], q);
      Pose pose_hist(t1, pos, q);

      // std::cout << "path.poses[idx].position: " << path.poses[idx].pose.position.x << " "
      //           << path.poses[idx].pose.position.y << " " << path.poses[idx].pose.position.z
      //           << endl;
      // std::cout << "pose_hist.position: " << pose_hist.position_.transpose() << endl;

      map_->active_loop_closure_candidates_selected_.push_back(pose_hist);

      // std::cout << "map_->active_loop_closure_candidates_selected_.size: "
      //           << map_->active_loop_closure_candidates_selected_.size() << endl;

      if (idx - last_idx != 1 && last_idx >= 0) {
        bool is_history = false;
        for (auto cluster : map_->active_loop_closure_clusters_) {
          if ((map_->active_loop_closure_candidates_selected_
                   [map_->active_loop_closure_candidates_selected_.size() / 2]
                       .position_ -
               cluster.center_position_)
                  .norm() < 2.0)
            is_history = true;
        }

        if (!is_history) {
          if (map_->active_loop_closure_candidates_selected_.size() < 20) {
            map_->active_loop_closure_clusters_.push_back(
                ActiveLoopClosureCluster(t1, map_->active_loop_closure_candidates_selected_));
            // std::cout << "map_->active_loop_closure_clusters_.size: "
            //           << map_->active_loop_closure_clusters_.size() << endl;
          } else {
            vector<Pose> tmp;
            int cnt = 0;
            for (int i = 0; i < map_->active_loop_closure_candidates_selected_.size(); i++) {
              tmp.push_back(map_->active_loop_closure_candidates_selected_[i]);
              cnt++;

              if (cnt == 20) {
                map_->active_loop_closure_clusters_.push_back(ActiveLoopClosureCluster(t1, tmp));
                tmp.clear();
              }
            }

            if (tmp.size() > 0)
              map_->active_loop_closure_clusters_.push_back(ActiveLoopClosureCluster(t1, tmp));
          }
          map_->active_loop_closure_candidates_selected_.clear();
        }
      }

      last_idx = idx;
    }

    if (!map_->active_loop_closure_candidates_selected_.empty()) {
      bool is_history = false;
      for (auto cluster : map_->active_loop_closure_clusters_) {
        if ((map_->active_loop_closure_candidates_selected_
                 [map_->active_loop_closure_candidates_selected_.size() / 2]
                     .position_ -
             cluster.center_position_)
                .norm() < 2.0)
          is_history = true;
      }

      if (!is_history) {
        if (map_->active_loop_closure_candidates_selected_.size() < 20) {
          map_->active_loop_closure_clusters_.push_back(
              ActiveLoopClosureCluster(t1, map_->active_loop_closure_candidates_selected_));
          // std::cout << "map_->active_loop_closure_clusters_.size: "
          //           << map_->active_loop_closure_clusters_.size() << endl;
        } else {
          vector<Pose> tmp;
          int cnt = 0;
          for (int i = 0; i < map_->active_loop_closure_candidates_selected_.size(); i++) {
            tmp.push_back(map_->active_loop_closure_candidates_selected_[i]);
            cnt++;

            if (cnt == 20) {
              map_->active_loop_closure_clusters_.push_back(ActiveLoopClosureCluster(t1, tmp));
              tmp.clear();
            }
          }

          if (tmp.size() > 0)
            map_->active_loop_closure_clusters_.push_back(ActiveLoopClosureCluster(t1, tmp));
        }
        map_->active_loop_closure_candidates_selected_.clear();
      }
    }

    lock.unlock();

    publishALCCandidates();
  }

  bool is_found_cluster = false;

  for (auto cluster : map_->active_loop_closure_clusters_) {
    for (auto pose : cluster.active_loop_closure_candidates_) {
      Eigen::Vector2d t_cur_xy(t_cur_[0], t_cur_[1]);
      Eigen::Vector2d pose_pos_xy(pose.position_[0], pose.position_[1]);
      if ((t_cur_xy - pose_pos_xy).norm() < 0.5) {
        if (!is_in_alc_cluster_) enter_alc_cluster_time_ = ros::Time::now();
        is_in_alc_cluster_ = true;
        is_found_cluster = true;
        break;
      }
    }
  }

  if (!is_found_cluster) {
    is_in_alc_cluster_ = false;
    enter_alc_cluster_time_ = ros::Time::now();
  }

  if (ros::Time::now() - enter_alc_cluster_time_ > ros::Duration(map_->mp_->alc_max_duration_) &&
      !no_alc_flag_) {
    // ROS_ERROR("Exceed time limit for alc cluster, exit and raise no alc flag");
    no_alc_flag_ = true;
    no_alc_time_ = ros::Time::now();
  }

  if ((ros::Time::now() - no_alc_time_).toSec() > 10.0 && no_alc_flag_) {
    // ROS_ERROR("normally find alc clusters");
    no_alc_flag_ = false;
  }

  // ROS_WARN("map_->active_loop_closure_clusters_ size: %d",
  //          map_->active_loop_closure_clusters_.size());

  ros::Time t2 = ros::Time::now();

  // ROS_WARN("kd tree search time: %f", (t2 - t1).toSec());
  // geometry_msgs::PoseArray posearray;
  // posearray.header.stamp = ros::Time::now();  // timestamp of creation of the msg
  // posearray.header.frame_id = "world";        // frame id in which the array is published

  // for (auto pose : map_->path_opt_.poses) {
  //   geometry_msgs::Pose p;  // one pose to put in the array
  //   p.position.x = pose.pose.position.x;
  //   p.position.y = pose.pose.position.y;
  //   p.position.z = pose.pose.position.z;
  //   p.orientation.w = pose.pose.orientation.w;
  //   p.orientation.x = pose.pose.orientation.x;
  //   p.orientation.y = pose.pose.orientation.y;
  //   p.orientation.z = pose.pose.orientation.z;
  //   // push in array (in C++ a vector, in python a list)
  //   posearray.poses.push_back(p);
  // }

  // path_pose_pub_.publish(posearray);
}

void MapROS::extrinsicCallback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  extrinsic_mutex_.lock();
  tic = Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  qic = Quaterniond(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
                    pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z)
            .toRotationMatrix();
  extrinsic_mutex_.unlock();
}

void MapROS::cloudGtCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  static bool is_received = false;

  if (!is_received) {
    pcl::PCLPointCloud2 tmp;
    pcl_conversions::toPCL(*cloud_msg, tmp);
    pcl::fromPCLPointCloud2(tmp, map_->cloud_gt_);
    // pcl::io::savePLYFile("/home/eason/workspace/plan_ws/src/LargeScaleExploration/fuel_planner/plan_env/gt.ply",
    //                      tmp);

    is_received = true;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (pcl::PointXYZ pt : map_->cloud_gt_.points) {
      if (pt.z > 1.5) continue;
      cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_gt_low_pub_.publish(cloud_msg);
  }

  return;
}

void MapROS::poseOptimizationTriggerCallback(const nav_msgs::Path &msg) {
  ROS_WARN(" Get path optimization ");
  // map_->path_opt_ = msg;
  // std::lock_guard<std::mutex> lock(map_->integration_mutex_);

  // map_->path_loop_ptr_ = msg;
  map_->path_loop_ = msg;
  new_loop_closure_flag_ = true;
  last_loop_time_ = ros::Time::now();
  if (is_simulation_)
    map_->handleLoopClosureSimulation();
  else
    map_->handleLoopClosure();
}

void MapROS::processDepthImage() {
  proj_points_cnt = 0;

  uint16_t *row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0) {
        if (is_simulation_)
          depth = depth_filter_maxdist_;
        else
          continue;
      } else {
        if (depth > depth_filter_maxdist_)  // continue;
          depth = depth_filter_maxdist_;
        else if (depth < depth_filter_mindist_)
          continue;
      }

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;

      auto &pt = point_cloud_.points[proj_points_cnt];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];

      auto &pt_c = point_cloud_cam_.points[proj_points_cnt];
      pt_c.x = pt_cur[0];
      pt_c.y = pt_cur[1];
      pt_c.z = pt_cur[2];

      proj_points_cnt++;
    }
  }
}

void MapROS::publishOccu() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  Eigen::Vector3i bbmin, bbmax;
  bbmin = map_->mp_->box_min_ + map_->mp_->box_min_inflate_;
  bbmax = map_->mp_->box_max_ + map_->mp_->box_max_inflate_;

  for (int x = bbmin[0]; x < bbmax[0]; ++x) {
    for (int y = bbmin[1]; y < bbmax[1]; ++y) {
      for (int z = bbmin[2]; z < bbmax[2]; ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] ==
                SDFMap::OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED ||
            map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] ==
                SDFMap::OCC_VOXEL_STATE::OCCUPIED) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
    }
  }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);

  // Output time and known volumn
  // double time_now = (ros::Time::now() - map_start_time_).toSec();
  // double known_volumn = 0;

  // for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
  //   for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
  //     for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
  //       if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
  //           map_->mp_->clamp_min_log_ - 1e-3)
  //         known_volumn += 0.1 * 0.1 * 0.1;
  //     }

  // ofstream file(
  //     "/home/boboyu/workspaces/plan_ws/src/fast_planner/"
  //     "exploration_manager/resource/"
  //     "curve1.txt",
  //     ios::app);
  // file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
}

void MapROS::publishOccuLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // for (int z = min_cut(2); z <= max_cut(2); ++z)
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        int addr = map_->toAddress(x, y, z);
        if (addr < 0 || addr >= map_->md_->occupancy_buffer_.size()) {
          ROS_ERROR("publishOccuLocal index out of range, x: %d, y: %d, z: %d", x, y, z);
          continue;
        }
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] ==
                SDFMap::OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED ||
            map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] ==
                SDFMap::OCC_VOXEL_STATE::OCCUPIED) {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
        // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y,
        // z)] == 1)
        // {
        //   // Inflated occupied cells
        //   Eigen::Vector3d pos;
        //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        //   if (pos(2) > visualization_truncate_height_)
        //     continue;
        //   if (pos(2) < visualization_truncate_low_)
        //     continue;

        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   cloud2.push_back(pt);
        // }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_.publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] ==
            SDFMap::OCC_VOXEL_STATE::UNKNOWN) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishDepth(const std_msgs::Header &head) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_cam_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "camera";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = head.stamp;
  // depth_pub_.publish(cloud_msg);
}

void MapROS::publishALCCandidates() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  geometry_msgs::Point pt;
  for (Pose &pose : map_->active_loop_closure_candidates_) {
    pt.x = pose.position_[0];
    pt.y = pose.position_[1];
    pt.z = pose.position_[2];
    marker.points.push_back(pt);
  }

  alc_candidates_pub_.publish(marker);

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5;

  marker.points.clear();
  for (ActiveLoopClosureCluster &alc_cluster : map_->active_loop_closure_clusters_) {
    // for (Pose &pose : alc_cluster.active_loop_closure_candidates_) {
    //   pt.x = pose.position_[0];
    //   pt.y = pose.position_[1];
    //   pt.z = pose.position_[2];
    //   marker.points.push_back(pt);
    // }

    pt.x = alc_cluster.center_position_[0];
    pt.y = alc_cluster.center_position_[1];
    pt.z = alc_cluster.center_position_[2];
    marker.points.push_back(pt);
  }

  alc_candidates_selected_pub_.publish(marker);
}

void MapROS::publishLoopClosurePoints() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  for (auto point : map_->loop_closure_points_) {
    marker.points.push_back(point);
  }

  loop_closure_pub_.publish(marker);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut =
      map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut =
      map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void MapROS::publishTSDF() {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = -map_->mp_->tsdf_default_truncation_dist_;
  const double max_dist = map_->mp_->tsdf_default_truncation_dist_;

  Eigen::Vector3i bbmin, bbmax;
  bbmin = map_->mp_->box_min_ + map_->mp_->box_min_inflate_;
  bbmax = map_->mp_->box_max_ + map_->mp_->box_max_inflate_;

  for (int x = bbmin[0]; x < bbmax[0]; ++x) {
    for (int y = bbmin[1]; y < bbmax[1]; ++y) {
      for (int z = bbmin[2]; z < bbmax[2]; ++z) {
        Eigen::Vector3i idx(x, y, z);
        Eigen::Vector3d pos = map_->indexToPos(idx);
        int adr = map_->toAddress(idx);

        if (map_->md_->tsdf_weight_buffer_[adr] < 1e-4) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        pt.intensity = (map_->md_->tsdf_value_buffer_[adr] - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  tsdf_all_pub_.publish(cloud_msg);
}

void MapROS::publishTSDFSlice() {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = -map_->mp_->tsdf_default_truncation_dist_;
  const double max_dist = map_->mp_->tsdf_default_truncation_dist_;

  Eigen::Vector3i min_cut =
      map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut =
      map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_,
                                                    map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y) {
      Eigen::Vector3i idx(x, y, tsdf_slice_height_ / map_->mp_->resolution_);
      Eigen::Vector3d pos = map_->indexToPos(idx);
      int adr = map_->toAddress(idx);

      if (map_->md_->tsdf_weight_buffer_[adr] < 1e-4) continue;

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (map_->md_->tsdf_value_buffer_[adr] - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  tsdf_slice_pub_.publish(cloud_msg);
}

void MapROS::publishTSDFOccu() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->tsdf_weight_buffer_[map_->toAddress(x, y, z)] > 1e-4 &&
            map_->md_->tsdf_value_buffer_[map_->toAddress(x, y, z)] <= 0.05) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  tsdf_occu_pub_.publish(cloud_msg);
}

}  // namespace fast_planner
