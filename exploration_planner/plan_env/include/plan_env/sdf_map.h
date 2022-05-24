#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <math.h>
#include <ros/ros.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <tuple>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/marching_cubes.h>
#include <plan_env/perception_utils_map.h>
#include <set_cover_solver/set_cover_solver.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

namespace cv {
class Mat;
}

class RayCaster;

namespace fast_planner {
struct MapParam;
struct MapData;
struct Pose;
struct ActiveLoopClosureCluster;
struct Keyframe;

class MapROS;

typedef Eigen::Vector3i BlockIndex;
typedef int BlockAddress;
typedef int KeyframeAddress;
typedef long int LongAddress;

class SDFMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SDFMap> Ptr;
  typedef std::shared_ptr<const SDFMap> ConstPtr;

  SDFMap();
  ~SDFMap();

  enum class OCCUPANCY { NA = -1, UNKNOWN, FREE, OCCUPIED };
  enum class OCC_VOXEL_STATE {
    UNKNOWN,
    UNKNOWN_FRONTIER_OCCUPIED,
    UNKNOWN_FRONTIER_FREE,
    FREE,
    OCCUPIED
  };
  enum class INTEGRATION_MODE { INTEGRATION, DEINTEGRATION, REINTEGRATION };

  void initMap(ros::NodeHandle &nh);
  void integratePointCloud(
      const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
      const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q,
      const INTEGRATION_MODE &integration_mode = INTEGRATION_MODE::INTEGRATION);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                       const Eigen::Vector3d &camera_pos);
  void inputPointCloudSetCover(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                               const Eigen::Vector3d &camera_pos,
                               const Eigen::Quaterniond &camera_q, const ros::Time t);
  void deintegratePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                             const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q);
  void reintegratePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                             const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q);
  void updateOccupancyFromTsdfAll();
  void updateOccupancyFromTsdfBoundingBoxIndex(const Eigen::Vector3i &bmin,
                                               const Eigen::Vector3i &bmax);
  void updateOccupancyFromTsdfAddressList(const vector<int> &addr_list);
  void updateOccupancyFromTsdfByAddress(const int &addr);

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  void posToIndexRayEnd(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  void posToIndexSetCover(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  void posToIndexBlock(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  void indexToPosSetCover(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  Eigen::Vector3d indexToPos(const Eigen::Vector3i &id);
  Eigen::Vector3d indexToPosSetCover(const Eigen::Vector3i &id);
  void boundIndex(Eigen::Vector3i &id);
  void boundIndexSetCover(Eigen::Vector3i &id);
  int toAddress(const Eigen::Vector3i &id);
  int toAddress(const int &x, const int &y, const int &z);
  int toAddressRayEnd(const Eigen::Vector3i &id);
  int toAddressRayEnd(const int &x, const int &y, const int &z);
  int toAddressSetCover(const Eigen::Vector3i &id);
  int toAddressSetCover(const int &x, const int &y, const int &z);
  int toAddressBlock(const Eigen::Vector3i &id);
  int toAddressBlock(const int &x, const int &y, const int &z);
  Eigen::Vector3i addressToIndexSetCover(const int &addr);
  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &idx);
  bool isInBox(const Eigen::Vector3i &id);
  bool isInBox(const Eigen::Vector3d &pos);
  void boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up);
  OCCUPANCY getOccupancy(const Eigen::Vector3d &pos);
  OCCUPANCY getOccupancy(const Eigen::Vector3i &id);
  void setOccupied(const Eigen::Vector3d &pos, const int &occ = 1);
  int getInflateOccupancy(const Eigen::Vector3d &pos);
  int getInflateOccupancy(const Eigen::Vector3i &id);
  double getDistance(const Eigen::Vector3d &pos);
  double getDistance(const Eigen::Vector3i &id);
  double getInterpDist(const Eigen::Vector3d &pos);
  double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
  void updateESDF3d();
  void resetBuffer();
  void resetOccuBuffer();
  void resetTSDFBuffer();
  void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
  void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset = false);
  double getResolution();
  int getVoxelNum();
  void getFrameCoveredGrids(vector<int> &grid_list, const pcl::PointCloud<pcl::PointXYZ> &points,
                            const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q);
  void getBlockIdxForFrame(Eigen::Vector3i &idx, const Eigen::Vector3d &pos);
  bool getPoseFromPathByTime(const ros::Time &t, const nav_msgs::Path &path, Eigen::Vector3d &p,
                             Eigen::Matrix3d &R);
  bool getPoseByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                     Eigen::Quaterniond &camera_q_correct);
  bool getCorrectPoseByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                            Eigen::Quaterniond &camera_q_correct);
  bool getCorrectPoseFromPathByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                                    Eigen::Quaterniond &camera_q_correct);
  bool getActiveLoopClosureViewpoints(vector<Eigen::Vector3d> &points, vector<double> &yaws,
                                      vector<vector<Eigen::Vector3d>> &viewpoints_pos,
                                      vector<vector<double>> &viewpoints_yaw);
  double computeDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &point,
                         const Eigen::Vector3d &voxel);
  void updateTsdfVoxel(const int &adr, const double &sdf, const double &weight = 1.0);
  void publishCoveredGrids(const vector<int> &grid_list);
  void publishAllCoveredGrids();
  void publishAllKeyframes();
  void publishAllKeyframesByBlock();
  void publishErrorLine(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2);
  void publishKeyframe(const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q);
  void publishAllBlocks();
  void publishBlock(const Eigen::Vector3d &center_pos);
  void publishBoundingBox(const Eigen::Vector3d &bmin, const Eigen::Vector3d &bmax);
  void publishKeyframeNumber();
  void publishCompensatedPose(const Eigen::Vector3d &camera_pos,
                              const Eigen::Quaterniond &camera_q);
  void publishViewpointsWithYaw(const vector<Eigen::Vector3d> &points, const vector<double> &yaws);
  void publishCenterViewpointsWithYaw(const vector<Eigen::Vector3d> &points,
                                      const vector<double> &yaws);
  void publishSingleViewpointsWithYaw(const Eigen::Vector3d &point, const double &yaw);
  void publishRay(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                  const vector<Eigen::Vector3d> &list3);
  bool saveTSDFCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool evalCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool handleLoopClosure();
  bool handleLoopClosureSimulation();
  bool saveTSDFMesh();
  // void saveTsdfMapPCL();
  void eval();
  void eval2();

 private:
  void clearAndInflateLocalMap();
  void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  void setCacheOccupancy(const int &adr, const int &occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  unique_ptr<MapParam> mp_;
  unique_ptr<MapData> md_;
  unique_ptr<MapROS> mr_;
  unique_ptr<RayCaster> caster_;
  unique_ptr<RayCaster> set_cover_caster_;

  std::atomic<int> atomic_voxel_idx_;
  int thread_num_;
  std::mutex mutex_, integration_mutex_, path_mutex_, map_mutex_, kf_buffer_mutex_;
  std::array<std::mutex, 1024> mutexes_, block_mutexes_;

  SetCoverSolver set_cover_solver_;
  vector<Keyframe> keyframe_list_;
  // saves correct poses  at all time, used for query after loop closure
  vector<pair<double, Eigen::Matrix4d>> correct_pose_vector_;
  vector<pair<double, Eigen::Matrix4d>> correct_pose_vector_sync_;
  vector<Pose> pose_graph_;
  nav_msgs::Path path_opt_, pose_graph_gt_, path_loop_;
  boost::shared_ptr<nav_msgs::Path const> path_loop_ptr_;

  pcl::PointCloud<pcl::PointXYZ> keyframe_points;
  pcl::PointCloud<pcl::PointXYZ> cloud_gt_;
  int set_cover_frame_count_, set_cover_th_;
  double set_cover_gain_threshold_;
  bool set_cover_enable_random_shuffle_;
  ros::Publisher tmp_grid_pub_, tmp_all_grid_pub_, kf_num_pub_, keyframe_fov_pub_,
      keyframe_fov_block_pub_, block_all_pub_, bbox_pub_, pose_compensated_pub_,
      viewpoints_pos_pub_, viewpoints_yaw_pub_, center_viewpoints_pos_pub_,
      center_viewpoints_yaw_pub_, time_pub_, pose_gt_pub_, mesh_pub_, ray_pub_, ray_pt_pub_,
      error_line_pub_;
  ros::ServiceServer loop_closure_trigger_, loop_closure_block_trigger_, save_tsdf_trigger_,
      eval_result_trigger_;
  shared_ptr<PerceptionUtilsMap> percep_utils_;
  int tmp_block_count_;
  Eigen::Vector3d camera_pos_latest_, pos_loop_compensation_;
  Eigen::Quaterniond camera_q_latest, q_loop_compensation_;
  ros::Time lastest_pose_time_stamp_;
  // vector<pair<BlockIndex, KeyframeAddress>> reintegration_vector_, deintegration_vector_;

  // For simulation mock loop closure use
  vector<Pose> pose_graph_drift_, pose_graph_loop_;  // record the history poses at around 10hz
  vector<geometry_msgs::Point> loop_closure_points_;
  vector<Pose> active_loop_closure_candidates_;
  vector<Pose> active_loop_closure_candidates_selected_;
  vector<ActiveLoopClosureCluster> active_loop_closure_clusters_;
  std::mutex alc_mutex_, bbox_mutex_, occu_mutex_, esdf_mutex_;

  double integration_time_, max_integration_time_;
  int integration_num_;

  ros::Timer reintegration_timer_;

  bool save_scp;

  MarchingCubes mesher;
  string mesh_dir_;

  friend MapROS;
};

struct MapParam {
  // map properties
  Eigen::Vector3d map_origin_, map_size_, block_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3d block_pos_min_, block_pos_max_;
  Eigen::Vector3i map_voxel_num_, map_set_cover_grid_num_, block_num_;
  double resolution_, resolution_inv_, set_cover_resolution_, set_cover_resolution_inv_;
  double obstacles_inflation_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_, box_min_inflate_, box_max_inflate_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;  // logit
  double max_ray_length_;
  double local_bound_inflate_;
  int local_map_margin_;
  int voxels_per_block_;
  int max_reintegration_num_;  // maximum number of reintegration frames for each frame arrived
  double unknown_flag_;
  double tsdf_update_weight_, tsdf_default_truncation_dist_, tsdf_occu_th_, tsdf_observation_th_,
      rayend_sample_scale_;
  int num_visb_set_cover_;
  double alc_max_duration_, loop_max_duration_;
};

// TODO: cross-block ray-casting
struct BlockData {
  double time_stamp_, resolution_;  // time of block creation
  Eigen::Vector3d block_size_, center_pos_;
  BlockIndex block_idx_;                      // might change to some hash table method later
  BlockIndex block_idx_min_, block_idx_max_;  // might change to some hash table method later

  // std::vector<double> occupancy_buffer_;
  // std::vector<double> tsdf_value_buffer_;
  // std::vector<double> tsdf_weight_buffer_;
  std::vector<BlockIndex> neighbours_;  // record the idx of neighbour blocks, 6 or 26
  std::vector<Keyframe> keyframe_buffer_;

  bool need_scp_flag_, is_visited;
  int available_frame_size_;

  Eigen::Vector3d set_cover_update_min_, set_cover_update_max_;

  BlockData(const double &stamp, const double &resolution, const Eigen::Vector3d &size,
            const Eigen::Vector3d &pos, const Eigen::Vector3i &idx,
            const Eigen::Vector3i &block_idx_min, const Eigen::Vector3i &block_idx_max,
            const int &voxel_num)
      : time_stamp_(stamp),
        resolution_(resolution),
        block_size_(size),
        center_pos_(pos),
        block_idx_(idx),
        block_idx_min_(block_idx_min),
        block_idx_max_(block_idx_max),
        need_scp_flag_(false),
        is_visited(false),
        available_frame_size_(0) {
    // occupancy_buffer_.resize(voxel_num);
    // tsdf_value_buffer_.resize(voxel_num);
    // tsdf_weight_buffer_.resize(voxel_num);

    set_cover_update_min_ =
        Vector3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::max());

    set_cover_update_max_ =
        Vector3d(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
                 -std::numeric_limits<double>::max());

    // for (int x : {-1, 1}) {
    //   if (block_idx_[0] + x >= 0 && block_idx_[0] + x <= 10)
    //     neighbours_.push_back(block_idx_ + Eigen::Vector3i(x, 0, 0));
    // }

    // for (int y : {-1, 1}) {
    //   if (block_idx_[1] + y >= 0 && block_idx_[1] + y <= 10)
    //     neighbours_.push_back(block_idx_ + Eigen::Vector3i(0, y, 0));
    // }

    // for (int z : {-1, 1}) {
    //   if (block_idx_[2] + z >= 0 && block_idx_[2] + z <= 2)
    //     neighbours_.push_back(block_idx_ + Eigen::Vector3i(0, 0, z));
    // }

    for (int x : {-1, 0, 1}) {
      for (int y : {-1, 0, 1}) {
        for (int z : {-1, 0, 1}) {
          BlockIndex neighbour_idx = block_idx_ + Eigen::Vector3i(x, y, z);
          bool valid = true;
          for (int i = 0; i < 3; ++i) {
            if (neighbour_idx[i] < block_idx_min_[i] || neighbour_idx[i] >= block_idx_max_[i]) {
              valid = false;
            }
          }
          if (valid) neighbours_.push_back(neighbour_idx);
        }
      }
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapData {
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<SDFMap::OCC_VOXEL_STATE> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<double> tsdf_value_buffer_;
  std::vector<double> tsdf_weight_buffer_;
  std::vector<char> set_cover_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  std::vector<BlockData> block_buffer_;
  // data for updating
  vector<short> count_hit_, count_miss_, count_hit_and_miss_;
  vector<int> flag_rayend_, flag_visited_;
  int raycast_num_;
  queue<int> cache_voxel_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  Eigen::Vector3d frame_update_min_, frame_update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// For posegraph
struct Pose {
  ros::Time time_stamp_;
  // Eigen::Vector4d transformation_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond quaternion_;

  Pose() {}

  Pose(const ros::Time &t, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
      : time_stamp_(t), position_(pos), quaternion_(q) {}

  Pose(const geometry_msgs::PoseStamped &pose) : time_stamp_(pose.header.stamp) {
    tf::pointMsgToEigen(pose.pose.position, position_);
    tf::quaternionMsgToEigen(pose.pose.orientation, quaternion_);
  }
};

struct ActiveLoopClosureCluster {
  ros::Time time_stamp_;
  Eigen::Vector3d center_position_;
  Eigen::Quaterniond center_quaternion_;
  double center_yaw_;

  vector<Pose> active_loop_closure_candidates_;  // viewpoints ?

  ActiveLoopClosureCluster(const ros::Time &t, const vector<Pose> &candidates)
      : time_stamp_(t), active_loop_closure_candidates_(candidates) {
    int size = candidates.size();

    center_position_ = candidates[size / 2].position_;
    center_quaternion_ = candidates[size / 2].quaternion_;
    center_yaw_ = atan2(2 * (center_quaternion_.w() * center_quaternion_.z() +
                             center_quaternion_.x() * center_quaternion_.y()),
                        1 - 2 * (center_quaternion_.y() * center_quaternion_.y() +
                                 center_quaternion_.z() * center_quaternion_.z()));
  }
};

enum class FRAME_STATE {
  INTEGRATED,
  DEINTEGRATED,
  WAITING_FOR_INTEGRATION,
  WAITING_FOR_DEINTEGRATION,
  WAITING_FOR_REINTEGRATION,
  DELETED
};

struct Keyframe {
  ros::Time time_stamp_;
  FRAME_STATE state_;

  BlockIndex block_idx_;
  Eigen::Vector3d update_min_, update_max_;
  Eigen::Vector3d camera_pos_, camera_pos_correct_;
  Eigen::Quaterniond camera_q_, camera_q_correct_;
  pcl::PointCloud<pcl::PointXYZ> points_;
  vector<int> covered_grids_list_;
  int proj_points_cnt_;
  double cost_;
  bool is_corrected_;

  // only available if the state is WAITING_FOR_REINTEGRATION
  // KeyframeAddress reintegration_vector_addr_;

  Keyframe(const ros::Time t, const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_q,
           pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
           const Eigen::Vector3d &bmin, const Eigen::Vector3d &bmax)
      : time_stamp_(t),
        camera_pos_(camera_pos),
        camera_q_(camera_q),
        proj_points_cnt_(point_num),
        cost_(1.0),
        update_min_(bmin),
        update_max_(bmax) {
    points_ = std::move(points);
    is_corrected_ = false;
  }
};

inline void SDFMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

inline void SDFMap::posToIndexRayEnd(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i)
    id(i) =
        floor((pos(i) - mp_->map_origin_(i)) * (mp_->rayend_sample_scale_ * mp_->resolution_inv_));
}

inline void SDFMap::posToIndexSetCover(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->set_cover_resolution_inv_);
}

inline void SDFMap::posToIndexBlock(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_->block_pos_min_(i)) / mp_->block_size_[i]);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::indexToPosSetCover(const Eigen::Vector3i &id, Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->set_cover_resolution_ + mp_->map_origin_(i);
}

inline Eigen::Vector3d SDFMap::indexToPos(const Eigen::Vector3i &id) {
  Eigen::Vector3d pos;
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
  return pos;
}

inline Eigen::Vector3d SDFMap::indexToPosSetCover(const Eigen::Vector3i &id) {
  Eigen::Vector3d pos;
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->set_cover_resolution_ + mp_->map_origin_(i);
  return pos;
}

inline void SDFMap::boundIndex(Eigen::Vector3i &id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline void SDFMap::boundIndexSetCover(Eigen::Vector3i &id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_set_cover_grid_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_set_cover_grid_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_set_cover_grid_num_(2) - 1), 0);
  id = id1;
}

inline int SDFMap::toAddress(const int &x, const int &y, const int &z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i &id) { return toAddress(id[0], id[1], id[2]); }

inline int SDFMap::toAddressRayEnd(const int &x, const int &y, const int &z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) * mp_->rayend_sample_scale_ *
             mp_->rayend_sample_scale_ +
         y * mp_->map_voxel_num_(2) * mp_->rayend_sample_scale_ + z;
}

inline int SDFMap::toAddressRayEnd(const Eigen::Vector3i &id) {
  return toAddressRayEnd(id[0], id[1], id[2]);
}

inline int SDFMap::toAddressSetCover(const int &x, const int &y, const int &z) {
  return x * mp_->map_set_cover_grid_num_(1) * mp_->map_set_cover_grid_num_(2) +
         y * mp_->map_set_cover_grid_num_(2) + z;
}

inline int SDFMap::toAddressSetCover(const Eigen::Vector3i &id) {
  return toAddressSetCover(id[0], id[1], id[2]);
}

inline int SDFMap::toAddressBlock(const int &x, const int &y, const int &z) {
  return x * mp_->block_num_(1) * mp_->block_num_(2) + y * mp_->block_num_(2) + z;
}

inline int SDFMap::toAddressBlock(const Eigen::Vector3i &id) {
  return toAddressBlock(id[0], id[1], id[2]);
}

inline Eigen::Vector3i SDFMap::addressToIndexSetCover(const int &addr) {
  int x, y, z;

  x = (addr / mp_->map_set_cover_grid_num_(2)) / mp_->map_set_cover_grid_num_(1);
  y = (addr / mp_->map_set_cover_grid_num_(2)) % mp_->map_set_cover_grid_num_(1);
  z = addr % mp_->map_set_cover_grid_num_(2);

  return Eigen::Vector3i(x, y, z);
}

inline bool SDFMap::isInMap(const Eigen::Vector3d &pos) {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i &idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
      idx(2) > mp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
      return false;
    }
  }
  return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up) {
  for (int i = 0; i < 3; ++i) {
    low[i] = max(low[i], mp_->box_mind_[i]);
    up[i] = min(up[i], mp_->box_maxd_[i]);
  }
}

inline SDFMap::OCCUPANCY SDFMap::getOccupancy(const Eigen::Vector3i &id) {
  if (!isInMap(id)) return OCCUPANCY::NA;
  // double occ = md_->occupancy_buffer_[toAddress(id)];
  SDFMap::OCC_VOXEL_STATE occ = md_->occupancy_buffer_[toAddress(id)];
  // if (occ < mp_->clamp_min_log_ - 1e-3) return OCCUPANCY::UNKNOWN;
  // if (occ > mp_->min_occupancy_log_) return OCCUPANCY::OCCUPIED;
  if (occ == SDFMap::OCC_VOXEL_STATE::UNKNOWN ||
      occ == SDFMap::OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED ||
      occ == SDFMap::OCC_VOXEL_STATE::UNKNOWN_FRONTIER_FREE)
    return OCCUPANCY::UNKNOWN;
  if (occ == SDFMap::OCC_VOXEL_STATE::OCCUPIED) return OCCUPANCY::OCCUPIED;
  return OCCUPANCY::FREE;
}

inline SDFMap::OCCUPANCY SDFMap::getOccupancy(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

inline void SDFMap::setOccupied(const Eigen::Vector3d &pos, const int &occ) {
  if (!isInMap(pos)) return;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i &id) {
  if (!isInMap(id)) return -1;
  return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}

inline double SDFMap::getDistance(const Eigen::Vector3i &id) {
  if (!isInMap(id)) return -1;

  std::lock_guard<std::mutex> lock(esdf_mutex_);
  return md_->distance_buffer_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance(id);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i &pt, int step,
                                 vector<Eigen::Vector3i> &pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}
}  // namespace fast_planner
#endif