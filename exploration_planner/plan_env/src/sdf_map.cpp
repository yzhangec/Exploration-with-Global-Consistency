#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include "plan_env/raycast.h"

namespace fast_planner {
SDFMap::SDFMap() {}

SDFMap::~SDFMap() {}

void SDFMap::initMap(ros::NodeHandle &nh) {
  mp_.reset(new MapParam);
  md_.reset(new MapData);
  mr_.reset(new MapROS);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
  nh.param("sdf_map/set_cover_resolution", mp_->set_cover_resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
  nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
  nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
  nh.param("sdf_map/optimistic", mp_->optimistic_, true);
  nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);
  nh.param("sdf_map/thread_num", thread_num_, 1);
  nh.param("sdf_map/set_cover_th", set_cover_th_, 100);
  nh.param("sdf_map/set_cover_gain_threshold", set_cover_gain_threshold_, 100.0);
  nh.param("sdf_map/set_cover_enable_random_shuffle", set_cover_enable_random_shuffle_, false);
  nh.param("sdf_map/num_visb_set_cover", mp_->num_visb_set_cover_, 1);
  nh.param("sdf_map/max_reintegration_num", mp_->max_reintegration_num_, 10);
  nh.param("sdf_map/mesh_dir", mesh_dir_, string("null"));

  mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  mp_->set_cover_resolution_inv_ = 1 / mp_->set_cover_resolution_;
  mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
  mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i) mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
  for (int i = 0; i < 3; ++i)
    mp_->map_set_cover_grid_num_(i) = ceil(mp_->map_size_(i) / mp_->set_cover_resolution_);
  mp_->map_min_boundary_ = mp_->map_origin_;
  mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);
  nh.param("sdf_map/tsdf_update_weight", mp_->tsdf_update_weight_, 1.0);
  nh.param("sdf_map/tsdf_default_truncation_dist", mp_->tsdf_default_truncation_dist_, 0.1);
  nh.param("sdf_map/tsdf_occu_th", mp_->tsdf_occu_th_, 0.1);
  nh.param("sdf_map/tsdf_observation_th", mp_->tsdf_observation_th_, 1e-4);
  nh.param("sdf_map/rayend_sample_scale", mp_->rayend_sample_scale_, 1.0);
  nh.param("sdf_map/alc_max_duration", mp_->alc_max_duration_, 15.0);
  nh.param("sdf_map/loop_max_duration", mp_->loop_max_duration_, 30.0);

  auto logit = [](const double &x) { return log(x / (1 - x)); };
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->unknown_flag_ = 0.01;
  cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
       << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
       << ", thresh: " << mp_->min_occupancy_log_ << endl;

  // Initialize data buffer of map
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  int set_cover_buffer_size = mp_->map_set_cover_grid_num_(0) * mp_->map_set_cover_grid_num_(1) *
                              mp_->map_set_cover_grid_num_(2);
  md_->occupancy_buffer_ = vector<OCC_VOXEL_STATE>(buffer_size, OCC_VOXEL_STATE::UNKNOWN);
  md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_->tsdf_value_buffer_ = vector<double>(buffer_size, 1.0);
  md_->tsdf_weight_buffer_ = vector<double>(buffer_size, 0.0);
  md_->set_cover_buffer_ = vector<char>(set_cover_buffer_size, 0);
  md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<int>(buffer_size * pow(mp_->rayend_sample_scale_, 3), -1);
  md_->flag_visited_ = vector<int>(buffer_size, -1);
  md_->tmp_buffer1_ = vector<double>(buffer_size, 0.0);
  md_->tmp_buffer2_ = vector<double>(buffer_size, 0.0);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;

  md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

  ROS_WARN_STREAM("Buffer size: " << buffer_size);

  // Try retriving bounding box of map, set box to map size
  // if not specified
  vector<string> axis = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
    nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);

    nh.param("sdf_map/box_min_inflate_" + axis[i], mp_->box_min_inflate_[i], 0);
    nh.param("sdf_map/box_max_inflate_" + axis[i], mp_->box_max_inflate_[i], 0);

    nh.param("sdf_map/block_size_" + axis[i], mp_->block_size_[i], 5.0);
    mp_->block_num_[i] = round((mp_->map_size_[i] / 2.0) / mp_->block_size_[i]) * 2 + 1;
  }

  ROS_WARN_STREAM("box_min_inflate_" << mp_->box_min_inflate_.transpose());

  posToIndex(mp_->box_mind_, mp_->box_min_);
  posToIndex(mp_->box_maxd_, mp_->box_max_);

  mp_->voxels_per_block_ = (mp_->block_size_[0] / mp_->resolution_) *
                           (mp_->block_size_[1] / mp_->resolution_) *
                           (mp_->block_size_[2] / mp_->resolution_);

  Eigen::Vector3i block_idx_min, block_idx_max;

  for (int i = 0; i < 3; ++i) {
    block_idx_min[i] = -(mp_->block_num_[i] - 1) / 2;
    block_idx_max[i] = (mp_->block_num_[i] - 1) / 2;

    mp_->block_pos_min_[i] = block_idx_min[i] * mp_->block_size_[i] - mp_->block_size_[i] / 2.0;
    mp_->block_pos_max_[i] = block_idx_max[i] * mp_->block_size_[i] + mp_->block_size_[i] / 2.0;
  }

  for (int idx_x = block_idx_min[0]; idx_x <= block_idx_max[0]; idx_x++) {
    for (int idx_y = block_idx_min[1]; idx_y <= block_idx_max[1]; idx_y++) {
      for (int idx_z = block_idx_min[2]; idx_z <= block_idx_max[2]; idx_z++) {
        md_->block_buffer_.push_back(BlockData(
            0.0, mp_->resolution_, mp_->block_size_,
            Eigen::Vector3d(idx_x * mp_->block_size_[0], idx_y * mp_->block_size_[1],
                            idx_z * mp_->block_size_[2]),
            Eigen::Vector3i(idx_x, idx_y, idx_z) - block_idx_min, Eigen::Vector3i(0, 0, 0),
            block_idx_max - block_idx_min, mp_->voxels_per_block_));
      }
    }
  }

  // Initialize ROS wrapper
  mr_->setMap(this);
  mr_->node_ = nh;
  mr_->init();

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);

  set_cover_caster_.reset(new RayCaster);
  set_cover_caster_->setParams(mp_->set_cover_resolution_, mp_->map_origin_);

  atomic_voxel_idx_ = 0;
  set_cover_frame_count_ = 0;

  tmp_grid_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/covered_grids", 10);
  tmp_all_grid_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/all_covered_grids", 10);
  kf_num_pub_ = nh.advertise<std_msgs::Int32>("/keyframe_num", 10);
  keyframe_fov_pub_ = nh.advertise<visualization_msgs::Marker>("/keyframe_fov", 10);
  keyframe_fov_block_pub_ = nh.advertise<visualization_msgs::Marker>("/keyframe_fov_block", 10);
  block_all_pub_ = nh.advertise<visualization_msgs::Marker>("/all_blocks", 10);
  bbox_pub_ = nh.advertise<visualization_msgs::Marker>("/bounding_box", 10);
  pose_compensated_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/sdf_map/compensated_pose", 10);
  viewpoints_pos_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/viewpoints_pos", 10);
  viewpoints_yaw_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/viewpoints_yaw", 10);
  center_viewpoints_pos_pub_ =
      nh.advertise<visualization_msgs::Marker>("/sdf_map/center_viewpoints_pos", 10);
  center_viewpoints_yaw_pub_ =
      nh.advertise<visualization_msgs::Marker>("/sdf_map/center_viewpoints_yaw", 10);
  time_pub_ = nh.advertise<std_msgs::Float64>("/sdf_map/time", 10);
  pose_gt_pub_ = nh.advertise<nav_msgs::Path>("/sdf_map/path_gt_sim", 10);
  mesh_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/mesh", 10);
  ray_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/rays", 10);
  ray_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/ray_pt", 10);
  error_line_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/error_line", 10);
  save_tsdf_trigger_ = nh.advertiseService("save_tsdf", &SDFMap::saveTSDFCallback, this);
  eval_result_trigger_ = nh.advertiseService("eval", &SDFMap::evalCallback, this);

  percep_utils_.reset(new PerceptionUtilsMap(nh));

  save_scp = false;
  integration_time_ = 0.0;
  max_integration_time_ = 0.0;
  integration_num_ = 0;
}

void SDFMap::resetBuffer() {
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetOccuBuffer() {
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->occupancy_buffer_.assign(buffer_size, OCC_VOXEL_STATE::UNKNOWN);
}

void SDFMap::resetTSDFBuffer() {
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->tsdf_value_buffer_.assign(buffer_size, 0.0);
  md_->tsdf_weight_buffer_.assign(buffer_size, 0.0);
}

void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  // std::lock_guard<std::mutex> lock(esdf_mutex_);
  if (!esdf_mutex_.try_lock()) {
    ROS_WARN("Cannot get the lock for esdf");
    return;
  } else {
    Eigen::Vector3i min_esdf = md_->local_bound_min_;
    Eigen::Vector3i max_esdf = md_->local_bound_max_;

    if (mp_->optimistic_) {
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
          fillESDF(
              [&](int z) {
                return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1
                           ? 0
                           : std::numeric_limits<double>::max();
              },
              [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
              max_esdf[2], 2);
        }
    } else {
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
          fillESDF(
              [&](int z) {
                int adr = toAddress(x, y, z);
                return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                        md_->occupancy_buffer_[adr] == OCC_VOXEL_STATE::UNKNOWN ||
                        md_->occupancy_buffer_[adr] == OCC_VOXEL_STATE::UNKNOWN_FRONTIER_FREE ||
                        md_->occupancy_buffer_[adr] == OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED)
                           ? 0
                           : std::numeric_limits<double>::max();
              },
              [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
              max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                 [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
                 min_esdf[1], max_esdf[1], 1);
      }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                 [&](int x, double val) {
                   md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                 },
                 min_esdf[0], max_esdf[0], 0);
      }

    if (mp_->signed_dist_) {
      // Compute negative distance
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
          fillESDF(
              [&](int z) {
                return md_->occupancy_buffer_inflate_[x * mp_->map_voxel_num_(1) *
                                                          mp_->map_voxel_num_(2) +
                                                      y * mp_->map_voxel_num_(2) + z] == 0
                           ? 0
                           : std::numeric_limits<double>::max();
              },
              [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
              max_esdf[2], 2);
        }
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
          fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                   [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
                   min_esdf[1], max_esdf[1], 1);
        }
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
          fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                   [&](int x, double val) {
                     md_->distance_buffer_neg_[toAddress(x, y, z)] =
                         mp_->resolution_ * std::sqrt(val);
                   },
                   min_esdf[0], max_esdf[0], 0);
        }
      // Merge negative distance with positive
      for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
          for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
            int idx = toAddress(x, y, z);
            if (md_->distance_buffer_neg_[idx] > 0.0)
              md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
          }
    }

    esdf_mutex_.unlock();
  }
}

void SDFMap::setCacheOccupancy(const int &adr, const int &occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;

  // md_->count_hit_and_miss_[adr] += 1;
  // if (occ == 1)
  //   md_->count_hit_[adr] += 1;
  // if (md_->count_hit_and_miss_[adr] == 1)
  //   md_->cache_voxel_.push(adr);
}

void SDFMap::getFrameCoveredGrids(vector<int> &grid_list,
                                  const pcl::PointCloud<pcl::PointXYZ> &points,
                                  const Eigen::Vector3d &camera_pos,
                                  const Eigen::Quaterniond &camera_q) {
  grid_list.clear();
  // uses ray casting to get all covered grids

  // md_->raycast_num_ += 1;
  Eigen::Matrix3d camera_r = camera_q.toRotationMatrix();
  Eigen::Vector3d pt_w, pt_c, vox_pos, ray_caster_end_point;
  Eigen::Vector3i idx;
  int vox_adr;
  double length, sdf;

  for (int i = 0; i < points.size(); ++i) {
    auto &pt = points.points[i];
    pt_c << pt.x, pt.y, pt.z;
    pt_w = camera_r * pt_c + camera_pos;
    // pt_w = (pt_w - camera_pos) * 1.5 + camera_pos;
    if (!isInMap(pt_w)) pt_w = closetPointInMap(pt_w, camera_pos);
    posToIndexSetCover(pt_w, idx);
    vox_adr = toAddressSetCover(idx);
    vox_pos = indexToPosSetCover(idx);

    // if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
    //   continue;
    // else
    //   md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    ray_caster_end_point = pt_w;
    length = (pt_w - camera_pos).norm();
    if (length > mp_->max_ray_length_)
      ray_caster_end_point = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;

    set_cover_caster_->input(ray_caster_end_point, camera_pos);

    while (set_cover_caster_->nextId(idx)) {
      vox_adr = toAddressSetCover(idx);
      grid_list.push_back(vox_adr);
    }
  }

  std::sort(grid_list.begin(), grid_list.end());
  auto it = std::unique(grid_list.begin(), grid_list.end());
  grid_list.erase(it, grid_list.end());
  // publishCoveredGrids(grid_list);
}

void SDFMap::getBlockIdxForFrame(Eigen::Vector3i &idx, const Eigen::Vector3d &pos) {
  posToIndexBlock(pos, idx);
}

double SDFMap::computeDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &point,
                               const Eigen::Vector3d &voxel) {
  const Eigen::Vector3d v_voxel_origin = voxel - origin;
  const Eigen::Vector3d v_point_origin = point - origin;

  const double dist_G = v_point_origin.norm();
  // projection of a (v_voxel_origin) onto b (v_point_origin)
  const double dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  double sdf = static_cast<double>(dist_G - dist_G_V);
  sdf = (sdf > 0.0) ? std::min(mp_->tsdf_default_truncation_dist_, sdf)
                    : std::max(-mp_->tsdf_default_truncation_dist_, sdf);

  return sdf;
}

void SDFMap::updateTsdfVoxel(const int &adr, const double &sdf, const double &weight) {
  if (md_->tsdf_weight_buffer_[adr] + weight < 1e-4) {
    md_->tsdf_value_buffer_[adr] = 1.0;
    md_->tsdf_weight_buffer_[adr] = 0.0;
    return;
  }

  if (md_->tsdf_weight_buffer_[adr] == 0) md_->tsdf_value_buffer_[adr] = 0;

  md_->tsdf_value_buffer_[adr] =
      (md_->tsdf_value_buffer_[adr] * md_->tsdf_weight_buffer_[adr] + sdf * weight) /
      (md_->tsdf_weight_buffer_[adr] + weight);
  md_->tsdf_weight_buffer_[adr] = md_->tsdf_weight_buffer_[adr] + weight;

  md_->tsdf_value_buffer_[adr] =
      (md_->tsdf_value_buffer_[adr] > 0.0)
          ? std::min(mp_->tsdf_default_truncation_dist_, md_->tsdf_value_buffer_[adr])
          : std::max(-mp_->tsdf_default_truncation_dist_, md_->tsdf_value_buffer_[adr]);
  md_->tsdf_weight_buffer_[adr] = std::min(10000.0, md_->tsdf_weight_buffer_[adr]);
}

// used for set cover reintegration
void SDFMap::integratePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                                 const Eigen::Vector3d &camera_pos,
                                 const Eigen::Quaterniond &camera_q,
                                 const INTEGRATION_MODE &integration_mode) {
  md_->raycast_num_ += 1;

  if (integration_mode == INTEGRATION_MODE::INTEGRATION) {
    // if (integration_mode != INTEGRATION_MODE::DEINTEGRATION) {
    md_->frame_update_min_ = camera_pos;
    md_->frame_update_max_ = camera_pos;
    bbox_mutex_.lock();
    if (md_->reset_updated_box_) {
      md_->update_min_ = camera_pos;
      md_->update_max_ = camera_pos;
      md_->reset_updated_box_ = false;
    }
    bbox_mutex_.unlock();
  }

  Eigen::Matrix3d camera_r = camera_q.toRotationMatrix();
  Eigen::Vector3d pt_w, pt_c, vox_pos, ray_caster_start_point, ray_caster_end_point, dir;
  Eigen::Vector3i idx;
  int vox_adr;
  double length, sdf, weight;

  keyframe_points.clear();

  ros::Time integrate_begin = ros::Time::now();

  // double t1_total = 0.0, t2_total = 0.0, t3_total = 0.0;
  // static int ray_cnt = 0;
  // vector<Eigen::Vector3d> list1, list2, list3;
  for (int i = 0; i < point_num; ++i) {
    auto &pt = points.points[i];
    pt_c << pt.x, pt.y, pt.z;
    pt_w = camera_r * pt_c + camera_pos;

    if (!isInMap(pt_w)) pt_w = closetPointInMap(pt_w, camera_pos);

    // if (pt_w[2] < 0.2) continue;

    // find the voxel idx/adr/pos for the point in
    // pointcloud
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);

    // avoid the case that multiple ray ends at same voxel
    // (treated as same ray)
    // if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
    //   continue;
    // else
    //   md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    pcl::PointXYZ pt_kf;
    pt_kf.x = pt_c[0];
    pt_kf.y = pt_c[1];
    pt_kf.z = pt_c[2];
    keyframe_points.push_back(pt_kf);

    length = (pt_w - camera_pos).norm();
    dir = (pt_w - camera_pos) / length;

    ray_caster_end_point = pt_w + dir * mp_->tsdf_default_truncation_dist_;

    if (length > mp_->max_ray_length_) {
      length = std::min(std::max(length - mp_->tsdf_default_truncation_dist_, 0.0),
                        mp_->max_ray_length_);
      ray_caster_end_point = camera_pos + dir * length;
    }
    // ray_caster_end_point = camera_pos + dir * mp_->max_ray_length_;

    if (!isInMap(ray_caster_end_point)) ray_caster_end_point = pt_w;

    if (integration_mode == INTEGRATION_MODE::INTEGRATION) {
      // if (integration_mode != INTEGRATION_MODE::DEINTEGRATION) {
      for (int k = 0; k < 3; ++k) {
        md_->frame_update_min_[k] = min(md_->frame_update_min_[k], ray_caster_end_point[k]);
        md_->frame_update_max_[k] = max(md_->frame_update_max_[k], ray_caster_end_point[k]);
      }
    }

    double z_dist = std::abs(pt_c.z());
    double simple_weight = 1.0 / (z_dist * z_dist), dropoff_weight, updated_weight;
    caster_->input(ray_caster_end_point, camera_pos);

    // if (ray_cnt % 100 == 0) {
    //   list1.push_back(camera_pos);
    //   list2.push_back(ray_caster_end_point);
    // }
    while (caster_->nextId(idx)) {
      vox_pos = indexToPos(idx);
      vox_adr = toAddress(idx);

      // if (ray_cnt % 100 == 0) list3.push_back(vox_pos);

      sdf = computeDistance(camera_pos, pt_w, vox_pos);

      if (sdf < -mp_->resolution_) {
        dropoff_weight = simple_weight * (mp_->tsdf_default_truncation_dist_ + sdf) /
                         (mp_->tsdf_default_truncation_dist_ - mp_->resolution_);
        dropoff_weight = std::max(dropoff_weight, 0.0);
      } else {
        dropoff_weight = simple_weight;
      }

      updated_weight =
          (integration_mode == INTEGRATION_MODE::DEINTEGRATION) ? -dropoff_weight : dropoff_weight;

      updateTsdfVoxel(vox_adr, sdf, updated_weight);
      updateOccupancyFromTsdfByAddress(vox_adr);
    }

    // caster_->input(ray_caster_start_point, camera_pos);
    // while (caster_->nextId(idx)) {
    //   vox_pos = indexToPos(idx);
    //   vox_adr = toAddress(idx);

    //   // // ray casting early terminate
    //   // if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
    //   //   break;
    //   // else
    //   //   md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    //   sdf = mp_->tsdf_default_truncation_dist_;
    //   weight = integration_mode == 2 ? -1.0 : 1.0;
    //   updateTsdfVoxel(vox_adr, sdf, weight);
    //   updateOccupancyFromTsdfByAddress(vox_adr);
    // }
    // ray_cnt++;
  }
  // publishRay(list1, list2, list3);
  ros::Time integrate_end = ros::Time::now();

  // cout << "Integration time for one frame: " << (integrate_end - integrate_begin).toSec() <<
  // endl; cout << "point cloud size: " << point_num << endl; cout << "pruned point cloud size: " <<
  // keyframe_points.size() << endl;

  if (integration_mode == INTEGRATION_MODE::INTEGRATION) {
    // if (integration_mode != INTEGRATION_MODE::DEINTEGRATION) {
    Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
    posToIndex(md_->frame_update_min_ - bound_inf, md_->local_bound_min_);
    posToIndex(md_->frame_update_max_ + bound_inf, md_->local_bound_max_);
    boundIndex(md_->local_bound_min_);
    boundIndex(md_->local_bound_max_);
    mr_->local_updated_ = true;

    // Bounding box for subsequent updating
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    for (int k = 0; k < 3; ++k) {
      md_->update_min_[k] = min(md_->frame_update_min_[k], md_->update_min_[k]);
      md_->update_max_[k] = max(md_->frame_update_max_[k], md_->update_max_[k]);
    }
  }
}

// original function logic for input pointcloud (changed to using tsdf)
void SDFMap::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                             const Eigen::Vector3d &camera_pos) {
  if (point_num == 0) return;
  md_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp, vox_pos;
  Eigen::Vector3i idx;
  int vox_adr;
  double length, tsdf_value, tsdf_offset = 5e-2;

  ros::Time integrate_begin = ros::Time::now();

  for (int i = 0; i < point_num; ++i) {
    auto &pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    vox_pos = indexToPos(idx);
    // setCacheOccupancy(vox_adr, tmp_flag);

    tsdf_value = computeDistance(camera_pos, pt_w, vox_pos);
    if ((vox_pos - camera_pos).norm() < mp_->max_ray_length_ - tsdf_offset)
      updateTsdfVoxel(vox_adr, tsdf_value);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);

    while (caster_->nextId(idx)) {
      // Occu
      // setCacheOccupancy(toAddress(idx), 0);

      // TSDF
      int tsdf_adr = toAddress(idx);
      vox_pos = indexToPos(idx);
      tsdf_value = computeDistance(camera_pos, pt_w, vox_pos);
      if ((vox_pos - camera_pos).norm() < mp_->max_ray_length_ - tsdf_offset)
        updateTsdfVoxel(tsdf_adr, tsdf_value);
    }
  }

  ros::Time integrate_end = ros::Time::now();

  // cout << "Integration time for one frame: " <<
  // integrate_end - integrate_begin
  //      << endl;
  // cout << "Size of pointcloud: " << point_num << endl;

  Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, md_->local_bound_max_);
  posToIndex(update_min - bound_inf, md_->local_bound_min_);
  boundIndex(md_->local_bound_min_);
  boundIndex(md_->local_bound_max_);
  mr_->local_updated_ = true;

  // Bounding box for subsequent updating
  for (int k = 0; k < 3; ++k) {
    md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
    md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
  }
}

// IMPORTANT: the pointcloud here is under CAMERA frame
void SDFMap::inputPointCloudSetCover(const pcl::PointCloud<pcl::PointXYZ> &points,
                                     const int &point_num, const Eigen::Vector3d &camera_pos,
                                     const Eigen::Quaterniond &camera_q, const ros::Time t) {
  if (point_num == 0) return;

  // static int cnt = 0;
  // cnt++;
  // ROS_WARN("Process frames cnt: %d, ts: %f", cnt, t.toSec());

  std::lock_guard<std::mutex> lock(integration_mutex_);

  camera_pos_latest_ = camera_pos;
  camera_q_latest = camera_q;
  lastest_pose_time_stamp_ = t;

  auto t1 = ros::Time::now();
  integratePointCloud(points, point_num, camera_pos, camera_q);
  auto t2 = ros::Time::now();

  integration_time_ += (t2 - t1).toSec();
  max_integration_time_ = max(max_integration_time_, (t2 - t1).toSec());
  integration_num_ += 1;
  if (mr_->show_occ_time_) {
    ROS_WARN("Integration t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(),
             integration_time_ / integration_num_, max_integration_time_);
  }

  // ROS_WARN("Original points size: %d", point_num);
  // ROS_WARN("Keyframe points size: %d", keyframe_points.size());
  Keyframe keyframe(t, camera_pos, camera_q, keyframe_points, keyframe_points.size(),
                    md_->frame_update_min_, md_->frame_update_max_);
  getFrameCoveredGrids(keyframe.covered_grids_list_, keyframe.points_, camera_pos, camera_q);
  getBlockIdxForFrame(keyframe.block_idx_, keyframe.camera_pos_);
  keyframe.state_ = FRAME_STATE::INTEGRATED;
  // publishBoundingBox(keyframe.update_min_, keyframe.update_max_);

  // keyframe_list_.push_back(keyframe);
  BlockData &block_cur = md_->block_buffer_[toAddressBlock(keyframe.block_idx_)];
  block_cur.keyframe_buffer_.push_back(keyframe);
  block_cur.need_scp_flag_ = true;
  block_cur.is_visited = true;
  // block_cur.available_frame_size_++;

  for (int k = 0; k < 3; ++k) {
    block_cur.set_cover_update_min_[k] =
        min(keyframe.update_min_[k], block_cur.set_cover_update_min_[k]);
    block_cur.set_cover_update_max_[k] =
        max(keyframe.update_max_[k], block_cur.set_cover_update_max_[k]);
  }
  // publishBoundingBox(block_cur.set_cover_update_min_, block_cur.set_cover_update_max_);

  set_cover_frame_count_++;

  auto t3 = ros::Time::now();
  if (mr_->show_occ_time_) {
    ROS_WARN("Time cost for keyframe save: %f", (t3 - t2).toSec());
  }

  bool is_scp = false;
  // trigger the set cover solver
  if (set_cover_frame_count_ >= set_cover_th_) {
    auto t_scp_1 = ros::Time::now();

    is_scp = true;
    set_cover_frame_count_ = 0;

    // solve the problem in a block manner
    for (BlockData &block : md_->block_buffer_) {
      if (block.keyframe_buffer_.size() == 0) continue;
      if (!block.need_scp_flag_) continue;

      int num_sets_block;
      int num_elements_block = mp_->map_set_cover_grid_num_(0) * mp_->map_set_cover_grid_num_(1) *
                               mp_->map_set_cover_grid_num_(2);
      vector<pair<KeyframeAddress, vector<int>>> sets_block;
      vector<double> costs_block;  // set all cost to 1 first)

      for (vector<Keyframe>::iterator kf_it = block.keyframe_buffer_.begin();
           kf_it != block.keyframe_buffer_.end(); kf_it++) {
        if (kf_it->state_ == FRAME_STATE::WAITING_FOR_DEINTEGRATION ||
            kf_it->state_ == FRAME_STATE::DEINTEGRATED)
          continue;
        sets_block.push_back(
            make_pair(kf_it - block.keyframe_buffer_.begin(), kf_it->covered_grids_list_));
        costs_block.push_back(kf_it->cost_);
      }

      num_sets_block = sets_block.size();

      if (set_cover_enable_random_shuffle_)
        std::random_shuffle(sets_block.begin(), sets_block.end());

      ROS_WARN_STREAM(
          "[SDF Map] Starting solve set cover for block: " << block.block_idx_.transpose());

      if (save_scp) {
        save_scp = false;
        ofstream scp_file;
        scp_file.open("/home/eason/output/scp.txt");
        Eigen::Vector3i min_idx, max_idx;
        posToIndexSetCover(block.set_cover_update_min_, min_idx);
        posToIndexSetCover(block.set_cover_update_max_, max_idx);
        // ROS_WARN("Total elements num: %d",
        //          (max_idx - min_idx)[0] * (max_idx - min_idx)[1] * (max_idx - min_idx)[2]);

        int num_elements_block_new =
            (max_idx - min_idx)[0] * (max_idx - min_idx)[1] * (max_idx - min_idx)[2];

        scp_file << num_elements_block_new << ' ' << num_sets_block << endl;

        for (auto set : sets_block) {
          scp_file << 1 << ' ';
          for (auto element : set.second) {
            scp_file << element << ' ';
          }
          scp_file << endl;
        }
      }

      SetCoverProblem problem_block(num_sets_block, num_elements_block, mp_->num_visb_set_cover_,
                                    costs_block, sets_block);
      SetCoverSolution solution_block;

      set_cover_solver_.initProblem(problem_block);
      set_cover_solver_.setThreshold(set_cover_gain_threshold_);
      set_cover_solver_.solve();
      set_cover_solver_.getSolution(solution_block);

      for (int i = 0; i < block.keyframe_buffer_.size(); i++) {
        if (!solution_block.masks_[i]) {
          block.keyframe_buffer_[i].state_ = FRAME_STATE::WAITING_FOR_DEINTEGRATION;
        }
      }

      block.need_scp_flag_ = false;

      ROS_WARN_STREAM("[SDF Map] Set cover solved for block: " << block.block_idx_.transpose());
    }

    // publishAllKeyframes();
    if (mr_->show_keyframe_) {
      publishAllKeyframesByBlock();
    }
    // publishAllCoveredGrids();
    auto t_scp_2 = ros::Time::now();

    if (mr_->show_set_cover_time_) {
      double scp_duration = (t_scp_2 - t_scp_1).toSec();
      ROS_WARN("Time cost for set cover: %f", scp_duration);
      std_msgs::Float64 msg;
      msg.data = scp_duration;
      time_pub_.publish(msg);
    }
  }

  auto t4 = ros::Time::now();

  publishKeyframeNumber();

  int process_frame_count = 0;

  if (!is_scp) {
    for (BlockIndex block_idx : block_cur.neighbours_) {
      BlockData &block = md_->block_buffer_[toAddressBlock(block_idx)];
      if (process_frame_count > mp_->max_reintegration_num_) break;
      if (block.keyframe_buffer_.size() == 0) continue;

      bool process_deintegration = false;
      for (Keyframe &keyframe : block.keyframe_buffer_) {
        if (process_frame_count > mp_->max_reintegration_num_) break;

        if (keyframe.state_ == FRAME_STATE::WAITING_FOR_DEINTEGRATION) {
          process_frame_count++;
          process_deintegration = true;
          deintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_, keyframe.camera_pos_,
                                keyframe.camera_q_);
          keyframe.state_ = FRAME_STATE::DEINTEGRATED;
        }

        if (keyframe.state_ == FRAME_STATE::WAITING_FOR_REINTEGRATION) {
          process_frame_count++;
          deintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_, keyframe.camera_pos_,
                                keyframe.camera_q_);
          reintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_,
                                keyframe.camera_pos_correct_, keyframe.camera_q_correct_);
          keyframe.camera_pos_ = keyframe.camera_pos_correct_;
          keyframe.camera_q_ = keyframe.camera_q_correct_;
          keyframe.state_ = FRAME_STATE::INTEGRATED;
          // ROS_WARN("getFrameCoveredGrids frame: ");

          // update frame covered grids
          getFrameCoveredGrids(keyframe.covered_grids_list_, keyframe.points_, keyframe.camera_pos_,
                               keyframe.camera_q_);
        }
      }

      if (process_deintegration) {
        block.keyframe_buffer_.erase(
            remove_if(block.keyframe_buffer_.begin(), block.keyframe_buffer_.end(),
                      [](const Keyframe &kf) { return kf.state_ == FRAME_STATE::DEINTEGRATED; }),
            block.keyframe_buffer_.end());
      }
    }

    // for (BlockIndex block_idx : block_cur.neighbours_) {
    for (BlockData &block : md_->block_buffer_) {
      // BlockData &block = md_->block_buffer_[toAddressBlock(block_idx)];
      if (process_frame_count > mp_->max_reintegration_num_) break;
      if (block.keyframe_buffer_.size() == 0) continue;

      bool process_deintegration = false;
      for (Keyframe &keyframe : block.keyframe_buffer_) {
        if (process_frame_count > mp_->max_reintegration_num_) break;

        if (keyframe.state_ == FRAME_STATE::WAITING_FOR_DEINTEGRATION) {
          process_frame_count++;
          process_deintegration = true;
          deintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_, keyframe.camera_pos_,
                                keyframe.camera_q_);
          keyframe.state_ = FRAME_STATE::DEINTEGRATED;
        }

        if (keyframe.state_ == FRAME_STATE::WAITING_FOR_REINTEGRATION) {
          process_frame_count++;
          deintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_, keyframe.camera_pos_,
                                keyframe.camera_q_);
          reintegratePointCloud(keyframe.points_, keyframe.proj_points_cnt_,
                                keyframe.camera_pos_correct_, keyframe.camera_q_correct_);
          keyframe.camera_pos_ = keyframe.camera_pos_correct_;
          keyframe.camera_q_ = keyframe.camera_q_correct_;
          keyframe.state_ = FRAME_STATE::INTEGRATED;
          // ROS_WARN("getFrameCoveredGrids frame: ");

          // update frame covered grids
          getFrameCoveredGrids(keyframe.covered_grids_list_, keyframe.points_, keyframe.camera_pos_,
                               keyframe.camera_q_);
        }
      }

      if (process_deintegration) {
        block.keyframe_buffer_.erase(
            remove_if(block.keyframe_buffer_.begin(), block.keyframe_buffer_.end(),
                      [](const Keyframe &kf) { return kf.state_ == FRAME_STATE::DEINTEGRATED; }),
            block.keyframe_buffer_.end());
      }
    }
  }

  auto t5 = ros::Time::now();
  if (mr_->show_occ_time_) {
    ROS_WARN("Time cost for de/reintegration: %f", (t5 - t4).toSec());
  }

  // ROS_WARN(
  //     "Total time: %.6lfs, keyframe : %.6lf, set cover: %.6lf, pub txt:
  //     %.6lf, inte pcl: %.6lf", (t5 - t1).toSec(), (t2 - t1).toSec(), (t3 -
  //     t2).toSec(), (t4 - t3).toSec(), (t5 - t4).toSec());
}

void SDFMap::deintegratePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points,
                                   const int &point_num, const Eigen::Vector3d &camera_pos,
                                   const Eigen::Quaterniond &camera_q) {
  if (point_num == 0) return;
  integratePointCloud(points, point_num, camera_pos, camera_q, INTEGRATION_MODE::DEINTEGRATION);
}

void SDFMap::reintegratePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points,
                                   const int &point_num, const Eigen::Vector3d &camera_pos,
                                   const Eigen::Quaterniond &camera_q) {
  if (point_num == 0) return;
  integratePointCloud(points, point_num, camera_pos, camera_q, INTEGRATION_MODE::REINTEGRATION);
}

// update occupacy map based on TSDF in a full-map order,
// time-consuming
void SDFMap::updateOccupancyFromTsdfAll() {
  updateOccupancyFromTsdfBoundingBoxIndex(mp_->box_min_, mp_->box_max_);
}

void SDFMap::updateOccupancyFromTsdfBoundingBoxIndex(const Eigen::Vector3i &bmin,
                                                     const Eigen::Vector3i &bmax) {
  for (int x = bmin(0); x < bmax(0); ++x)
    for (int y = bmin(1); y < bmax(1); ++y)
      for (int z = bmin(2); z < bmax(2); ++z) updateOccupancyFromTsdfByAddress(toAddress(x, y, z));
}

void SDFMap::updateOccupancyFromTsdfAddressList(const vector<int> &addr_list) {
  for (int addr : addr_list) updateOccupancyFromTsdfByAddress(addr);
}

void SDFMap::updateOccupancyFromTsdfByAddress(const int &addr) {
  if (md_->tsdf_weight_buffer_[addr] < 1e-5) {
    md_->occupancy_buffer_[addr] = OCC_VOXEL_STATE::UNKNOWN;
    return;
  }

  if (md_->tsdf_weight_buffer_[addr] < mp_->tsdf_observation_th_) {
    if (md_->tsdf_value_buffer_[addr] < mp_->tsdf_occu_th_) {
      md_->occupancy_buffer_[addr] = OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED;
    } else {
      md_->occupancy_buffer_[addr] = OCC_VOXEL_STATE::UNKNOWN_FRONTIER_FREE;
    }
    return;
  }

  if (md_->tsdf_value_buffer_[addr] < mp_->tsdf_occu_th_) {
    md_->occupancy_buffer_[addr] = OCC_VOXEL_STATE::OCCUPIED;
  } else {
    md_->occupancy_buffer_[addr] = OCC_VOXEL_STATE::FREE;
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d &pt,
                                         const Eigen::Vector3d &camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  // update inflated occupied cells
  // clean outdated occupancy

  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);

  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate newest occpuied cells
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        int id1 = toAddress(x, y, z);
        if (md_->occupancy_buffer_[id1] == OCC_VOXEL_STATE::UNKNOWN_FRONTIER_OCCUPIED ||
            md_->occupancy_buffer_[id1] == OCC_VOXEL_STATE::OCCUPIED) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (auto inf_pt : inf_pts) {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 && idx_inf < mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) *
                                              mp_->map_voxel_num_(2)) {
              md_->occupancy_buffer_inflate_[idx_inf] = 1;
            }
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_->virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
        // md_->occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        md_->occupancy_buffer_[toAddress(x, y, ceil_id)] == OCC_VOXEL_STATE::OCCUPIED;
      }
  }
}

double SDFMap::getResolution() { return mp_->resolution_; }

int SDFMap::getVoxelNum() {
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) {
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax) {
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset) {
  std::lock_guard<std::mutex> lock(bbox_mutex_);
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

double SDFMap::getInterpDist(const Eigen::Vector3d &pos) {
  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = md_->tsdf_value_buffer_[toAddress(current_idx)];
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  return dist;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_->resolution_inv_;

  return dist;
}

bool SDFMap::getPoseFromPathByTime(const ros::Time &t, const nav_msgs::Path &path,
                                   Eigen::Vector3d &p, Eigen::Matrix3d &R) {
  if (path.poses.size() < 2) return false;

  if (t.toSec() < path.poses.front().header.stamp.toSec() ||
      t.toSec() > path.poses.back().header.stamp.toSec()) {
    return false;
  }

  geometry_msgs::PoseStamped last_pose = path.poses.front();
  for (const geometry_msgs::PoseStamped &pose : path.poses) {
    if (pose.header.stamp.toSec() > t.toSec() && last_pose.header.stamp.toSec() <= t.toSec()) {
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

      p = (1 - t_diff) * pos_last + t_diff * pos_cur;
      R = (q_last.slerp(t_diff, q_cur)).toRotationMatrix();

      return true;
    }
    last_pose = pose;
  }

  return false;
}

// From loop_path_after_
bool SDFMap::getPoseByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                           Eigen::Quaterniond &camera_q_correct) {
  if (mr_->loop_path_before_.poses.size() < 2) return false;

  if (t.toSec() < mr_->loop_path_before_.poses.front().header.stamp.toSec() ||
      t.toSec() > mr_->loop_path_before_.poses.back().header.stamp.toSec()) {
    return false;
  }

  geometry_msgs::PoseStamped last_pose = mr_->loop_path_before_.poses.front();
  for (geometry_msgs::PoseStamped &pose : mr_->loop_path_before_.poses) {
    if (pose.header.stamp.toSec() > t.toSec()) {
      double offset_oldest_ns = t.toSec() - last_pose.header.stamp.toSec();
      double offset_newest_ns = pose.header.stamp.toSec() - t.toSec();
      double t_diff_ratio = offset_oldest_ns / (offset_newest_ns + offset_oldest_ns);

      // Interpolation
      Eigen::Vector3d pos_cur, pos_last;
      Eigen::Quaterniond q_cur, q_last;

      pos_cur = Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      pos_last = Eigen::Vector3d(last_pose.pose.position.x, last_pose.pose.position.y,
                                 last_pose.pose.position.z);

      q_cur = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                                 pose.pose.orientation.y, pose.pose.orientation.z);
      q_last = Eigen::Quaterniond(last_pose.pose.orientation.w, last_pose.pose.orientation.x,
                                  last_pose.pose.orientation.y, last_pose.pose.orientation.z);

      camera_pos_correct = (1 - t_diff_ratio) * pos_last + t_diff_ratio * pos_cur;
      camera_q_correct = q_last.slerp(t_diff_ratio, q_cur);

      break;
    }
    last_pose = pose;
  }

  return true;
}

bool SDFMap::getCorrectPoseByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                                  Eigen::Quaterniond &camera_q_correct) {
  if (pose_graph_.size() == 0) return false;

  if (t.toSec() < pose_graph_.front().time_stamp_.toSec() ||
      t.toSec() > pose_graph_.back().time_stamp_.toSec())
    return false;

  Pose last_pose = pose_graph_.front();

  for (Pose &pose : pose_graph_) {
    if (pose.time_stamp_.toSec() > t.toSec()) {
      double offset_oldest_ns = t.toSec() - last_pose.time_stamp_.toSec();
      double offset_newest_ns = pose.time_stamp_.toSec() - t.toSec();
      double t_diff_ratio = offset_oldest_ns / (offset_newest_ns + offset_oldest_ns);

      // Interpolation
      if (t_diff_ratio < 1e-4)
        t_diff_ratio = 1e-4;
      else if (t_diff_ratio > 1.0 - 1e-4)
        t_diff_ratio = 1.0 - 1e-4;
      camera_pos_correct = (1 - t_diff_ratio) * last_pose.position_ + t_diff_ratio * pose.position_;
      camera_q_correct = last_pose.quaternion_.slerp(t_diff_ratio, pose.quaternion_);

      break;
    }
    last_pose = pose;
  }

  return true;
}

bool SDFMap::getCorrectPoseFromPathByTime(const ros::Time &t, Eigen::Vector3d &camera_pos_correct,
                                          Eigen::Quaterniond &camera_q_correct) {
  if (path_opt_.poses.size() == 0) return false;

  if (t.toSec() < path_opt_.poses.front().header.stamp.toSec() ||
      t.toSec() > path_opt_.poses.back().header.stamp.toSec())
    return false;

  geometry_msgs::PoseStamped last_pose = path_opt_.poses.front();

  for (geometry_msgs::PoseStamped &pose : path_opt_.poses) {
    if (pose.header.stamp.toSec() > t.toSec()) {
      double offset_oldest_ns = t.toSec() - last_pose.header.stamp.toSec();
      double offset_newest_ns = pose.header.stamp.toSec() - t.toSec();
      double t_diff_ratio = offset_oldest_ns / (offset_newest_ns + offset_oldest_ns);

      // Interpolation
      Eigen::Vector3d pos_cur, pos_last;
      Eigen::Quaterniond q_cur, q_last;

      pos_cur = Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      pos_last = Eigen::Vector3d(last_pose.pose.position.x, last_pose.pose.position.y,
                                 last_pose.pose.position.z);

      q_cur = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                                 pose.pose.orientation.y, pose.pose.orientation.z);
      q_last = Eigen::Quaterniond(last_pose.pose.orientation.w, last_pose.pose.orientation.x,
                                  last_pose.pose.orientation.y, last_pose.pose.orientation.z);

      camera_pos_correct = (1 - t_diff_ratio) * pos_last + t_diff_ratio * pos_cur;
      camera_q_correct = q_last.slerp(t_diff_ratio, q_cur);

      break;
    }
    last_pose = pose;
  }

  return true;
}

bool SDFMap::getActiveLoopClosureViewpoints(vector<Eigen::Vector3d> &points, vector<double> &yaws,
                                            vector<vector<Eigen::Vector3d>> &viewpoints_pos,
                                            vector<vector<double>> &viewpoints_yaw) {
  points.clear();
  yaws.clear();
  viewpoints_pos.clear();
  viewpoints_yaw.clear();

  vector<Eigen::Vector3d> visualize_points;
  vector<double> visualize_yaws;

  if (!alc_mutex_.try_lock()) {
    ROS_ERROR("Cannot get the mutex lock for alc clusters");
    return false;
  } else {
    ROS_WARN_STREAM(
        "Get active_loop_closure_clusters_ with size: " << active_loop_closure_clusters_.size());
    for (ActiveLoopClosureCluster &cluster : active_loop_closure_clusters_) {
      if (cluster.center_position_[2] < 0.5) continue;
      Eigen::Quaterniond q = cluster.center_quaternion_;

      if (mr_->is_simulation_)
        q = q.toRotationMatrix() * mr_->cam02body.block<3, 3>(0, 0).transpose();  // R_wb

      double yaw =
          atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

      points.push_back(cluster.center_position_);
      yaws.push_back(yaw);

      vector<Eigen::Vector3d> viewpoint_pos;
      vector<double> viewpoint_yaw;
      for (Pose &pose : cluster.active_loop_closure_candidates_) {
        Eigen::Quaterniond vp_q = pose.quaternion_;

        if (mr_->is_simulation_)
          vp_q = vp_q.toRotationMatrix() * mr_->cam02body.block<3, 3>(0, 0).transpose();  // R_wb

        double vp_yaw = atan2(2 * (vp_q.w() * vp_q.z() + vp_q.x() * vp_q.y()),
                              1 - 2 * (vp_q.y() * vp_q.y() + vp_q.z() * vp_q.z()));

        viewpoint_pos.push_back(pose.position_);
        viewpoint_yaw.push_back(vp_yaw);

        visualize_points.push_back(pose.position_);
        visualize_yaws.push_back(vp_yaw);
      }
      viewpoints_pos.push_back(viewpoint_pos);
      viewpoints_yaw.push_back(viewpoint_yaw);
    }
    alc_mutex_.unlock();
  }

  publishViewpointsWithYaw(visualize_points, visualize_yaws);
}

void SDFMap::publishCoveredGrids(const vector<int> &grid_list) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  for (int addr : grid_list) {
    Eigen::Vector3d pos = indexToPosSetCover((addressToIndexSetCover(addr)));
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    cloud1.push_back(pt);
  }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  tmp_grid_pub_.publish(cloud_msg);
}

void SDFMap::publishAllCoveredGrids() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  for (const BlockData &block : md_->block_buffer_) {
    for (const Keyframe &kf : block.keyframe_buffer_) {
      for (const int &addr : kf.covered_grids_list_) {
        Eigen::Vector3d pos = indexToPosSetCover((addressToIndexSetCover(addr)));
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud1.push_back(pt);
      }
    }
  }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  tmp_all_grid_pub_.publish(cloud_msg);
}

void SDFMap::publishAllKeyframes() {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  mk.action = visualization_msgs::Marker::DELETE;
  keyframe_fov_pub_.publish(mk);

  for (auto kf : keyframe_list_) {
    Eigen::Vector3d camera_pos = kf.camera_pos_;
    Eigen::Quaterniond camera_q = kf.camera_q_;
    Eigen::Matrix3d camera_r = camera_q.toRotationMatrix();                       // R_wc
    Eigen::Matrix3d R = camera_r * mr_->cam02body.block<3, 3>(0, 0).transpose();  // R_wb
    Eigen::Quaterniond q(R);
    double yaw =
        atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    percep_utils_->setPose(camera_pos, yaw);
    vector<Eigen::Vector3d> list1, list2;
    percep_utils_->getFOV(list1, list2);

    if (list1.size() == 0) return;

    // Pub new marker
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
  }

  mk.action = visualization_msgs::Marker::ADD;

  keyframe_fov_pub_.publish(mk);
}

void SDFMap::publishAllKeyframesByBlock() {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.02;
  mk.scale.y = 0.02;
  mk.scale.z = 0.02;

  mk.action = visualization_msgs::Marker::DELETE;
  keyframe_fov_pub_.publish(mk);
  for (BlockData &block : md_->block_buffer_) {
    for (Keyframe &kf : block.keyframe_buffer_) {
      Eigen::Vector3d camera_pos = kf.camera_pos_;
      Eigen::Quaterniond q(kf.camera_q_.toRotationMatrix() *
                           mr_->cam02body.block<3, 3>(0, 0).transpose());
      double yaw =
          atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
      percep_utils_->setPose(camera_pos, yaw);
      vector<Eigen::Vector3d> list1, list2;
      percep_utils_->getFOV(list1, list2);

      if (list1.size() == 0) return;

      // Pub new marker
      geometry_msgs::Point pt;
      for (int i = 0; i < int(list1.size()); ++i) {
        pt.x = list1[i](0);
        pt.y = list1[i](1);
        pt.z = list1[i](2);
        mk.points.push_back(pt);

        pt.x = list2[i](0);
        pt.y = list2[i](1);
        pt.z = list2[i](2);
        mk.points.push_back(pt);
      }
    }
  }

  mk.action = visualization_msgs::Marker::ADD;

  keyframe_fov_block_pub_.publish(mk);
}

void SDFMap::publishErrorLine(const vector<Eigen::Vector3d> &list1,
                              const vector<Eigen::Vector3d> &list2) {
  if (list1.size() == 0) return;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.01;
  mk.scale.y = 0.01;
  mk.scale.z = 0.01;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;

  error_line_pub_.publish(mk);
}

void SDFMap::publishKeyframe(const Eigen::Vector3d &camera_pos,
                             const Eigen::Quaterniond &camera_q) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  mk.action = visualization_msgs::Marker::DELETE;
  keyframe_fov_pub_.publish(mk);

  Eigen::Matrix3d camera_r = camera_q.toRotationMatrix();                       // R_wc
  Eigen::Matrix3d R = camera_r * mr_->cam02body.block<3, 3>(0, 0).transpose();  // R_wb
  Eigen::Quaterniond q(R);
  double yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  percep_utils_->setPose(camera_pos, yaw);
  vector<Eigen::Vector3d> list1, list2;
  percep_utils_->getFOV(list1, list2);

  if (list1.size() == 0) return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;

  keyframe_fov_pub_.publish(mk);
}

void SDFMap::publishAllBlocks() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = mp_->block_size_[0];
  marker.scale.y = mp_->block_size_[1];
  marker.scale.z = mp_->block_size_[2];

  marker.lifetime = ros::Duration();

  geometry_msgs::Point p;
  std_msgs::ColorRGBA c;

  for (BlockData &block : md_->block_buffer_) {
    if (!block.is_visited) continue;

    p.x = block.center_pos_[0];
    p.y = block.center_pos_[1];
    p.z = block.center_pos_[2];
    marker.points.push_back(p);

    c.r = 1.0;
    c.g = 0;
    c.b = 0;
    c.a = 0.1;
    marker.colors.push_back(c);
  }

  block_all_pub_.publish(marker);
}
void SDFMap::publishBlock(const Eigen::Vector3d &center_pos) {}

void SDFMap::publishBoundingBox(const Eigen::Vector3d &bmin, const Eigen::Vector3d &bmax) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Vector3d center_pos = (bmin + bmax) / 2;
  marker.pose.position.x = center_pos[0];
  marker.pose.position.y = center_pos[1];
  marker.pose.position.z = center_pos[2];

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = bmax[0] - bmin[0];
  marker.scale.y = bmax[1] - bmin[1];
  marker.scale.z = bmax[2] - bmin[2];

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.1;

  bbox_pub_.publish(marker);
}

void SDFMap::publishKeyframeNumber() {
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "world";
  // marker.header.stamp = ros::Time::now();
  // marker.ns = "basic_shapes";
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.orientation.w = 1.0;
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // marker.scale.z = 1.0;
  // marker.color.b = 0;
  // marker.color.g = 0;
  // marker.color.r = 255;
  // marker.color.a = 1;

  // geometry_msgs::Pose pose;
  // pose.position.x = 10;
  // pose.position.y = 0;
  // pose.position.z = 0;

  // ostringstream str;
  int size = 0;
  for (BlockData &block : md_->block_buffer_) {
    size += block.keyframe_buffer_.size();
  }
  // str << "Size of keyframe list: " << size;  // keyframe_list_.size();
  // marker.text = str.str();
  // marker.pose = pose;

  std_msgs::Int32 msg;
  msg.data = size;
  kf_num_pub_.publish(msg);

  // ROS_WARN("Keyframe num: %d", size);
}

void SDFMap::publishCompensatedPose(const Eigen::Vector3d &camera_pos,
                                    const Eigen::Quaterniond &camera_q) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.seq = 0;
  pose.pose.position.x = camera_pos(0);
  pose.pose.position.y = camera_pos(1);
  pose.pose.position.z = camera_pos(2);
  pose.pose.orientation.x = camera_q.x();
  pose.pose.orientation.y = camera_q.y();
  pose.pose.orientation.z = camera_q.z();
  pose.pose.orientation.w = camera_q.w();
  pose_compensated_pub_.publish(pose);
}

void SDFMap::publishViewpointsWithYaw(const vector<Eigen::Vector3d> &points,
                                      const vector<double> &yaws) {
  if (points.size() != yaws.size()) {
    ROS_ERROR("publishViewpointsWithYaw wrong dim");
    return;
  }

  visualization_msgs::Marker mk;
  geometry_msgs::Point pt;

  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 1.0;
  mk.color.b = 1.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.2;
  mk.scale.y = 0.2;
  mk.scale.z = 0.2;

  for (Eigen::Vector3d point : points) {
    pt.x = point(0);
    pt.y = point(1);
    pt.z = point(2);
    mk.points.push_back(pt);
  }

  viewpoints_pos_pub_.publish(mk);

  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.5;
  mk.scale.x = 0.01;
  mk.scale.y = 0.01;
  mk.scale.z = 0.01;
  mk.points.clear();
  vector<Eigen::Vector3d> list1, list2;

  for (int i = 0; i < points.size(); i++) {
    percep_utils_->setPose(points[i], yaws[i]);
    percep_utils_->getFOV(list1, list2);

    if (list1.size() == 0) return;

    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
  }

  viewpoints_yaw_pub_.publish(mk);
}

void SDFMap::publishCenterViewpointsWithYaw(const vector<Eigen::Vector3d> &points,
                                            const vector<double> &yaws) {
  if (points.size() != yaws.size()) {
    ROS_ERROR("publishCenterViewpointsWithYaw wrong dim");
    return;
  }

  visualization_msgs::Marker mk;
  geometry_msgs::Point pt;

  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.4;
  mk.scale.y = 0.4;
  mk.scale.z = 0.4;

  for (Eigen::Vector3d point : points) {
    pt.x = point(0);
    pt.y = point(1);
    pt.z = point(2);
    mk.points.push_back(pt);
  }

  center_viewpoints_pos_pub_.publish(mk);

  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.5;
  mk.scale.x = 0.02;
  mk.scale.y = 0.02;
  mk.scale.z = 0.02;
  mk.points.clear();
  vector<Eigen::Vector3d> list1, list2;

  for (int i = 0; i < points.size(); i++) {
    percep_utils_->setPose(points[i], yaws[i]);
    percep_utils_->getFOV(list1, list2);

    if (list1.size() == 0) return;

    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
  }

  center_viewpoints_yaw_pub_.publish(mk);
}

void SDFMap::publishSingleViewpointsWithYaw(const Eigen::Vector3d &point, const double &yaw) {
  if (yaw < -100) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.ns = "current_pose";
    mk.action = visualization_msgs::Marker::DELETE;
    center_viewpoints_pos_pub_.publish(mk);
    center_viewpoints_yaw_pub_.publish(mk);
    return;
  }
  visualization_msgs::Marker mk;
  geometry_msgs::Point pt;

  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.4;
  mk.scale.y = 0.4;
  mk.scale.z = 0.4;

  pt.x = point(0);
  pt.y = point(1);
  pt.z = point(2);
  mk.points.push_back(pt);

  center_viewpoints_pos_pub_.publish(mk);

  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.5;
  mk.scale.x = 0.02;
  mk.scale.y = 0.02;
  mk.scale.z = 0.02;
  mk.points.clear();
  vector<Eigen::Vector3d> list1, list2;

  percep_utils_->setPose(point, yaw);
  percep_utils_->getFOV(list1, list2);

  if (list1.size() == 0) return;

  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }

  center_viewpoints_yaw_pub_.publish(mk);
}

void SDFMap::publishRay(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                        const vector<Eigen::Vector3d> &list3) {
  if (list1.size() == 0) return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.005;
  mk.scale.y = 0.005;
  mk.scale.z = 0.005;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }

  mk.action = visualization_msgs::Marker::ADD;

  ray_pub_.publish(mk);

  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 0.3;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  mk.points.clear();

  for (int i = 0; i < int(list3.size()); ++i) {
    pt.x = list3[i](0);
    pt.y = list3[i](1);
    pt.z = list3[i](2);
    mk.points.push_back(pt);
  }

  ray_pt_pub_.publish(mk);
}

bool SDFMap::handleLoopClosure() {
  // resetOccuBuffer();
  // resetTSDFBuffer();

  std::lock_guard<std::mutex> lock(integration_mutex_);

  // publishAllKeyframesByBlock();

  int reint_frame_count = 0;

  // changed to read from extrinsic topic
  Eigen::Matrix3d Ric;
  Eigen::Vector3d tic;

  mr_->extrinsic_mutex_.lock();
  Ric = mr_->qic.toRotationMatrix();
  tic = mr_->tic;
  mr_->extrinsic_mutex_.unlock();
  ROS_WARN_STREAM("tic: " << tic.transpose());

  for (BlockData &block : md_->block_buffer_) {
    if (block.keyframe_buffer_.size() == 0) continue;
    // ROS_WARN("Re-integration the frame");

    for (vector<Keyframe>::iterator f_it = block.keyframe_buffer_.begin();
         f_it != block.keyframe_buffer_.end(); ++f_it) {
      if (f_it->is_corrected_) continue;
      if (f_it->state_ == FRAME_STATE::WAITING_FOR_DEINTEGRATION ||
          f_it->state_ == FRAME_STATE::DEINTEGRATED)
        continue;

      reint_frame_count++;
      // find the corrected pose for this frame
      Eigen::Vector3d body_pos_correct, camera_pos_correct;
      Eigen::Matrix3d body_R_correct;
      Eigen::Quaterniond camera_q_correct;

      // use interpolator to find the correct pose from correct pose graph
      // if (!getCorrectPoseFromPathByTime(f_it->time_stamp_, body_pos_correct, body_q_correct)) {
      //   ROS_ERROR("The keyframe is not in the pose graph range");
      //   continue;
      // }

      if (!getPoseFromPathByTime(f_it->time_stamp_, path_loop_, body_pos_correct, body_R_correct)) {
        ROS_ERROR("The keyframe is not in the pose graph range");
        ROS_ERROR("kf_t: %f, path t0: %f, path t1: %f", f_it->time_stamp_.toSec(),
                  path_loop_.poses.front().header.stamp.toSec(),
                  path_loop_.poses.back().header.stamp.toSec());
        continue;
      }

      camera_pos_correct = body_pos_correct + body_R_correct * tic;
      camera_q_correct = body_R_correct * Ric;

      f_it->camera_pos_correct_ = camera_pos_correct;
      f_it->camera_q_correct_ = camera_q_correct;
      f_it->is_corrected_ = true;
      f_it->state_ = FRAME_STATE::WAITING_FOR_REINTEGRATION;
    }
  }

  ROS_WARN_STREAM("Re-integration frame count: " << reint_frame_count);

  // publishAllKeyframesByBlock();
  // publishAllCoveredGrids();

  return true;
}

bool SDFMap::handleLoopClosureSimulation() {
  // resetOccuBuffer();
  // resetTSDFBuffer();

  std::lock_guard<std::mutex> lock(integration_mutex_);

  // publishAllKeyframesByBlock();

  int reint_frame_count = 0;

  for (BlockData &block : md_->block_buffer_) {
    if (block.keyframe_buffer_.size() == 0) continue;
    // ROS_WARN("Re-integration the frame");

    // Start from the lastest keyframe
    // for (vector<Keyframe>::reverse_iterator f_it = block.keyframe_buffer_.rbegin();
    //      f_it != block.keyframe_buffer_.rend(); ++f_it) {
    for (vector<Keyframe>::iterator f_it = block.keyframe_buffer_.begin();
         f_it != block.keyframe_buffer_.end(); ++f_it) {
      if (f_it->is_corrected_) continue;

      reint_frame_count++;

      // find the corrected pose for this frame
      // bool isCorrectedPoseFound = false;

      Eigen::Vector3d camera_pos_correct;
      Eigen::Matrix3d camera_R_correct;
      Eigen::Quaterniond camera_q_correct;

      // use interpolator to find the correct pose from correct pose graph
      // if (!getCorrectPoseByTime(f_it->time_stamp_, camera_pos_correct, camera_q_correct)) {
      //   ROS_ERROR("The keyframe is not in the pose graph range");
      //   continue;
      // }
      if (!getPoseFromPathByTime(f_it->time_stamp_, path_loop_, camera_pos_correct,
                                 camera_R_correct)) {
        ROS_ERROR("The keyframe is not in the pose graph range");
        ROS_ERROR("kf_t: %f, path t0: %f, path t1: %f", f_it->time_stamp_.toSec(),
                  path_loop_.poses.front().header.stamp.toSec(),
                  path_loop_.poses.back().header.stamp.toSec());
        continue;
      }

      // if ((camera_pos_correct - f_it->camera_pos_).norm() < 0.1 &&
      //     acos(((camera_R_correct * f_it->camera_q_.toRotationMatrix().transpose()).trace() - 1)
      //     /
      //          2) < 5.0 * M_PI / 180.0) {
      //   continue;
      // }

      f_it->camera_pos_correct_ = camera_pos_correct;
      f_it->camera_q_correct_ = camera_R_correct;
      f_it->is_corrected_ = true;
      f_it->state_ = FRAME_STATE::WAITING_FOR_REINTEGRATION;
    }
  }

  ROS_WARN_STREAM("Re-integration frame count: " << reint_frame_count);

  // publishAllKeyframesByBlock();
  // publishAllCoveredGrids();

  return true;
}

bool SDFMap::saveTSDFCallback(std_srvs::Empty::Request &request,
                              std_srvs::Empty::Response &response) {
  bool res = saveTSDFMesh();
  // ros::Duration(2.0).sleep();

  return true;
}

bool SDFMap::saveTSDFMesh() {
  if (mesh_dir_.empty()) {
    ROS_ERROR("Empty mesh save path");
    return false;
  }

  boost::posix_time::ptime t = ros::Time::now().toBoost();
  std::string t_iso_str = boost::posix_time::to_iso_string(t);

  stringstream ss;
  ss << mesh_dir_ << "TSDF_mesh_" << t_iso_str << ".stl";

  Eigen::Vector3i bbmin, bbmax;
  bbmin = mp_->box_min_ + mp_->box_min_inflate_;
  bbmax = mp_->box_max_ + mp_->box_max_inflate_;

  int x_max = bbmax[0] - bbmin[0];
  int y_max = bbmax[1] - bbmin[1];
  int z_max = bbmax[2] - bbmin[2];

  std::vector<std::vector<std::vector<double>>> tsdfGrid;
  tsdfGrid.resize(x_max);
  for (int i = 0; i < x_max; i++) {
    tsdfGrid[i].resize(y_max);

    for (int j = 0; j < y_max; j++) {
      tsdfGrid[i][j].resize(z_max, -20.0);
    }
  }

  for (int x = bbmin[0]; x < bbmax[0]; ++x) {
    for (int y = bbmin[1]; y < bbmax[1]; ++y) {
      for (int z = bbmin[2]; z < bbmax[2]; ++z) {
        tsdfGrid[x - bbmin[0]][y - bbmin[1]][z - bbmin[2]] =
            md_->tsdf_value_buffer_[toAddress(Eigen::Vector3i(x, y, z))];
      }
    }
  }

  ROS_WARN_STREAM("box_min_: " << mp_->box_min_.transpose());
  ROS_WARN_STREAM("box_max_: " << mp_->box_max_.transpose());

  mesher.setParam(mp_->resolution_);
  mesher.generateMeshFile(tsdfGrid, ss.str(), x_max, y_max, z_max);

  return true;
}

bool SDFMap::evalCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  // saveTsdfMapPCL();
  // eval();
  eval2();
}

// Compare the mapping result with a ground truth pointcloud from simulator
void SDFMap::eval() {
  ROS_WARN("Evalation of map result start");

  double mse = 0.0, sdf, weight;
  int total_evaluated_voxels = 0, unknown_voxels = 0;
  Eigen::Vector3d pt;
  Eigen::Vector3i vox_idx;
  int vox_adr;

  for (pcl::PointXYZ pt_pcl : cloud_gt_.points) {
    total_evaluated_voxels++;

    pt[0] = pt_pcl.x;
    pt[1] = pt_pcl.y;
    pt[2] = pt_pcl.z;

    if (!isInMap(pt)) {
      continue;
    }

    posToIndex(pt, vox_idx);
    vox_adr = toAddress(vox_idx);

    sdf = md_->tsdf_value_buffer_[vox_adr];
    weight = md_->tsdf_weight_buffer_[vox_adr];

    if (weight <= 0.01) {
      unknown_voxels++;
      continue;
    }

    if (sdf >= mp_->tsdf_default_truncation_dist_) {
      mse += mp_->tsdf_default_truncation_dist_ * mp_->tsdf_default_truncation_dist_;
    } else {
      // trilinear interpolate to get dist
      double dist = getInterpDist(pt);
      mse += dist * dist;
    }
  }

  double rms = sqrt(mse / (total_evaluated_voxels - unknown_voxels));

  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  ROS_WARN_STREAM("RMSE: " << rms);
  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

// Compare the mapping result with a ground truth pointcloud from external file
void SDFMap::eval2() {
  ROS_WARN("Evalation of map result start");

  pcl::PLYReader ply_reader;
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_ptcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  ply_reader.read("/home/eason/source/bag/gt.ply", *gt_ptcloud_ptr);
  kdtree.setInputCloud(gt_ptcloud_ptr);

  int K = 1;
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);
  pcl::PointXYZRGB searchPoint;

  double mse = 0.0, sdf, weight;
  int total_evaluated_voxels = 0, useful_voxels = 0;
  Eigen::Vector3d pt;
  Eigen::Vector3i vox_idx;
  int vox_adr;

  double resolution = 0.1;
  int iter_cnt = 100;
  uint64_t voxel_cnt = 0;

  Eigen::Vector3d negtive_offset_x(-resolution / 2, 0, 0);
  Eigen::Vector3d positive_offset_x(resolution / 2, 0, 0);
  Eigen::Vector3d step_x(resolution / iter_cnt, 0, 0);

  Eigen::Vector3i bbmin, bbmax;
  bbmin = mp_->box_min_ + mp_->box_min_inflate_;
  bbmax = mp_->box_max_ + mp_->box_max_inflate_;

  vector<Eigen::Vector3d> list1, list2;

  for (int x = bbmin[0]; x < bbmax[0]; ++x) {
    for (int y = bbmin[1]; y < bbmax[1]; ++y) {
      for (int z = bbmin[2]; z < bbmax[2]; ++z) {
        Eigen::Vector3i idx(x, y, z);
        Eigen::Vector3d pos = indexToPos(idx);
        int adr = toAddress(idx);
        total_evaluated_voxels++;

        sdf = md_->tsdf_value_buffer_[adr];
        weight = md_->tsdf_weight_buffer_[adr];

        if (weight > 1e-4 && sdf <= 0.1 && sdf >= -0.1) {
          Eigen::Vector3d surface_point;
          Eigen::Vector3d start_x;
          Eigen::Vector3d end_x;

          start_x = pos + negtive_offset_x;
          end_x = pos + positive_offset_x;

          double dist_x_neg, dist_x_pos;
          dist_x_neg = getInterpDist(start_x);
          dist_x_pos = getInterpDist(end_x);

          bool is_found = false;
          if (dist_x_neg * dist_x_pos < 0) {
            // find surface point on x axis
            Eigen::Vector3d pt;
            double dist_x, dist_last = 999;
            for (int i = 0; i < iter_cnt; i++) {
              pt = start_x + i * step_x;
              dist_x = getInterpDist(pt);

              if (dist_x * dist_last < 0 && dist_last < 100) {
                is_found = true;
                surface_point = pt;
                break;
              }

              dist_last = dist_x;
            }
          } else
            continue;

          if (!is_found) continue;

          useful_voxels++;
          // ROS_WARN_STREAM("Voxel center: " << coord.transpose());
          // ROS_WARN_STREAM("surface_point: " << surface_point.transpose());

          // find nearest point in gt cloud
          searchPoint.x = surface_point[0];
          searchPoint.y = surface_point[1];
          searchPoint.z = surface_point[2];
          if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) >
              0) {
            Eigen::Vector3d gt_pt;
            gt_pt[0] = (*gt_ptcloud_ptr)[pointIdxKNNSearch[0]].x;
            gt_pt[1] = (*gt_ptcloud_ptr)[pointIdxKNNSearch[0]].y;
            gt_pt[2] = (*gt_ptcloud_ptr)[pointIdxKNNSearch[0]].z;

            double error = (surface_point - gt_pt).norm();
            list1.push_back(surface_point);
            list2.push_back(gt_pt);
            mse += error * error;
            voxel_cnt++;
          }
        }
      }
    }
  }

  // publishErrorLine(list1, list2);
  double rms = sqrt(mse / voxel_cnt);
  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  std::cout << "Finished evaluating.\n"
            << "\nRMS Error:           " << rms << ")\n";
  ROS_WARN("total vox: %d, useful vox: %d", total_evaluated_voxels, useful_voxels);
  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

}  // namespace fast_planner
