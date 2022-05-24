#include "plan_env/visualization_utils.h"

namespace fast_planner {
void VisualizationUtils::init(ros::NodeHandle &nh) {
  covered_grids_single_keyframe_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/single_covered_grids", 1);
  covered_grids_all_keyframes_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/all_covered_grids", 1);
  keyframe_num_pub_ = nh.advertise<std_msgs::Int32>("/keyframe_num", 1);
  keyframe_fov_pub_ = nh.advertise<visualization_msgs::Marker>("/keyframe_fov", 1);
  block_pub_ = nh.advertise<visualization_msgs::Marker>("/all_blocks", 1);
  update_bbox_pub_ = nh.advertise<visualization_msgs::Marker>("/update_bounding_box", 1);
}

void VisualizationUtils::publishSingleKeyframeCoveredGrids(const vector<int> &grid_list) {
  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  for (const int &addr : grid_list) {
    Eigen::Vector3d pos = indexToPosSetCover((addressToIndexSetCover(addr)));
    pointcloud.push_back(new pcl::PointXYZ(pos(0), pos(1), pos(2)));
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  covered_grids_single_keyframe_pub_.publish(pointcloud_msg);
}

void VisualizationUtils::publishAllKeyframeCoveredGrids() {}
void VisualizationUtils::publishSingleKeyframeFOV(const Eigen::Vector3d &camera_pos,
                                                  const Eigen::Quaterniond &camera_q) {}
void VisualizationUtils::publishAllKeyframesFOV() {}
void VisualizationUtils::publishAllBlocks() {}
void VisualizationUtils::publishUpdateBoundingBox(const Eigen::Vector3d &bmin,
                                                  const Eigen::Vector3d &bmax) {}

}  // namespace fast_planner