#include <ros/ros.h>

namespace fast_planner {
class VisualizationUtils {
 public:
  VisualizationUtils();
  ~VisualizationUtils();

  void init(ros::NodeHandle &nh);
  void publishSingleKeyframeCoveredGrids(const vector<int> &grid_list);
  void publishAllKeyframeCoveredGrids();
  void publishSingleKeyframeFOV(const Eigen::Vector3d &camera_pos,
                                const Eigen::Quaterniond &camera_q);
  void publishAllKeyframesFOV();
  void publishAllBlocks();
  void publishUpdateBoundingBox(const Eigen::Vector3d &bmin, const Eigen::Vector3d &bmax);

 private:
  ros::Publisher covered_grids_single_keyframe_pub_, covered_grids_all_keyframes_pub_,
      keyframe_num_pub_, keyframe_fov_pub_, block_pub_, update_bbox_pub_;
}
}  // namespace fast_planner
