#include <plan_env/sdf_map.h>

using namespace std;
using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sdf_map_node");
  ros::NodeHandle nh("~");

  shared_ptr<SDFMap> sdf_map_;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  
  return 0; 
}