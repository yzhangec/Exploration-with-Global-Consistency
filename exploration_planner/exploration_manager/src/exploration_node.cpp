#include <ros/ros.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  // Wait for all registration finished
  ros::Duration(0.1).sleep();

  ros::MultiThreadedSpinner spinner(10); 
  spinner.spin();

  return 0;
}
