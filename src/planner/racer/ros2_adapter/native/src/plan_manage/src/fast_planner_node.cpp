#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>
#include <plan_manage/local_explore_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fast_planner_node");

  int planner;
  planner = node->declare_parameter<int>("planner_node.planner", -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;
  LocalExploreFSM local_explore;

  if (planner == 1) {
    kino_replan.init(node);
  } else if (planner == 2) {
    topo_replan.init(node);
  } else if (planner == 3) {
    local_explore.init(node);
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
