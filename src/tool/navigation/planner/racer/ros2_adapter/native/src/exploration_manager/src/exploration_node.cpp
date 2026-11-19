#include <exploration_manager/fast_exploration_fsm.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("exploration_node");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(node);

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
