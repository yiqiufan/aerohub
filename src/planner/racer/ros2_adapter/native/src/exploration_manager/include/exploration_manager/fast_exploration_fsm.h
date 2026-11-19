#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>
#include <exploration_manager/latest_sample.h>

#include <bspline/msg/bspline.hpp>
#include <exploration_manager/msg/drone_state.hpp>
#include <exploration_manager/msg/grid_tour.hpp>
#include <exploration_manager/msg/h_grid.hpp>
#include <exploration_manager/msg/pair_opt.hpp>
#include <exploration_manager/msg/pair_opt_response.hpp>
#include <exploration_manager/msg/trajectory_execution.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <deque>
#include <string>
#include <thread>
#include <unordered_map>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;
struct LocalTrajData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(const rclcpp::Node::SharedPtr& node);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  int callExplorationPlanner(const rclcpp::Time& start);
  bool externalExecutionPending();
  bool drainExecutionStatus();
  void executionStatus(const exploration_manager::msg::TrajectoryExecution::ConstSharedPtr& msg);
  void stopExternalExecution(EXPL_STATE next, const std::string& reason);
  void retryExternalPlan(const std::string& reason);
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS functions */
  void FSMCallback();
  void safetyCallback();
  void frontierCallback();
  void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  // Swarm
  void droneStateTimerCallback();
  void droneStateMsgCallback(const exploration_manager::msg::DroneState::ConstSharedPtr& msg);
  void optTimerCallback();
  void optMsgCallback(const exploration_manager::msg::PairOpt::ConstSharedPtr& msg);
  void optResMsgCallback(const exploration_manager::msg::PairOptResponse::ConstSharedPtr& msg);
  void swarmTrajCallback(const bspline::msg::Bspline::ConstSharedPtr& msg);
  void swarmTrajTimerCallback();

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;
  bool show_grid_text_;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr bspline_pub_;

  // Transport callback only queues; planner/map state stays in callback_group_.
  struct ReceivedExecutionStatus {
    exploration_manager::msg::TrajectoryExecution::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received;
  };
  rclcpp::CallbackGroup::SharedPtr execution_status_group_;
  std::mutex execution_status_mutex_;
  std::deque<ReceivedExecutionStatus> execution_status_queue_;
  bool external_execution_{false};
  bool pending_accepted_{false};
  bool waiting_for_stop_{false};
  bool need_fresh_odom_{false};
  double execution_lead_{0.75};
  double minimum_lead_{0.10};
  double stop_reserve_{0.35};
  int execution_id_{0};
  int64_t diagnostic_odom_stamp_ns_{0};
  int64_t last_static_plan_odom_ns_{0};
  rclcpp::CallbackGroup::SharedPtr odometry_receive_group_;
  LatestSample<nav_msgs::msg::Odometry> odometry_inbox_;
  EXPL_STATE after_stop_{PLAN_TRAJ};
  rclcpp::Time stop_confirmed_at_;
  std::shared_ptr<LocalTrajData> active_execution_, pending_execution_;
  bspline::msg::Bspline active_execution_msg_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr execution_plan_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr execution_stop_pub_;
  rclcpp::Subscription<exploration_manager::msg::TrajectoryExecution>::SharedPtr execution_status_sub_;

  // Swarm state
  rclcpp::Publisher<exploration_manager::msg::DroneState>::SharedPtr drone_state_pub_;
  rclcpp::Publisher<exploration_manager::msg::PairOpt>::SharedPtr opt_pub_;
  rclcpp::Publisher<exploration_manager::msg::PairOptResponse>::SharedPtr opt_res_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr swarm_traj_pub_;
  rclcpp::Publisher<exploration_manager::msg::GridTour>::SharedPtr grid_tour_pub_;
  rclcpp::Publisher<exploration_manager::msg::HGrid>::SharedPtr hgrid_pub_;
  rclcpp::Subscription<exploration_manager::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<exploration_manager::msg::PairOpt>::SharedPtr opt_sub_;
  rclcpp::Subscription<exploration_manager::msg::PairOptResponse>::SharedPtr opt_res_sub_;
  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr swarm_traj_sub_;
  rclcpp::TimerBase::SharedPtr drone_state_timer_, opt_timer_, swarm_traj_timer_;
};

}  // namespace fast_planner

#endif
