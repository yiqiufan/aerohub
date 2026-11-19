#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>

using SolveMTSP = lkh_mtsp_solver::srv::SolveMTSP;

namespace {
std::string mtsp_dir1;
std::string mtsp_dir2;
std::string mtsp_dir3;
int drone_id = 1;

void solveCallback(
    const std::shared_ptr<SolveMTSP::Request> request,
    std::shared_ptr<SolveMTSP::Response> response) {
  if (request->prob == 1) {
    solveMTSPWithLKH3(mtsp_dir1.c_str());
  } else if (request->prob == 2) {
    solveMTSPWithLKH3(mtsp_dir2.c_str());
  } else if (request->prob == 3) {
    const std::string cmd = "lkh3_solver " + mtsp_dir3;
    if (std::system(cmd.c_str()) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mtsp_node"), "Failed to run bundled LKH3 solver.");
    }
  }
  response->empty = 0;
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mtsp_node");

  node->declare_parameter<std::string>("exploration.mtsp_dir", "null");
  node->declare_parameter<int>("exploration.drone_id", 1);
  node->declare_parameter<int>("exploration.problem_id", 1);

  const auto mtsp_dir = node->get_parameter("exploration.mtsp_dir").as_string();
  drone_id = static_cast<int>(node->get_parameter("exploration.drone_id").as_int());
  const int problem_id = static_cast<int>(node->get_parameter("exploration.problem_id").as_int());

  mtsp_dir1 = mtsp_dir + "/amtsp_" + std::to_string(drone_id) + ".par";
  mtsp_dir2 = mtsp_dir + "/amtsp2_" + std::to_string(drone_id) + ".par";
  mtsp_dir3 = mtsp_dir + "/amtsp3_" + std::to_string(drone_id) + ".par";

  const std::string service_name =
      problem_id == 1 ? "/solve_tsp_" + std::to_string(drone_id)
                      : "/solve_acvrp_" + std::to_string(drone_id);

  auto server = node->create_service<SolveMTSP>(service_name, solveCallback);
  RCLCPP_WARN(node->get_logger(), "MTSP server %d is ready.", drone_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
