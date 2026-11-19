#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <armadillo>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pose_utils.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace arma;
using namespace std;

namespace {

string mesh_resource;
double color_r;
double color_g;
double color_b;
double color_a;
double cov_scale;
double scale;
int drone_id;
int drone_num;

bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covVelPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensorPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr heightPub;

geometry_msgs::msg::PoseStamped poseROS;
nav_msgs::msg::Path pathROS;
visualization_msgs::msg::Marker velROS;
visualization_msgs::msg::Marker covROS;
visualization_msgs::msg::Marker covVelROS;
visualization_msgs::msg::Marker trajROS;
visualization_msgs::msg::Marker sensorROS;
visualization_msgs::msg::Marker meshROS;
sensor_msgs::msg::Range heightROS;
string frame_id;

Eigen::Vector4d getColor(const double& h, double alpha) {
  double h1 = h;
  if (!std::isfinite(h1) || h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda = 0.0;
  Eigen::Vector4d color1(1, 0, 0, 1);
  Eigen::Vector4d color2(1, 0, 0, 1);
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->header.frame_id == string("null")) return;

  colvec q(4);
  meshROS.header.frame_id = frame_id;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::msg::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x;
  meshROS.pose.position.y = msg->pose.pose.position.y;
  meshROS.pose.position.z = msg->pose.pose.position.z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;

  const double color_h = drone_num > 1 ? (drone_id - 1) / double(drone_num - 1) : 0.0;
  auto color = getColor(color_h, 1);

  meshROS.color.r = color[0];
  meshROS.color.g = color[1];
  meshROS.color.b = color[2];
  meshROS.color.a = color[3];
  meshROS.mesh_resource = mesh_resource;
  meshPub->publish(meshROS);
}

void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
  if (cmd->header.frame_id == string("null")) return;

  colvec q(4);
  q(0) = 1.0;
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;

  meshROS.header.frame_id = frame_id;
  meshROS.header.stamp = cmd->header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::msg::Marker::ADD;
  meshROS.pose.position.x = cmd->position.x;
  meshROS.pose.position.y = cmd->position.y;
  meshROS.pose.position.z = cmd->position.z;

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = 2.0;
  meshROS.scale.y = 2.0;
  meshROS.scale.z = 2.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub->publish(meshROS);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("odom_visualization");

  mesh_resource = node->declare_parameter<string>(
      "mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
  color_r = node->declare_parameter<double>("color.r", 1.0);
  color_g = node->declare_parameter<double>("color.g", 0.0);
  color_b = node->declare_parameter<double>("color.b", 0.0);
  color_a = node->declare_parameter<double>("color.a", 1.0);
  origin = node->declare_parameter<bool>("origin", false);
  scale = node->declare_parameter<double>("robot_scale", 2.0);
  frame_id = node->declare_parameter<string>("frame_id", "world");

  cross_config = node->declare_parameter<bool>("cross_config", false);
  tf45 = node->declare_parameter<bool>("tf45", false);
  cov_scale = node->declare_parameter<double>("covariance_scale", 100.0);
  cov_pos = node->declare_parameter<bool>("covariance_position", false);
  cov_vel = node->declare_parameter<bool>("covariance_velocity", false);
  cov_color = node->declare_parameter<bool>("covariance_color", false);
  drone_id = node->declare_parameter<int>("drone_id", 1);
  drone_num = node->declare_parameter<int>("drone_num", 1);

  (void)tf45;
  (void)cov_scale;
  (void)cov_pos;
  (void)cov_vel;
  (void)cov_color;
  (void)origin;
  (void)isOriginSet;
  (void)poseOrigin;
  (void)poseROS;
  (void)pathROS;
  (void)velROS;
  (void)covROS;
  (void)covVelROS;
  (void)trajROS;
  (void)sensorROS;
  (void)heightROS;

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>("~/odom", 100, odom_callback);
  auto sub_cmd =
      node->create_subscription<quadrotor_msgs::msg::PositionCommand>("~/cmd", 100, cmd_callback);
  (void)sub_odom;
  (void)sub_cmd;

  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable();
  posePub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", pub_qos);
  pathPub = node->create_publisher<nav_msgs::msg::Path>("~/path", pub_qos);
  velPub = node->create_publisher<visualization_msgs::msg::Marker>("~/velocity", pub_qos);
  covPub = node->create_publisher<visualization_msgs::msg::Marker>("~/covariance", pub_qos);
  covVelPub =
      node->create_publisher<visualization_msgs::msg::Marker>("~/covariance_velocity", pub_qos);
  trajPub = node->create_publisher<visualization_msgs::msg::Marker>("~/trajectory", pub_qos);
  sensorPub = node->create_publisher<visualization_msgs::msg::Marker>("~/sensor", pub_qos);
  meshPub = node->create_publisher<visualization_msgs::msg::Marker>("~/robot", pub_qos);
  heightPub = node->create_publisher<sensor_msgs::msg::Range>("~/height", pub_qos);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
