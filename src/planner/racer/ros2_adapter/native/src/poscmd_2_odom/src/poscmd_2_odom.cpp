#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using nav_msgs::msg::Odometry;
using quadrotor_msgs::msg::PositionCommand;

rclcpp::Node::SharedPtr node;
rclcpp::Subscription<PositionCommand>::SharedPtr cmd_sub;
rclcpp::Publisher<Odometry>::SharedPtr odom_pub;

PositionCommand cmd;
double init_x = 0.0;
double init_y = 0.0;
double init_z = 0.0;
int drone_id = 1;
bool rcv_cmd = false;

void rcvPosCmdCallBack(const PositionCommand::SharedPtr msg) {
  rcv_cmd = true;
  cmd = *msg;
}

void pubOdom() {
  Odometry odom;
  odom.header.stamp = node->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = std::to_string(drone_id);

  if (rcv_cmd) {
    odom.pose.pose.position.x = cmd.position.x;
    odom.pose.pose.position.y = cmd.position.y;
    odom.pose.pose.position.z = cmd.position.z;

    Eigen::Vector3d alpha(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
    alpha += 9.8 * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d xC(std::cos(cmd.yaw), std::sin(cmd.yaw), 0);
    Eigen::Vector3d yC(-std::sin(cmd.yaw), std::cos(cmd.yaw), 0);
    Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    Eigen::Vector3d zB = xB.cross(yB);
    Eigen::Matrix3d R;
    R.col(0) = xB;
    R.col(1) = yB;
    R.col(2) = zB;
    Eigen::Quaterniond q(R);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    odom.twist.twist.linear.x = cmd.velocity.x;
    odom.twist.twist.linear.y = cmd.velocity.y;
    odom.twist.twist.linear.z = cmd.velocity.z;

    odom.twist.twist.angular.x = cmd.acceleration.x;
    odom.twist.twist.angular.y = cmd.acceleration.y;
    odom.twist.twist.angular.z = cmd.acceleration.z;
  } else {
    odom.pose.pose.position.x = init_x;
    odom.pose.pose.position.y = init_y;
    odom.pose.pose.position.z = init_z;

    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
  }

  odom_pub->publish(odom);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("odom_generator");

  init_x = node->declare_parameter<double>("init_x", 0.0);
  init_y = node->declare_parameter<double>("init_y", 0.0);
  init_z = node->declare_parameter<double>("init_z", 0.0);
  drone_id = node->declare_parameter<int>("drone_id", 1);

  cmd_sub = node->create_subscription<PositionCommand>("~/command", 1, rcvPosCmdCallBack);
  odom_pub = node->create_publisher<Odometry>("~/odometry", 1);

  rclcpp::WallRate rate(100);
  while (rclcpp::ok()) {
    pubOdom();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
