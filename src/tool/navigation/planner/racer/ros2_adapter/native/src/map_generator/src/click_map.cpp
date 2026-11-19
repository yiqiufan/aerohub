#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

namespace {

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub;
sensor_msgs::msg::PointCloud2 map_msg;
pcl::PointCloud<pcl::PointXYZ> map_cloud;

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_sub;
vector<Eigen::Vector3d> points;
double len2;

void clickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  points.push_back(Eigen::Vector3d(x, y, 0));
  if (points.size() < 2) return;

  Eigen::Vector3d p1 = points[0];
  Eigen::Vector3d p2 = points[1];
  points.clear();

  Eigen::Vector3d dir1 = (p2 - p1).normalized();
  double len = (p2 - p1).norm();
  Eigen::Vector3d dir2;
  dir2[0] = -dir1[1];
  dir2[1] = dir1[0];

  pcl::PointXYZ pt_random;
  for (double l1 = 0.0; l1 <= len + 1e-3; l1 += 0.1) {
    Eigen::Vector3d tmp1 = p1 + l1 * dir1;
    for (double l2 = -len2; l2 <= len2 + 1e-3; l2 += 0.1) {
      Eigen::Vector3d tmp2 = tmp1 + l2 * dir2;
      for (double h = -0.5; h < 2.5; h += 0.1) {
        pt_random.x = tmp2[0];
        pt_random.y = tmp2[1];
        pt_random.z = h;
        map_cloud.push_back(pt_random);
      }
    }
  }

  map_cloud.width = map_cloud.points.size();
  map_cloud.height = 1;
  map_cloud.is_dense = true;
  pcl::toROSMsg(map_cloud, map_msg);
  map_msg.header.frame_id = "world";
  all_map_pub->publish(map_msg);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("click_map");

  len2 = node->declare_parameter<double>("map.len2", 0.15);

  all_map_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 1);

  click_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, clickCallback);

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    pcl::toROSMsg(map_cloud, map_msg);
    map_msg.header.frame_id = "world";
    all_map_pub->publish(map_msg);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
