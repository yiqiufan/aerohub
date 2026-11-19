#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

bool finish = false;

void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::io::savePCDFileASCII("/home/boboyu/Downloads/tmp.pcd", cloud);

  std::cout << "map saved." << std::endl;
  finish = true;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_recorder");

  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_generator/global_cloud", 10, cloudCallback);
  (void)cloud_sub;

  rclcpp::sleep_for(std::chrono::seconds(1));

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (finish) break;
    rate.sleep();
  }

  RCLCPP_WARN(node->get_logger(), "[Map Recorder]: finish record map.");
  rclcpp::shutdown();
  return 0;
}
