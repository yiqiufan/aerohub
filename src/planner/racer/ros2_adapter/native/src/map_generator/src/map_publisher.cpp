#include <chrono>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_pub");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto cloud_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", qos);

  const std::string file_name = argv[1];
  rclcpp::sleep_for(std::chrono::seconds(1));

  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1) {
    std::cout << "can't read file." << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  for (double x = -7; x <= 7; x += 0.1) {
    for (double y = -15; y <= 15; y += 0.1) {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    cloud_pub->publish(msg);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  std::cout << "finish publish map." << std::endl;

  rclcpp::shutdown();
  return 0;
}
