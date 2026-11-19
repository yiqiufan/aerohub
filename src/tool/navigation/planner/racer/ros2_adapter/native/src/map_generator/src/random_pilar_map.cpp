#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
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

random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;

int obs_num;
int retry_num;
double x_size;
double y_size;
double min_r;
double max_r;
double resolution;
double fb_min_x;
double fb_max_x;
double fb_min_y;
double fb_max_y;
double min_x;
double max_x;
double min_y;
double max_y;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub;
sensor_msgs::msg::PointCloud2 map_msg;
pcl::PointCloud<pcl::PointXYZ> map_cloud;

vector<Eigen::Vector3d> points;

void generatePilarMap() {
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(min_x, max_x);
  rand_y = uniform_real_distribution<double>(min_y, max_y);
  rand_w = uniform_real_distribution<double>(min_r, max_r);

  for (double x = -16; x <= 16; x += resolution) {
    for (double y = -16; y <= 16; y += resolution) {
      pt_random.x = x;
      pt_random.y = y;
      pt_random.z = 0.0;
      map_cloud.points.push_back(pt_random);
    }
  }

  int gen_num = 0;
  int fail_num = 0;
  while (gen_num < obs_num) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double w = rand_w(eng);

    if (x < fb_max_x && x > fb_min_x && y < fb_max_y && y > fb_min_y) continue;

    x = floor(x / resolution) * resolution + resolution / 2.0;
    y = floor(y / resolution) * resolution + resolution / 2.0;
    Eigen::Vector3d pt(x, y, w);
    bool occupied = false;
    for (auto pre : points) {
      if ((pre - pt).head<2>().norm() < 1.414 * (pre[2] + pt[2]) + 2.0) {
        occupied = true;
        break;
      }
    }
    if (occupied) {
      if (++fail_num > retry_num) {
        return;
      }
      continue;
    }
    points.push_back(pt);

    int wid_num = ceil(w / resolution);
    for (int r = -wid_num; r < wid_num; r++) {
      for (int s = -wid_num; s < wid_num; s++) {
        int hei_num = ceil(3.0 / resolution);
        for (int t = -10; t < hei_num; t++) {
          pt_random.x = x + r * resolution + 1e-2;
          pt_random.y = y + s * resolution + 1e-2;
          pt_random.z = (t + 0.5) * resolution + 1e-2;
          map_cloud.points.push_back(pt_random);
        }
      }
    }
    gen_num++;
    fail_num = 0;
  }

  map_cloud.width = map_cloud.points.size();
  map_cloud.height = 1;
  map_cloud.is_dense = true;
  RCLCPP_WARN(node->get_logger(), "Finished generate random map ");
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("pilar_map");

  all_map_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 1);

  x_size = node->declare_parameter<double>("map.x_size", 50.0);
  y_size = node->declare_parameter<double>("map.y_size", 50.0);
  obs_num = node->declare_parameter<int>("map.obs_num", 30);
  retry_num = node->declare_parameter<int>("map.retry_num", 30);
  resolution = node->declare_parameter<double>("map.resolution", 0.1);
  min_r = node->declare_parameter<double>("map.min_r", 0.3);
  max_r = node->declare_parameter<double>("map.max_r", 0.8);
  fb_min_x = node->declare_parameter<double>("map.fb_min_x", 0.8);
  fb_max_x = node->declare_parameter<double>("map.fb_max_x", 0.8);
  fb_min_y = node->declare_parameter<double>("map.fb_min_y", 0.8);
  fb_max_y = node->declare_parameter<double>("map.fb_max_y", 0.8);

  min_x = -x_size / 2.0;
  max_x = +x_size / 2.0;

  min_y = -y_size / 2.0;
  max_y = +y_size / 2.0;

  obs_num = min(obs_num, static_cast<int>(x_size) * 10);

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  int seed = node->declare_parameter<int>("map.seed", -1);
  if (seed < 0) {
    seed = rd() % INT32_MAX;
  }
  std::cout << "map seed: " << seed << std::endl;
  eng = default_random_engine(seed);

  generatePilarMap();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    pcl::toROSMsg(map_cloud, map_msg);
    map_msg.header.frame_id = "world";
    all_map_pub->publish(map_msg);

    rclcpp::spin_some(node);
    rate.sleep();
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "seed: %d", seed);
  }

  rclcpp::shutdown();
  return 0;
}
