#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}

using namespace cv;
using namespace std;
using namespace Eigen;

cv::Mat depth_mat;

int width;
int height;
double fx;
double fy;
double cx;
double cy;
double sensing_horizon;
double sensing_rate;
double estimation_rate;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
rclcpp::TimerBase::SharedPtr local_sensing_timer;
rclcpp::TimerBase::SharedPtr estimation_timer;

bool has_global_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::msg::Odometry odom;

builtin_interfaces::msg::Time last_odom_stamp;
Eigen::Vector3d last_pose_world;
pcl::PointCloud<pcl::PointXYZ> cloudIn;

void odometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  has_odom = true;
  odom = *odom_msg;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom_msg->pose.pose.position.x;
  request_position.y() = odom_msg->pose.pose.position.y;
  request_position.z() = odom_msg->pose.pose.position.z;
  request_pose.x() = odom_msg->pose.pose.orientation.x;
  request_pose.y() = odom_msg->pose.pose.orientation.y;
  request_pose.z() = odom_msg->pose.pose.orientation.z;
  request_pose.w() = odom_msg->pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);
  last_odom_stamp = odom_msg->header.stamp;

  last_pose_world(0) = odom_msg->pose.pose.position.x;
  last_pose_world(1) = odom_msg->pose.pose.position.y;
  last_pose_world(2) = odom_msg->pose.pose.position.z;
}

void pubCameraPose() {
  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header = odom.header;
  camera_pose.header.frame_id = "/map";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose->publish(camera_pose);
}

void pointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  if (has_global_map) return;

  RCLCPP_WARN(node->get_logger(), "Global Pointcloud received..");
  pcl::fromROSMsg(*pointcloud_map, cloudIn);
  printf("global map has points: %zu.\n", cloudIn.points.size());
  has_global_map = true;
}

void renderDepth() {
  Matrix4d cam_pose = cam2world.inverse();
  (void)cam_pose;

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);

  Eigen::Matrix4d Tcw = cam2world.inverse();
  Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
  Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);

  Eigen::Vector3d pos = cam2world.block<3, 1>(0, 3);
  for (auto pt : cloudIn.points) {
    Eigen::Vector3d pw(pt.x, pt.y, pt.z);
    if ((pos - pw).norm() > 5.0) continue;

    Eigen::Vector3d pc = Rcw * pw + tcw;

    if (pc[2] <= 0.0) continue;

    float projected_x;
    float projected_y;
    projected_x = pc[0] / pc[2] * fx + cx;
    projected_y = pc[1] / pc[2] * fy + cy;
    if (projected_x < 0 || projected_x >= width || projected_y < 0 || projected_y >= height)
      continue;

    float dist = pc[2];
    int r = 0.0573 * fx / dist + 0.5;
    int min_x = max(int(projected_x - r), 0);
    int max_x = min(int(projected_x + r), width - 1);
    int min_y = max(int(projected_y - r), 0);
    int max_y = min(int(projected_y + r), height - 1);

    for (int to_x = min_x; to_x <= max_x; to_x++) {
      for (int to_y = min_y; to_y <= max_y; to_y++) {
        float value = depth_mat.at<float>(to_y, to_x);
        if (value < 1e-3) {
          depth_mat.at<float>(to_y, to_x) = dist;
        } else {
          depth_mat.at<float>(to_y, to_x) = min(value, dist);
        }
      }
    }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "world";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth->publish(*out_msg.toImageMsg());
}

void renderSensedPoints() {
  if (!has_global_map) return;
  renderDepth();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("pcl_render");

  width = node->declare_parameter<int>("cam_width", 640);
  height = node->declare_parameter<int>("cam_height", 480);
  fx = node->declare_parameter<double>("cam_fx", 387.229);
  fy = node->declare_parameter<double>("cam_fy", 387.229);
  cx = node->declare_parameter<double>("cam_cx", 321.046);
  cy = node->declare_parameter<double>("cam_cy", 243.449);
  sensing_horizon = node->declare_parameter<double>("sensing_horizon", 5.0);
  sensing_rate = node->declare_parameter<double>("sensing_rate", 10.0);
  estimation_rate = node->declare_parameter<double>("estimation_rate", 10.0);
  node->declare_parameter<double>("map.x_size", 35.0);
  node->declare_parameter<double>("map.y_size", 35.0);
  node->declare_parameter<double>("map.z_size", 3.5);
  node->declare_parameter<double>("map.init_x", 0.0);
  node->declare_parameter<double>("map.init_y", 0.0);
  node->declare_parameter<int>("map.drone_id", 1);
  (void)sensing_horizon;

  cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0;

  cam2world = Matrix4d::Identity();

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_generator/global_cloud", map_qos, pointCloudCallBack);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("~/odometry", 50, odometryCallbck);

  pub_depth = node->create_publisher<sensor_msgs::msg::Image>("/pcl_render_node/depth", 1000);
  pub_pose =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/pcl_render_node/sensor_pose", 1000);

  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer =
      node->create_wall_timer(std::chrono::duration<double>(sensing_duration), renderSensedPoints);
  estimation_timer =
      node->create_wall_timer(std::chrono::duration<double>(estimate_duration), pubCameraPose);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
