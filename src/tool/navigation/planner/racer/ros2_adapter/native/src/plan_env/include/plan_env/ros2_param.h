#ifndef _ROS2_PARAM_H_
#define _ROS2_PARAM_H_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace fast_planner {

template <typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }

  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

}  // namespace fast_planner

#endif
