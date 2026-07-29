// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_rtk_spline_planner/rtk_spline_planner.hpp"

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_rtk_spline_planner
{

RtkSplinePlanner::RtkSplinePlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

RtkSplinePlanner::~RtkSplinePlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type RtkSplinePlanner",
    name_.c_str());
}

void RtkSplinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring plugin %s of type RtkSplinePlanner", name_.c_str());

  // Declare and read parameters
  declare_parameter_if_not_declared(
    node, name_ + ".path_resolution", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, name_ + ".search_radius", rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(
    node, name_ + ".use_waypoint_topic", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".waypoint_topic", rclcpp::ParameterValue("rtk_waypoints"));
  declare_parameter_if_not_declared(
    node, name_ + ".check_costmap_collision", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".waypoints", rclcpp::ParameterValue(std::vector<double>{}));

  node->get_parameter(name_ + ".path_resolution", path_resolution_);
  node->get_parameter(name_ + ".search_radius", search_radius_);
  node->get_parameter(name_ + ".use_waypoint_topic", use_waypoint_topic_);
  node->get_parameter(name_ + ".waypoint_topic", waypoint_topic_);
  node->get_parameter(name_ + ".check_costmap_collision", check_costmap_collision_);

  // Load waypoints from parameter
  loadWaypointsFromParam();

  RCLCPP_INFO(
    logger_, "RtkSplinePlanner configured: resolution=%.3f, search_radius=%.1f, "
    "waypoints=%zu, total_length=%.1f m",
    path_resolution_, search_radius_, wp_x_.size(), spline_.totalLength());
}

void RtkSplinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type RtkSplinePlanner", name_.c_str());

  auto node = node_.lock();

  // Subscribe to dynamic waypoint topic if enabled
  if (use_waypoint_topic_) {
    waypoint_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      waypoint_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&RtkSplinePlanner::waypointPathCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "Subscribing to waypoint topic: %s", waypoint_topic_.c_str());
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&RtkSplinePlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void RtkSplinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type RtkSplinePlanner", name_.c_str());
  dyn_params_handler_.reset();
  waypoint_sub_.reset();
}

void RtkSplinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type RtkSplinePlanner", name_.c_str());
  std::lock_guard<std::mutex> lock(spline_mutex_);
  wp_x_.clear();
  wp_y_.clear();
}

// =============================================================================
// createPlan — core planning logic
// =============================================================================
nav_msgs::msg::Path RtkSplinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  std::lock_guard<std::mutex> lock(spline_mutex_);

  if (spline_.empty()) {
    RCLCPP_ERROR(logger_, "RtkSplinePlanner: spline is empty, no waypoints loaded");
    return path;
  }

  // Project start and goal onto the spline
  double s_start = spline_.findClosest(
    start.pose.position.x, start.pose.position.y, 0.0, search_radius_);
  double s_goal = spline_.findClosest(
    goal.pose.position.x, goal.pose.position.y, s_start, search_radius_);

  RCLCPP_DEBUG(
    logger_,
    "RtkSplinePlanner: start projected to s=%.2f, goal projected to s=%.2f (total=%.2f)",
    s_start, s_goal, spline_.totalLength());

  // Sample the spline between s_start and s_goal
  std::vector<double> xs, ys, yaws;
  spline_.sample(path_resolution_, s_start, s_goal, xs, ys, yaws);

  if (xs.empty()) {
    RCLCPP_WARN(logger_, "RtkSplinePlanner: sampled path is empty");
    return path;
  }

  // Build nav_msgs/Path
  path.poses.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); ++i) {
    if (check_costmap_collision_ && !isCollisionFree(xs[i], ys[i])) {
      RCLCPP_WARN(
        logger_,
        "RtkSplinePlanner: collision detected at (%.2f, %.2f); "
        "planning failed because this planner cannot detour",
        xs[i], ys[i]);
      throw nav2_core::PlannerException(
              "RTK spline blocked at (" + std::to_string(xs[i]) + ", " +
              std::to_string(ys[i]) + "); no collision-free path available");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = xs[i];
    pose.pose.position.y = ys[i];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaws[i]);
    path.poses.push_back(pose);
  }

  // Use the nearest RTK waypoint heading for the terminal pose so the
  // controller can rotate base_link to match the waypoint orientation.
  if (!path.poses.empty() && !wp_yaw_.empty()) {
    const size_t goal_wp_idx = findClosestWaypointIndex(
      goal.pose.position.x, goal.pose.position.y);
    const double goal_yaw = wp_yaw_[goal_wp_idx];
    path.poses.back().pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(goal_yaw);
  }

  RCLCPP_INFO(
    logger_, "RtkSplinePlanner: created path with %zu poses (%.2f m)",
    path.poses.size(),
    std::abs(s_goal - s_start));

  return path;
}

// =============================================================================
// Waypoint loading
// =============================================================================
void RtkSplinePlanner::loadWaypointsFromParam()
{
  auto node = node_.lock();
  if (!node) {return;}

  std::vector<double> flat_waypoints;
  try {
    node->get_parameter(name_ + ".waypoints", flat_waypoints);
  } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
    RCLCPP_WARN(logger_, "RtkSplinePlanner: waypoints parameter type mismatch, skipping");
    return;
  }

  if (flat_waypoints.empty()) {
    RCLCPP_WARN(
      logger_,
      "RtkSplinePlanner: no waypoints loaded from parameter '%s.waypoints'. "
      "Provide waypoints as [x1, y1, x2, y2, ...] or enable waypoint_topic.",
      name_.c_str());
    return;
  }

  if (flat_waypoints.size() % 2 != 0) {
    RCLCPP_ERROR(
      logger_,
      "RtkSplinePlanner: waypoints parameter must have even number of elements "
      "(x1, y1, x2, y2, ...). Got %zu elements.", flat_waypoints.size());
    return;
  }

  std::lock_guard<std::mutex> lock(spline_mutex_);
  wp_x_.clear();
  wp_y_.clear();
  wp_yaw_.clear();

  for (size_t i = 0; i < flat_waypoints.size(); i += 2) {
    wp_x_.push_back(flat_waypoints[i]);
    wp_y_.push_back(flat_waypoints[i + 1]);
    if (i + 2 < flat_waypoints.size()) {
      const double dx = flat_waypoints[i + 2] - flat_waypoints[i];
      const double dy = flat_waypoints[i + 3] - flat_waypoints[i + 1];
      wp_yaw_.push_back(std::atan2(dy, dx));
    } else if (!wp_yaw_.empty()) {
      wp_yaw_.push_back(wp_yaw_.back());
    } else {
      wp_yaw_.push_back(0.0);
    }
  }

  RCLCPP_INFO(logger_, "RtkSplinePlanner: loaded %zu waypoints from parameter", wp_x_.size());
  rebuildSpline();
}

void RtkSplinePlanner::rebuildSpline()
{
  if (wp_x_.size() < 2) {
    RCLCPP_WARN(
      logger_,
      "RtkSplinePlanner: need at least 2 waypoints to build spline, got %zu",
      wp_x_.size());
    return;
  }

  try {
    spline_.build(wp_x_, wp_y_);
    RCLCPP_INFO(
      logger_, "RtkSplinePlanner: spline rebuilt with %zu waypoints, total length=%.2f m",
      wp_x_.size(), spline_.totalLength());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "RtkSplinePlanner: failed to build spline: %s", e.what());
  }
}

void RtkSplinePlanner::waypointPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.size() < 2) {
    RCLCPP_WARN(logger_, "RtkSplinePlanner: received path with fewer than 2 poses, ignoring");
    return;
  }

  std::lock_guard<std::mutex> lock(spline_mutex_);

  // Skip rebuild if waypoints haven't changed
  if (msg->poses.size() == wp_x_.size()) {
    bool same = true;
    for (size_t i = 0; i < wp_x_.size() && same; ++i) {
      same = (std::abs(msg->poses[i].pose.position.x - wp_x_[i]) < 1e-6 &&
              std::abs(msg->poses[i].pose.position.y - wp_y_[i]) < 1e-6);
    }
    if (same) {return;}
  }

  wp_x_.clear();
  wp_y_.clear();
  wp_yaw_.clear();

  for (const auto & pose : msg->poses) {
    wp_x_.push_back(pose.pose.position.x);
    wp_y_.push_back(pose.pose.position.y);
    wp_yaw_.push_back(tf2::getYaw(pose.pose.orientation));
  }

  RCLCPP_INFO(
    logger_, "RtkSplinePlanner: received %zu waypoints from topic", wp_x_.size());
  rebuildSpline();
}

// =============================================================================
// Costmap collision check
// =============================================================================
bool RtkSplinePlanner::isCollisionFree(double wx, double wy) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;  // off-map treated as free (robot is in open field)
  }
  unsigned char cost = costmap_->getCost(mx, my);
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// =============================================================================
// Dynamic parameters
// =============================================================================
rcl_interfaces::msg::SetParametersResult
RtkSplinePlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & type = param.get_type();
    const auto & pname = param.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (pname == name_ + ".path_resolution") {
        path_resolution_ = param.as_double();
        RCLCPP_INFO(logger_, "RtkSplinePlanner: path_resolution updated to %.3f", path_resolution_);
      } else if (pname == name_ + ".search_radius") {
        search_radius_ = param.as_double();
        RCLCPP_INFO(logger_, "RtkSplinePlanner: search_radius updated to %.1f", search_radius_);
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (pname == name_ + ".check_costmap_collision") {
        check_costmap_collision_ = param.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (pname == name_ + ".waypoints") {
        auto flat = param.as_double_array();
        if (flat.size() >= 4 && flat.size() % 2 == 0) {
          std::lock_guard<std::mutex> lock(spline_mutex_);
          wp_x_.clear();
          wp_y_.clear();
          wp_yaw_.clear();
          for (size_t i = 0; i < flat.size(); i += 2) {
            wp_x_.push_back(flat[i]);
            wp_y_.push_back(flat[i + 1]);
            if (i + 2 < flat.size()) {
              const double dx = flat[i + 2] - flat[i];
              const double dy = flat[i + 3] - flat[i + 1];
              wp_yaw_.push_back(std::atan2(dy, dx));
            } else if (!wp_yaw_.empty()) {
              wp_yaw_.push_back(wp_yaw_.back());
            } else {
              wp_yaw_.push_back(0.0);
            }
          }
          rebuildSpline();
          RCLCPP_INFO(logger_, "RtkSplinePlanner: waypoints updated dynamically (%zu pts)",
            wp_x_.size());
        }
      }
    }
  }

  return result;
}

size_t RtkSplinePlanner::findClosestWaypointIndex(double x, double y) const
{
  if (wp_x_.empty()) {
    return 0;
  }

  size_t best_idx = 0;
  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < wp_x_.size(); ++i) {
    const double dx = wp_x_[i] - x;
    const double dy = wp_y_[i] - y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_idx = i;
    }
  }
  return best_idx;
}

}  // namespace nav2_rtk_spline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rtk_spline_planner::RtkSplinePlanner, nav2_core::GlobalPlanner)
