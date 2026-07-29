// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_RTK_SPLINE_PLANNER__RTK_SPLINE_PLANNER_HPP_
#define NAV2_RTK_SPLINE_PLANNER__RTK_SPLINE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_rtk_spline_planner/cubic_spline.hpp"

namespace nav2_rtk_spline_planner
{

/**
 * @brief RTK Spline Planner — Nav2 GlobalPlanner plugin.
 *
 * Workflow:
 *   1. Load RTK waypoints from YAML parameter (map-frame x/y pairs)
 *      or subscribe to a waypoint topic for dynamic updates.
 *   2. Build a CubicSpline2D through the waypoints.
 *   3. On createPlan(): project start & goal onto the spline,
 *      sample the sub-path at a configurable resolution, and
 *      return nav_msgs/Path.
 */
class RtkSplinePlanner : public nav2_core::GlobalPlanner
{
public:
  RtkSplinePlanner();
  ~RtkSplinePlanner();

  // --- nav2_core::GlobalPlanner interface ---

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief Load waypoints from the parameter server and rebuild the spline.
   * Waypoints are provided as a flat list [x1, y1, x2, y2, ...].
   */
  void loadWaypointsFromParam();

  /**
   * @brief Rebuild the internal cubic spline from current waypoints.
   */
  void rebuildSpline();

  /**
   * @brief Callback for dynamically updated waypoints on a topic.
   * Expects nav_msgs/Path with poses in the global frame.
   */
  void waypointPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Check whether a pose on the spline is collision-free in the costmap.
   */
  bool isCollisionFree(double wx, double wy) const;

  // --- Dynamic parameter callback ---
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RtkSplinePlanner")};
  std::string name_;

  // Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // Spline data
  CubicSpline2D spline_;
  std::vector<double> wp_x_;
  std::vector<double> wp_y_;
  std::vector<double> wp_yaw_;
  mutable std::mutex spline_mutex_;

  /**
   * @brief Find the waypoint index closest to a map-frame position.
   */
  size_t findClosestWaypointIndex(double x, double y) const;

  // Parameters
  double path_resolution_;         // sampling step along spline (m)
  double search_radius_;           // how far to search for closest point on spline (m)
  bool use_waypoint_topic_;        // subscribe to dynamic waypoints?
  std::string waypoint_topic_;
  bool check_costmap_collision_;   // check sampled poses against costmap?

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;

  // Dynamic parameter handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_rtk_spline_planner

#endif  // NAV2_RTK_SPLINE_PLANNER__RTK_SPLINE_PLANNER_HPP_
