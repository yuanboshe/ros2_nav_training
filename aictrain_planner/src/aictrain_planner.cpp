#include "aictrain_planner/aictrain_planner.hpp"

namespace aictrain_planner
{

void TrainPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<train::Train>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  // planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
}

void TrainPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
}

void TrainPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

void TrainPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

nav_msgs::msg::Path TrainPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = global_path.header;
  pose.pose.position.z = 0.0;

  pose.pose = start.pose;
  global_path.poses.push_back(pose);

  pose.pose = goal.pose;
  global_path.poses.push_back(pose);

  return global_path;
}
}  // namespace aictrain_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(aictrain_planner::TrainPlanner, nav2_core::GlobalPlanner)