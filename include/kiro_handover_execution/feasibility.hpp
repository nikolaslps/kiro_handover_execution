#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

struct FeasibilityResult
{
  bool is_feasible = false;
  bool in_collision = false;
  std::string message;
  std::vector<double> joint_values; // not using it
};

FeasibilityResult checkFeasibility(
    const moveit::core::RobotState & current_state,
    const planning_scene_monitor::LockedPlanningSceneRO & scene, // looking only for static obstacles ... 
    const std::string & planning_group,
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & ee_link,
    double ik_timeout_sec);