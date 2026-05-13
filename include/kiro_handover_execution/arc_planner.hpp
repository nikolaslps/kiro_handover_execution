#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

// Plan a Cartesian arc through the given waypoints. Returns the fraction of the path achieved (0.0–1.0).
double planArc(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & label);

// Execute a pre-planned trajectory. Returns true on success.
bool executeTrajectory(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & label);