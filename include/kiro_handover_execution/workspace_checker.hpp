#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

struct UR10eDH
{
    static constexpr double MIN_REACH = 0.25; // instead of 0.19, which is inner dead zone radius
    static constexpr double MAX_REACH = 1.00; // instead of 1.30, which is full arm extension
    static constexpr double FORWARD_MARGIN = 0.3; // min distance ahead of start
};

class WorkspaceChecker
{
public:
    explicit WorkspaceChecker(const Eigen::Vector3d & base_in_world);

    // True if dist(p, base) is within [MIN_REACH, MAX_REACH]
    bool isReachable(const Eigen::Vector3d & p_world) const;

    // If needed 'pushes' p onto the reachable shell along the base to p direction
    Eigen::Vector3d project(const Eigen::Vector3d & p_world) const;

    geometry_msgs::msg::Point projectPoint(const geometry_msgs::msg::Point & p) const;

    // Project P1 and P2 such that:
    //   1. dist(P, base) in [MIN_REACH, MAX_REACH]
    //   2. Prevents the arc from swinging back behind the base
    // Returns true if both were already valid (no change made)
    bool adjustControlPoints(
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & target,
        geometry_msgs::msg::Point & P1,
        geometry_msgs::msg::Point & P2) const;

private:
    Eigen::Vector3d base_in_world_;

    // Clamp a single point — reach limits + forward halfspace
    Eigen::Vector3d clampPoint(
        const Eigen::Vector3d & p,
        const Eigen::Vector3d & forward_normal,
        const Eigen::Vector3d & plane_origin) const;
};