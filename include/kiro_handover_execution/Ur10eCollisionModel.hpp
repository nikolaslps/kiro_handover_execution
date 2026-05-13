#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <vector>

struct CollisionCapsule
{
    std::string link_name;
    double radius;
    double length;
    Eigen::Isometry3d transform; // global pose of the link origin
};

class Ur10eCollisionModel
{
public:
    Ur10eCollisionModel();

    // updates cylinders position (collision capsules)
    void updatePoses(const moveit::core::RobotState& robot_state);

    bool isColliding(const std::vector<geometry_msgs::msg::Point>& obstacles, 
                    double safety_margin = 0.05) const;

    double getMinDistance(const std::vector<geometry_msgs::msg::Point>& obstacles) const;

    visualization_msgs::msg::MarkerArray getCapsuleMarkers(
        const std::string& frame_id = "kiro_base_link", 
        const std::string& ns = "robot_capsules") const;

private:
    std::vector<CollisionCapsule> capsules_;

    double pointToSegmentDistance(const Eigen::Vector3d& p, 
                                const Eigen::Vector3d& a, 
                                const Eigen::Vector3d& b) const;
};