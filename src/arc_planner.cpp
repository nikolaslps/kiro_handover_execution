#include "arc_planner.hpp"
#include <rclcpp/rclcpp.hpp>

static const auto LOGGER = rclcpp::get_logger("arc_planner");

double planArc(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & label)
{
    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,   // eef_step: max 1cm between consecutive IK solutions
        0.0,    // jump_threshold: disabled <-- this needs to be checked
        trajectory);
    RCLCPP_INFO(LOGGER, "[%s] %.1f%% achieved.", label.c_str(), fraction * 100.0);
    return fraction;
}

bool executeTrajectory(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & label)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto result = move_group.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "[%s] Execution successful.", label.c_str());
        return true;
    }
    RCLCPP_ERROR(LOGGER, "[%s] Execution failed (code: %d).", label.c_str(), (int)result.val);
    return false;
}