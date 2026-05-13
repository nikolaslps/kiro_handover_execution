#include "workspace_checker.hpp"
#include <cmath>

static const auto LOGGER = rclcpp::get_logger("workspace_checker");

WorkspaceChecker::WorkspaceChecker(const Eigen::Vector3d & base_in_world): base_in_world_(base_in_world)
{
    RCLCPP_INFO(LOGGER,
        "WorkspaceChecker ready — UR10e base at [%.3f, %.3f, %.3f]  "
        "reach=[%.3f, %.3f] m",
        base_in_world.x(), base_in_world.y(), base_in_world.z(),
        UR10eDH::MIN_REACH, UR10eDH::MAX_REACH);
}

bool WorkspaceChecker::isReachable(const Eigen::Vector3d & p_world) const
{
    double dist = (p_world - base_in_world_).norm();
    return dist >= UR10eDH::MIN_REACH && dist <= UR10eDH::MAX_REACH;
}

Eigen::Vector3d WorkspaceChecker::project(const Eigen::Vector3d & p_world) const
{
    Eigen::Vector3d v = p_world - base_in_world_;
    double dist = v.norm();

    if (dist < 1e-9) {
        RCLCPP_WARN(LOGGER, "project: point coincides with base — pushing to MIN_REACH along +X.");
        return base_in_world_ + Eigen::Vector3d(UR10eDH::MIN_REACH, 0.0, 0.0);
    }

    Eigen::Vector3d dir = v / dist;

    if (dist < UR10eDH::MIN_REACH)
        return base_in_world_ + dir * UR10eDH::MIN_REACH;

    if (dist > UR10eDH::MAX_REACH)
        return base_in_world_ + dir * UR10eDH::MAX_REACH;

    return p_world;
}

geometry_msgs::msg::Point WorkspaceChecker::projectPoint(
    const geometry_msgs::msg::Point & p) const
{
    Eigen::Vector3d proj = project(Eigen::Vector3d(p.x, p.y, p.z));
    geometry_msgs::msg::Point out;
    out.x = proj.x(); out.y = proj.y(); out.z = proj.z();
    return out;
}

Eigen::Vector3d WorkspaceChecker::clampPoint(
    const Eigen::Vector3d & p,
    const Eigen::Vector3d & forward_normal,
    const Eigen::Vector3d & plane_origin) const
{
    // reachability constraint
    Eigen::Vector3d clamped = project(p);

    // forward halfspace constraint
    double signed_dist = forward_normal.dot(clamped - plane_origin);

    if (signed_dist < 0.0) {
        clamped = clamped - signed_dist * forward_normal;

        RCLCPP_DEBUG(LOGGER, "clampPoint: point was %.3fm behind forward plane — projected onto plane.", -signed_dist);

        // re-enforce reach limits after halfspace projection
        clamped = project(clamped);
    }

    return clamped;
}

bool WorkspaceChecker::adjustControlPoints(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & target,
    geometry_msgs::msg::Point & P1,
    geometry_msgs::msg::Point & P2) const
{
    const double FORWARD_MARGIN = UR10eDH::FORWARD_MARGIN;

    Eigen::Vector3d p_start(start.position.x, start.position.y, start.position.z);
    Eigen::Vector3d p_target(target.position.x, target.position.y, target.position.z);

    Eigen::Vector3d motion_vec = p_target - p_start;

    bool all_inside = true;

    if (motion_vec.norm() < 1e-6) {
        // start == target — skip halfspace, reach only
        RCLCPP_WARN(LOGGER,
            "adjustControlPoints: start and target coincide — "
            "skipping halfspace constraint.");

        Eigen::Vector3d p1(P1.x, P1.y, P1.z);
        Eigen::Vector3d p1c = project(p1);
        if ((p1c - p1).norm() > 1e-6) {
            all_inside = false;
            P1.x = p1c.x(); P1.y = p1c.y(); P1.z = p1c.z();
        }

        Eigen::Vector3d p2(P2.x, P2.y, P2.z);
        Eigen::Vector3d p2c = project(p2);
        if ((p2c - p2).norm() > 1e-6) {
            all_inside = false;
            P2.x = p2c.x(); P2.y = p2c.y(); P2.z = p2c.z();
        }

        return all_inside;
    }

    Eigen::Vector3d forward_normal = motion_vec.normalized();

    // Shift the plane FORWARD_MARGIN metres ahead of start along the motion
    Eigen::Vector3d plane_origin = p_start + forward_normal * FORWARD_MARGIN;

    // for control point P1, enforce reachability and forward halfspace
    Eigen::Vector3d p1(P1.x, P1.y, P1.z);
    Eigen::Vector3d p1c = clampPoint(p1, forward_normal, plane_origin);

    if ((p1c - p1).norm() > 1e-6) {
        all_inside = false;
        RCLCPP_INFO(LOGGER,
            "P1 adjusted: [%.3f,%.3f,%.3f] to [%.3f,%.3f,%.3f] (moved %.3fm)",
            P1.x, P1.y, P1.z,
            p1c.x(), p1c.y(), p1c.z(),
            (p1c - p1).norm());
        P1.x = p1c.x(); P1.y = p1c.y(); P1.z = p1c.z();
    }

    // for control point P2, enforce reachability and forward halfspace
    Eigen::Vector3d p2(P2.x, P2.y, P2.z);
    Eigen::Vector3d p2c = clampPoint(p2, forward_normal, plane_origin);

    if ((p2c - p2).norm() > 1e-6) {
        all_inside = false;
        RCLCPP_INFO(LOGGER,
            "P2 adjusted: [%.3f,%.3f,%.3f] to [%.3f,%.3f,%.3f] (moved %.3fm)",
            P2.x, P2.y, P2.z,
            p2c.x(), p2c.y(), p2c.z(),
            (p2c - p2).norm());
        P2.x = p2c.x(); P2.y = p2c.y(); P2.z = p2c.z();
    }

    if (all_inside)
        RCLCPP_INFO(LOGGER, "P1 and P2 are valid — no adjustment needed.");

    return all_inside;
}