#include "feasibility.hpp"
#include <moveit/collision_detection/collision_common.h>

FeasibilityResult checkFeasibility(
    const moveit::core::RobotState & current_state,
    const planning_scene_monitor::LockedPlanningSceneRO & scene,
    const std::string & planning_group,
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & ee_link,
    double ik_timeout_sec)
{
    FeasibilityResult result;

    const moveit::core::JointModelGroup * jmg = current_state.getJointModelGroup(planning_group);
    if (!jmg) {
        result.is_feasible = false;
        result.message = "JointModelGroup not found: " + planning_group;
        return result;
    }

    // IK check 
    moveit::core::RobotState ik_state(current_state);

    bool ik_ok = false;
    try {
        ik_ok = ik_state.setFromIK(jmg, target_pose, ee_link, ik_timeout_sec);
    } catch (...) {
        ik_ok = ik_state.setFromIK(jmg, target_pose, ik_timeout_sec);
    }

    if (!ik_ok) {
        result.is_feasible = false;
        result.message = "No IK solution found within timeout.";
        return result;
    }

    // Collision check
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = planning_group;
    req.contacts = true;
    req.max_contacts = 50;
    req.max_contacts_per_pair = 5;

    scene->checkCollision(req, res, ik_state);

    if (res.collision) {
        result.is_feasible = false;
        result.in_collision = true;
        result.message = "IK feasible but target pose is in collision.";
        ik_state.copyJointGroupPositions(planning_group, result.joint_values);

        return result;
    }

    result.is_feasible = true;
    result.in_collision = false;
    result.message = "IK feasible and collision-free.";
    ik_state.copyJointGroupPositions(planning_group, result.joint_values);
    return result;
}