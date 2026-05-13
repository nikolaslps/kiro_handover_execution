#include "Ur10eCollisionModel.hpp"

Ur10eCollisionModel::Ur10eCollisionModel() {
    // {Link Name, Radius, Length} initial values
    capsules_ = {
        {"shoulder_link", 0.10, 0.1807, Eigen::Isometry3d::Identity()},
        {"upper_arm_link", 0.08, 0.6127, Eigen::Isometry3d::Identity()},
        {"forearm_link", 0.08, 0.6914, Eigen::Isometry3d::Identity()},
        {"wrist_1_link", 0.07, 0.17415, Eigen::Isometry3d::Identity()},
        {"wrist_2_link", 0.07, 0.11985, Eigen::Isometry3d::Identity()},
        {"wrist_3_link", 0.06, 0.11655, Eigen::Isometry3d::Identity()},

        // {"gripper_offset_link", 0.05, 0.1431, Eigen::Isometry3d::Identity()},
        // {"vacuum_gripper_link", 0.10, 0.15, Eigen::Isometry3d::Identity()},
        // {"camera_link", 0.04, 0.05, Eigen::Isometry3d::Identity()}
    };
}

void Ur10eCollisionModel::updatePoses(const moveit::core::RobotState& robot_state) {
    auto align_to_joints = [&](const std::string& current_link, const std::string& child_joint_link, double radius, double lateral_offset) {
        Eigen::Isometry3d parent_tf = robot_state.getGlobalLinkTransform(current_link);
        Eigen::Isometry3d child_tf = robot_state.getGlobalLinkTransform(child_joint_link);

        Eigen::Vector3d p1 = parent_tf.translation();
        Eigen::Vector3d p2 = child_tf.translation();
        Eigen::Vector3d v = p2 - p1; // vector hat points from parent link to child link
        double len = v.norm();
        Eigen::Vector3d center = p1 + 0.5 * v;

        Eigen::Isometry3d final_tf = Eigen::Isometry3d::Identity();
        final_tf.translate(center);
        
        // align cylinder (with z-axis up) so it points along the vector v
        if (len > 1e-6) {
            final_tf.rotate(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), v.normalized()));
        }

        for (auto& cap : capsules_) {
            if (cap.link_name == current_link) {
                if (cap.link_name == "upper_arm_link"){
                    final_tf.translate(Eigen::Vector3d(0, -0.1824, 0)); // a bit to the -y-axis
                }
                if (cap.link_name == "forearm_link") {
                    Eigen::Vector3d wrist_offset(0, 0, -0.1824); // shift the forearm_link capsule to the edge of the wrist link
                    Eigen::Vector3d global_wrist_offset = parent_tf.linear() * wrist_offset;
                    Eigen::Vector3d p2_offset = p2 + global_wrist_offset;

                    v = p2_offset - p1;
                    len = v.norm();
                    center = p1 + 0.5 * v;

                    final_tf = Eigen::Isometry3d::Identity();
                    final_tf.translate(center);
                    if (len > 1e-6) {
                        final_tf.rotate(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), v.normalized()));
                    }
                }
                cap.transform = final_tf;
                cap.length = len;
                cap.radius = radius;
            }
        }
    };

    align_to_joints("upper_arm_link", "forearm_link", 0.1, 0.0);
    align_to_joints("forearm_link", "wrist_1_link", 0.1, 0.0);
    
    Eigen::Isometry3d shoulder_tf = robot_state.getGlobalLinkTransform("shoulder_link");
    for (auto& cap : capsules_) {
        if (cap.link_name == "shoulder_link") {
            cap.transform = shoulder_tf;
        }

        // else if (cap.link_name == "gripper_offset_link" || 
        //         cap.link_name == "vacuum_gripper_link" || 
        //         cap.link_name == "camera_link") {

        //     cap.transform = robot_state.getGlobalLinkTransform(cap.link_name);
        // }
    }

    for (auto& cap : capsules_) {
        if (cap.link_name.find("wrist") != std::string::npos) {
            cap.transform = robot_state.getGlobalLinkTransform(cap.link_name);
        }
    }
}

bool Ur10eCollisionModel::isColliding(const std::vector<geometry_msgs::msg::Point>& obstacles, 
                                    double safety_margin) const {
    return getMinDistance(obstacles) < safety_margin;
}

double Ur10eCollisionModel::getMinDistance(const std::vector<geometry_msgs::msg::Point>& obstacles) const {
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& cap : capsules_) {
        // Since cap.transform is the CENTER of the capsule:
        // p1 is at -half length along local Z
        // p2 is at +half length along local Z
        Eigen::Vector3d p1 = cap.transform * Eigen::Vector3d(0, 0, -cap.length / 2.0);
        Eigen::Vector3d p2 = cap.transform * Eigen::Vector3d(0, 0, cap.length / 2.0);

        for (const auto& obs : obstacles) {
            Eigen::Vector3d obs_vec(obs.x, obs.y, obs.z);
            // distance from point to segment minus the radius
            double d = pointToSegmentDistance(obs_vec, p1, p2) - cap.radius;
            if (d < min_dist) min_dist = d;
        }
    }
    return min_dist;
}

double Ur10eCollisionModel::pointToSegmentDistance(const Eigen::Vector3d& p, 
                                                    const Eigen::Vector3d& a, 
                                                    const Eigen::Vector3d& b) const {
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ap = p - a;
    double t = ap.dot(ab) / ab.dot(ab);
    t = std::max(0.0, std::min(1.0, t));
    return (p - (a + t * ab)).norm(); // where (a + t * ab) represents the closest point of P on the line 
}

visualization_msgs::msg::MarkerArray Ur10eCollisionModel::getCapsuleMarkers(
    const std::string& frame_id, const std::string& ns) const 
{
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    for (const auto& cap : capsules_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = ns;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position from capsule transform
        marker.pose.position.x = cap.transform.translation().x();
        marker.pose.position.y = cap.transform.translation().y();
        marker.pose.position.z = cap.transform.translation().z();

        // Orientation
        Eigen::Quaterniond q(cap.transform.rotation());
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Scale (Cylinder diameter is x and y, height is z)
        marker.scale.x = cap.radius * 2.0;
        marker.scale.y = cap.radius * 2.0;
        marker.scale.z = cap.length;

        // Color (Green, 40% transparent)
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.4f;

        markers.markers.push_back(marker);
    }
    return markers;
}
