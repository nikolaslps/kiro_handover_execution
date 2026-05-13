#include "bezier_arcs.hpp"
#include "point_cloud_observer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

static const auto LOGGER = rclcpp::get_logger("bezier_arcs");

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
}

std::vector<geometry_msgs::msg::Pose> generateCubicBezier(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const geometry_msgs::msg::Point & P1,
    const geometry_msgs::msg::Point & P2,
    int n)
{
    std::vector<geometry_msgs::msg::Pose> wps;
    wps.reserve(n);

    for (int i = 1; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        double u = 1.0 - t;
        double u2 = u * u, u3 = u2 * u;
        double t2 = t * t, t3 = t2 * t;

        geometry_msgs::msg::Pose wp;
        wp.position.x = u3*start.position.x + 3*u2*t*P1.x + 3*u*t2*P2.x + t3*end.position.x;
        wp.position.y = u3*start.position.y + 3*u2*t*P1.y + 3*u*t2*P2.y + t3*end.position.y;
        wp.position.z = u3*start.position.z + 3*u2*t*P1.z + 3*u*t2*P2.z + t3*end.position.z;
        wp.orientation = start.orientation; // orientation fixed
        wps.push_back(wp);
    }
    return wps;
}

void controlPoints_StraightLine( // Bezier curve degenerates to straight line
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    geometry_msgs::msg::Point & P1_out,
    geometry_msgs::msg::Point & P2_out)
{
    Eigen::Vector3d P0(start.position.x, start.position.y, start.position.z);
    Eigen::Vector3d P3(end.position.x, end.position.y, end.position.z);
    P1_out = toPoint(P0 + (P3 - P0) / 3.0);
    P2_out = toPoint(P0 + 2.0 * (P3 - P0) / 3.0);
}

std::vector<ObstacleInfo> findObstaclesAlongLine(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const std::string & planning_frame,
    const tf2_ros::Buffer & tf_buffer,
    double lateral_threshold)
{
    std::vector<ObstacleInfo> obstacles;

    if (!cloud_msg) {
        RCLCPP_WARN(LOGGER, "No point cloud available.");
        return obstacles;
    }

    // TF: cloud frame to planning frame
    const std::string cloud_frame = cloud_msg->header.frame_id;
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer.lookupTransform(planning_frame, cloud_frame, cloud_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(LOGGER, "TF lookup failed: %s", ex.what());
        return obstacles;
    }

    // Line geometry
    Eigen::Vector3d P0(start.position.x, start.position.y, start.position.z);
    Eigen::Vector3d P3(end.position.x, end.position.y, end.position.z);
    Eigen::Vector3d line_vec = P3 - P0;
    double line_length = line_vec.norm();
    if (line_length < 1e-4) return obstacles;

    // Create local coordinate frame for the line:
    //   forward = along the line (from start towards the target)
    //   lateral = perpendicular to forward in the horizontal plane (LEFT from line is +)
    //   up_perp = vertical (ABOVE line is +)
    Eigen::Vector3d forward = line_vec / line_length;
    Eigen::Vector3d world_up(0.0, 0.0, 1.0); // +Z is up
    Eigen::Vector3d lateral = forward.cross(world_up).normalized();
    Eigen::Vector3d up_perp = lateral.cross(forward).normalized();

    // Scan all cloud points
    auto raw_points = parsePointCloud(*cloud_msg);

    for (const auto & raw_pt : raw_points) {
        // transform to planning frame
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header.frame_id = cloud_frame;
        p_in.header.stamp = cloud_msg->header.stamp;
        p_in.point.x = raw_pt.x();
        p_in.point.y = raw_pt.y();
        p_in.point.z = raw_pt.z();
        tf2::doTransform(p_in, p_out, tf_stamped);

        Eigen::Vector3d pt(p_out.point.x, p_out.point.y, p_out.point.z);

        // project onto line axis to find where along the line this point is closest to
        double t = (pt - P0).dot(forward) / line_length;

        // only consider points between start and end
        if (t < -0.05 || t > 1.05) continue;

        // vector from the closest point on the line to this point
        Eigen::Vector3d on_line = P0 + t * line_length * forward;
        Eigen::Vector3d offset = pt - on_line;

        double dist_from_line = offset.norm();
        if (dist_from_line > lateral_threshold) continue;

        // signed lateral distance from the line: + = LEFT, - = RIGHT
        double lat = offset.dot(lateral);

        // signed vertical distance from the line: + = ABOVE, - = BELOW
        double vert = offset.dot(up_perp);

        ObstacleInfo obs;
        obs.position = pt;
        obs.t_along_line = t;
        obs.lateral_dist = lat;
        obs.vertical_dist = vert;
        obs.dist_from_line = dist_from_line;
        obstacles.push_back(obs);
    }

    RCLCPP_INFO(LOGGER, "Found %zu obstacle points near the line (threshold=%.2fm).", obstacles.size(), lateral_threshold);

    return obstacles;
}

void computeAvoidanceControlPoints(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const std::vector<ObstacleInfo> & obstacles,
    double clearance,
    geometry_msgs::msg::Point & P1_out,
    geometry_msgs::msg::Point & P2_out)
{
    Eigen::Vector3d P0(start.position.x, start.position.y, start.position.z);
    Eigen::Vector3d P3(end.position.x, end.position.y, end.position.z);
    double line_length = (P3 - P0).norm();
    Eigen::Vector3d forward = (P3 - P0).normalized();
    Eigen::Vector3d world_up(0.0, 0.0, 1.0);
    Eigen::Vector3d lateral = forward.cross(world_up).normalized();
    Eigen::Vector3d up_perp = lateral.cross(forward).normalized();

    // Start with straight-line control points
    Eigen::Vector3d P1 = P0 + (P3 - P0) / 3.0;
    Eigen::Vector3d P2 = P0 + 2.0 * (P3 - P0) / 3.0;

    if (obstacles.empty()) {
        P1_out = toPoint(P1);
        P2_out = toPoint(P2);
        return;
    }

    // P1 and P2 are treated independently — obstacles at small t affect P1 more
    double P1_lat = 0.0; // (+ = LEFT)
    double P2_lat = 0.0;
    double P1_vert = 0.0; // (+ = UP)
    double P2_vert = 0.0;

    double max_obs_left = 0.0; // largest obstacle on LEFT side
    double max_obs_right = 0.0; // largest obstacle on RIGHT side

    for (const auto & obs : obstacles)
    {
        // Lateral push
        //
        //   obs.lateral_dist > 0: obstacle is to the LEFT so push curve to the RIGHT (negative)
        //   obs.lateral_dist < 0: obstacle is to the RIGHT so push curve to the LEFT (positive)
        //
        // Required push magnitude = |lateral_dist| + clearance
        // Sign: opposite to obstacle side

        double lat_sign = (obs.lateral_dist >= 0.0) ? -1.0 : +1.0;
        double lat_magnitude = std::abs(obs.lateral_dist) + clearance;
        double lat_push = lat_sign * lat_magnitude;

        if (obs.lateral_dist > 0.0)
            max_obs_left = std::max(max_obs_left, obs.lateral_dist);
        else
            max_obs_right = std::max(max_obs_right, -obs.lateral_dist);

        // Vertical push
        //
        //   obs.vertical_dist < 0: obstacle BELOW so push curve UP
        //   obs.vertical_dist > 0: obstacle ABOVE so push curve DOWN (optional)
        //
        // Push magnitude proportional to how far below the line the obstacle is.

        double vert_push = 0.0;
        if (obs.vertical_dist < 0.0) {
            vert_push = std::abs(obs.vertical_dist) + clearance;
        } else if (obs.vertical_dist > 0.0) {
            vert_push = -(obs.vertical_dist + clearance);
        }

        // Weight by proximity along line
        // t_along_line in [0, 1]:
        //   t < 0.5: obstacle is in the first half so adjust P1 more
        //   t >= 0.5: obstacle is in the second half so adjust P2 more
        double t = std::clamp(obs.t_along_line, 0.0, 1.0);
        double w1 = 1.0 - t; // weight for P1: high near start, low near end
        double w2 = t; // weight for P2: low near start, high near end

        double p1_lat_candidate = lat_push * (1.0 + w1);
        double p2_lat_candidate = lat_push * (1.0 + w2);

        if (std::abs(p1_lat_candidate) > std::abs(P1_lat)) P1_lat = p1_lat_candidate;
        if (std::abs(p2_lat_candidate) > std::abs(P2_lat)) P2_lat = p2_lat_candidate;

        double p1_vert_candidate = vert_push * (1.0 + w1);
        double p2_vert_candidate = vert_push * (1.0 + w2);

        if (std::abs(p1_vert_candidate) > std::abs(P1_vert)) P1_vert = p1_vert_candidate;
        if (std::abs(p2_vert_candidate) > std::abs(P2_vert)) P2_vert = p2_vert_candidate;
    }

    // If both sides are blocked, route to the side with less obstruction
    if (max_obs_left > 0.0 && max_obs_right > 0.0) {
        double sign = (max_obs_left < max_obs_right) ? +1.0 : -1.0;
        double total_push = std::max(max_obs_left, max_obs_right) + clearance;

        P1_lat = sign * total_push * 2.0;
        P2_lat = sign * total_push * 2.0;

        RCLCPP_WARN(LOGGER,
            "Obstacles on BOTH lateral sides (left=%.2fm, right=%.2fm). "
            "Routing to %s with push=%.2fm.",
            max_obs_left, max_obs_right,
            sign > 0 ? "LEFT" : "RIGHT", total_push);
    }

    // Apply offsets to control points
    P1 += lateral * P1_lat + up_perp * P1_vert;
    P2 += lateral * P2_lat + up_perp * P2_vert;

    RCLCPP_INFO(LOGGER, "Control point offsets:");
    RCLCPP_INFO(LOGGER, "  P1: lateral=%.3fm  vertical=%.3fm", P1_lat, P1_vert);
    RCLCPP_INFO(LOGGER, "  P2: lateral=%.3fm  vertical=%.3fm", P2_lat, P2_vert);
    RCLCPP_INFO(LOGGER, "  P1 final: [%.3f, %.3f, %.3f]", P1.x(), P1.y(), P1.z());
    RCLCPP_INFO(LOGGER, "  P2 final: [%.3f, %.3f, %.3f]", P2.x(), P2.y(), P2.z());

    P1_out = toPoint(P1);
    P2_out = toPoint(P2);
}