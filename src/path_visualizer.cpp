#include "path_visualizer.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

static const auto LOGGER = rclcpp::get_logger("path_visualizer");

PathVisualizer::PathVisualizer(rclcpp::Node::SharedPtr node,
                               const std::string & frame_id): frame_id_(frame_id)
{
    pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/handover/BezierCurves", rclcpp::QoS(10).transient_local());

    RCLCPP_INFO(LOGGER, "PathVisualizer ready. Topic: /handover/BezierCurves  Frame: %s", frame_id_.c_str());
}

std_msgs::msg::ColorRGBA PathVisualizer::rgba(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

visualization_msgs::msg::Marker PathVisualizer::makeMarker(
    int id,
    const std::string & ns,
    int type,
    const std_msgs::msg::ColorRGBA & color,
    double scale_x, 
    double scale_y, 
    double scale_z)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = rclcpp::Clock().now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color = color;
    m.scale.x = scale_x;
    m.scale.y = (scale_y < 0.0) ? scale_x : scale_y;
    m.scale.z = (scale_z < 0.0) ? scale_x : scale_z;
    m.pose.orientation.w = 1.0;
    return m;
}

void PathVisualizer::publishStraightLine(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    double lateral_threshold)
{
    visualization_msgs::msg::MarkerArray ma;

    auto line = makeMarker(0, "straight_line",
        visualization_msgs::msg::Marker::LINE_STRIP,
        rgba(0.6f, 0.6f, 0.6f, 0.8f), 0.005);
    line.points.push_back(start.position);
    line.points.push_back(end.position);
    ma.markers.push_back(line);

    auto start_sphere = makeMarker(1, "endpoints",
        visualization_msgs::msg::Marker::SPHERE,
        rgba(0.0f, 0.4f, 1.0f, 1.0f), 0.04);
    start_sphere.pose.position = start.position;
    ma.markers.push_back(start_sphere);

    auto end_sphere = makeMarker(2, "endpoints",
        visualization_msgs::msg::Marker::SPHERE,
        rgba(1.0f, 0.0f, 1.0f, 1.0f), 0.04);
    end_sphere.pose.position = end.position;
    ma.markers.push_back(end_sphere);

    // Threshold tube around the line
    {
        Eigen::Vector3d P0(start.position.x, start.position.y, start.position.z);
        Eigen::Vector3d P3(end.position.x, end.position.y, end.position.z);
        Eigen::Vector3d axis = (P3 - P0).normalized();
        double length = (P3 - P0).norm();

        // Rotate world +Z to `axis`:
        Eigen::Vector3d z_hat(0.0, 0.0, 1.0);
        Eigen::Vector3d rot_axis = z_hat.cross(axis);
        double rot_angle = std::acos(std::clamp(z_hat.dot(axis), -1.0, 1.0));

        visualization_msgs::msg::Marker tube = makeMarker(3, "threshold_tube",
            visualization_msgs::msg::Marker::CYLINDER,
            rgba(1.0f, 0.8f, 0.0f, 0.08f), // yellow, very transparent
            lateral_threshold * 2.0,       // diameter
            lateral_threshold * 2.0,
            length);

        // Centre of the cylinder
        tube.pose.position.x = (start.position.x + end.position.x) * 0.5;
        tube.pose.position.y = (start.position.y + end.position.y) * 0.5;
        tube.pose.position.z = (start.position.z + end.position.z) * 0.5;

        // align cylinder Z with line direction
        if (rot_axis.norm() > 1e-6) {
            rot_axis.normalize();
            double half = rot_angle * 0.5;
            tube.pose.orientation.x = rot_axis.x() * std::sin(half);
            tube.pose.orientation.y = rot_axis.y() * std::sin(half);
            tube.pose.orientation.z = rot_axis.z() * std::sin(half);
            tube.pose.orientation.w = std::cos(half);
        } else {
            tube.pose.orientation.w = 1.0;
        }
        ma.markers.push_back(tube);
    }

    auto label_start = makeMarker(4, "labels",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        rgba(1.0f, 1.0f, 1.0f, 1.0f), 0.04);
    label_start.pose.position = start.position;
    label_start.pose.position.z += 0.06;
    label_start.text = "START";
    ma.markers.push_back(label_start);

    auto label_end = makeMarker(5, "labels",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        rgba(1.0f, 1.0f, 1.0f, 1.0f), 0.04);
    label_end.pose.position = end.position;
    label_end.pose.position.z += 0.06;
    label_end.text = "TARGET";
    ma.markers.push_back(label_end);

    pub_->publish(ma);
    RCLCPP_DEBUG(LOGGER, "Published straight-line preview (%zu markers).", ma.markers.size());
}

void PathVisualizer::publishBezierPath(
    const geometry_msgs::msg::Pose  & start,
    const geometry_msgs::msg::Pose  & end,
    const geometry_msgs::msg::Point & P1,
    const geometry_msgs::msg::Point & P2,
    bool is_avoidance,
    int  n_samples)
{
    visualization_msgs::msg::MarkerArray ma;

    auto curve_color = is_avoidance
        ? rgba(1.0f, 0.2f, 0.2f, 1.0f)   // red  = avoidance arc
        : rgba(0.2f, 1.0f, 0.2f, 1.0f);  // green = straight / clear

    auto curve = makeMarker(10, "bezier_curve",
        visualization_msgs::msg::Marker::LINE_STRIP,
        curve_color, 0.008);

    for (int i = 0; i <= n_samples; ++i) {
        double t = static_cast<double>(i) / n_samples;
        double u = 1.0 - t;
        double u2 = u*u, u3 = u2*u;
        double t2 = t*t, t3 = t2*t;

        geometry_msgs::msg::Point pt;
        pt.x = u3*start.position.x + 3*u2*t*P1.x + 3*u*t2*P2.x + t3*end.position.x;
        pt.y = u3*start.position.y + 3*u2*t*P1.y + 3*u*t2*P2.y + t3*end.position.y;
        pt.z = u3*start.position.z + 3*u2*t*P1.z + 3*u*t2*P2.z + t3*end.position.z;
        curve.points.push_back(pt);
    }
    ma.markers.push_back(curve);

    auto polygon = makeMarker(11, "control_polygon",
        visualization_msgs::msg::Marker::LINE_STRIP,
        rgba(1.0f, 1.0f, 1.0f, 0.4f), 0.003);
    polygon.points.push_back(start.position);
    polygon.points.push_back(P1);
    polygon.points.push_back(P2);
    polygon.points.push_back(end.position);
    ma.markers.push_back(polygon);

    auto cp_spheres = makeMarker(12, "control_points",
        visualization_msgs::msg::Marker::SPHERE_LIST,
        rgba(1.0f, 1.0f, 0.0f, 1.0f), 0.035);
    cp_spheres.points.push_back(P1);
    cp_spheres.points.push_back(P2);
    ma.markers.push_back(cp_spheres);

    auto label_p1 = makeMarker(13, "cp_labels",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        rgba(1.0f, 1.0f, 0.0f, 1.0f), 0.035);
    label_p1.pose.position = P1;
    label_p1.pose.position.z += 0.05;
    label_p1.text = "P1";
    ma.markers.push_back(label_p1);

    auto label_p2 = makeMarker(14, "cp_labels",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        rgba(1.0f, 1.0f, 0.0f, 1.0f), 0.035);
    label_p2.pose.position = P2;
    label_p2.pose.position.z += 0.05;
    label_p2.text = "P2";
    ma.markers.push_back(label_p2);

    auto label_arc = makeMarker(15, "arc_label",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        curve_color, 0.04);

    {
        constexpr double t = 0.5, u = 0.5;
        label_arc.pose.position.x =
            u*u*u*start.position.x + 3*u*u*t*P1.x + 3*u*t*t*P2.x + t*t*t*end.position.x;
        label_arc.pose.position.y =
            u*u*u*start.position.y + 3*u*u*t*P1.y + 3*u*t*t*P2.y + t*t*t*end.position.y;
        label_arc.pose.position.z =
            u*u*u*start.position.z + 3*u*u*t*P1.z + 3*u*t*t*P2.z + t*t*t*end.position.z;
        label_arc.pose.position.z += 0.07;
    }
    label_arc.text = is_avoidance ? "AVOIDANCE ARC" : "STRAIGHT PATH";
    ma.markers.push_back(label_arc);

    pub_->publish(ma);
    RCLCPP_DEBUG(LOGGER, "Published Bézier path (%s, %zu markers).",
        is_avoidance ? "avoidance" : "straight", ma.markers.size());
}

void PathVisualizer::publishObstacles(const std::vector<ObstacleInfo> & obstacles)
{
    if (obstacles.empty()) return;

    visualization_msgs::msg::MarkerArray ma;

    auto obs_left = makeMarker(20, "obstacles",
        visualization_msgs::msg::Marker::SPHERE_LIST,
        rgba(1.0f, 0.45f, 0.0f, 0.85f), 0.025);   // left: orange

    auto obs_right = makeMarker(21, "obstacles",
        visualization_msgs::msg::Marker::SPHERE_LIST,
        rgba(1.0f, 0.8f, 0.0f, 0.85f), 0.025);    // right: yellow-orange

    auto obs_below = makeMarker(22, "obstacles",
        visualization_msgs::msg::Marker::SPHERE_LIST,
        rgba(0.8f, 0.2f, 1.0f, 0.85f), 0.025);    // below: purple

    auto obs_above = makeMarker(23, "obstacles",
        visualization_msgs::msg::Marker::SPHERE_LIST,
        rgba(0.2f, 0.8f, 1.0f, 0.85f), 0.025);    // above: cyan

    for (const auto & obs : obstacles) {
        geometry_msgs::msg::Point p;
        p.x = obs.position.x();
        p.y = obs.position.y();
        p.z = obs.position.z();

        // Route to the right marker list based on which side the obstacle is on
        bool is_left = obs.lateral_dist > 0.01;
        bool is_right = obs.lateral_dist < -0.01;
        bool is_below = obs.vertical_dist < -0.01;
        bool is_above = obs.vertical_dist > 0.01;

        if (is_below) obs_below.points.push_back(p);
        else if (is_above) obs_above.points.push_back(p);
        else if (is_left) obs_left.points.push_back(p);
        else obs_right.points.push_back(p);
    }

    if (!obs_left.points.empty()) ma.markers.push_back(obs_left);
    if (!obs_right.points.empty()) ma.markers.push_back(obs_right);
    if (!obs_below.points.empty()) ma.markers.push_back(obs_below);
    if (!obs_above.points.empty()) ma.markers.push_back(obs_above);

    if (!obstacles.empty()) {
        // Place the label at the centroid of all obstacle points
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto & o : obstacles) centroid += o.position;
        centroid /= static_cast<double>(obstacles.size());

        auto obs_label = makeMarker(24, "obstacle_label",
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
            rgba(1.0f, 0.5f, 0.0f, 1.0f), 0.04);
        obs_label.pose.position.x = centroid.x();
        obs_label.pose.position.y = centroid.y();
        obs_label.pose.position.z = centroid.z() + 0.08;
        obs_label.text = std::to_string(obstacles.size()) + " obstacle pts";
        ma.markers.push_back(obs_label);
    }

    pub_->publish(ma);
    RCLCPP_DEBUG(LOGGER, "Published %zu obstacle markers.", obstacles.size());
}

void PathVisualizer::clear()
{
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.header.frame_id = frame_id_;
    del.header.stamp = rclcpp::Clock().now();
    ma.markers.push_back(del);
    pub_->publish(ma);
}