#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "bezier_arcs.hpp" // for ObstacleInfo

class PathVisualizer
{
public:
    explicit PathVisualizer(rclcpp::Node::SharedPtr node,
                            const std::string & frame_id = "world");

    void publishStraightLine(
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & end,
        double lateral_threshold);

    void publishBezierPath(
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & end,
        const geometry_msgs::msg::Point & P1,
        const geometry_msgs::msg::Point & P2,
        bool is_avoidance, // true = red arc, false = green arc
        int n_samples = 100);

    void publishObstacles(const std::vector<ObstacleInfo> & obstacles);

    void clear();

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    std::string frame_id_;
    int marker_id_ = 0;

    visualization_msgs::msg::Marker makeMarker(
        int id,
        const std::string & ns,
        int type,
        const std_msgs::msg::ColorRGBA & color,
        double scale_x, 
        double scale_y = -1.0, 
        double scale_z = -1.0);

    std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a = 1.0f);
};