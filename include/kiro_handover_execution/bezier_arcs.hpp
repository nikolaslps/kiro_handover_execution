#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>

struct ObstacleInfo
{
    Eigen::Vector3d position;

    double t_along_line;    // 0 = at start, 1 = at target
    double lateral_dist;    // + = LEFT, - = RIGHT
    double vertical_dist;   // + = ABOVE the line, - = BELOW
    double dist_from_line;  // total Euclidean distance from the line (unsigned)
};

// Generate N waypoints along a cubic Bézier curve.
// Orientation is kept fixed at start.orientation throughout.
std::vector<geometry_msgs::msg::Pose> generateCubicBezier(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const geometry_msgs::msg::Point & P1,
    const geometry_msgs::msg::Point & P2,
    int n = 80);

// P1 and P2 at 1/3 and 2/3 along the straight line (no curvature).
void controlPoints_StraightLine(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    geometry_msgs::msg::Point & P1_out,
    geometry_msgs::msg::Point & P2_out);


// Scan the point cloud for points that are within lateral_threshold metres
// of the straight line connecting start->end. Returns one ObstacleInfo per 
// cloud point that is "on the path".
std::vector<ObstacleInfo> findObstaclesAlongLine(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const std::string & planning_frame,
    const tf2_ros::Buffer & tf_buffer,
    double lateral_threshold = 0.20);

// Given a list of obstacles along the path, compute Bezier control points
// P1 and P2 that steer the curve away from obstacles.
//      - Obstacle to the LEFT : curve goes RIGHT (negative lateral offset)
//      - Obstacle to the RIGHT : curve goes LEFT (positive lateral offset)
//      - Obstacle BELOW line : curve also goes UP (positive vertical offset)
//      - Obstacle ABOVE line : curve also goes DOWN (negative vertical offset)
//      - Obstacle closer to start (t < 0.5) : adjust P1 more than P2
//      - Obstacle closer to target (t >= 0.5) : adjust P2 more than P1
//      - Obstacles on BOTH lateral sides : go to the side with more clearance 
void computeAvoidanceControlPoints(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    const std::vector<ObstacleInfo> & obstacles,
    double clearance, // minimum distance of the curve from obstacles
    geometry_msgs::msg::Point & P1_out,
    geometry_msgs::msg::Point & P2_out);

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v);