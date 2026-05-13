#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>

class PointCloudObserver
{
public:
    explicit PointCloudObserver(rclcpp::Node::SharedPtr node, 
                                const std::string & topic,
                                const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions());

    sensor_msgs::msg::PointCloud2::SharedPtr latest();
    bool hasCloud();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_;
    std::mutex mutex_; // to avoid read and writes colliding
};

// Decode raw PointCloud2 bytes vector of Eigen points
std::vector<Eigen::Vector3d> parsePointCloud(const sensor_msgs::msg::PointCloud2 & cloud);

// Find the lidar point nearest to a query position
// query is in planning_frame; result is returned in planning_frame
// Returns false if no point found within search_radius
bool findNearestPointInCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    const Eigen::Vector3d & query_in_planning_frame,
    const std::string & planning_frame,
    const tf2_ros::Buffer & tf_buffer,
    double search_radius,
    Eigen::Vector3d & nearest_in_planning_frame);