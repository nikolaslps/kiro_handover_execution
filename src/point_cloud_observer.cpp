#include "point_cloud_observer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <cstring>

static const auto LOGGER = rclcpp::get_logger("point_cloud_observer");

PointCloudObserver::PointCloudObserver(rclcpp::Node::SharedPtr node,
                                       const std::string & topic,
                                       const rclcpp::SubscriptionOptions & options)
{
    sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = msg;
        },
        options);
    RCLCPP_INFO(LOGGER, "Subscribed to point cloud: %s", topic.c_str());
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudObserver::latest()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

bool PointCloudObserver::hasCloud()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_ != nullptr;
}

std::vector<Eigen::Vector3d> parsePointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud.width * cloud.height);

    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto & f : cloud.fields) {
        if (f.name == "x") x_off = f.offset;
        if (f.name == "y") y_off = f.offset;
        if (f.name == "z") z_off = f.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) return points;

    const uint8_t * data = cloud.data.data();
    const size_t step = cloud.point_step;
    const size_t n = cloud.width * cloud.height;

    for (size_t i = 0; i < n; ++i) {
        const uint8_t * p = data + i * step;
        float x, y, z;
        std::memcpy(&x, p + x_off, sizeof(float));
        std::memcpy(&y, p + y_off, sizeof(float));
        std::memcpy(&z, p + z_off, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        points.emplace_back(x, y, z);
    }
    return points;
}

bool findNearestPointInCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    const Eigen::Vector3d & query_in_planning_frame,
    const std::string & planning_frame,
    const tf2_ros::Buffer & tf_buffer,
    double search_radius,
    Eigen::Vector3d & nearest_in_planning_frame)
{
    if (!cloud_msg) {
        RCLCPP_WARN(LOGGER, "No point cloud available.");
        return false;
    }

    const std::string cloud_frame = cloud_msg->header.frame_id;
    Eigen::Vector3d query_in_cloud;

    // transform query point to cloud frame from planning frame
    try {
        geometry_msgs::msg::PointStamped q_in, q_out;
        q_in.header.frame_id = planning_frame;
        q_in.header.stamp = cloud_msg->header.stamp;
        q_in.point.x = query_in_planning_frame.x();
        q_in.point.y = query_in_planning_frame.y();
        q_in.point.z = query_in_planning_frame.z();
        tf_buffer.transform(q_in, q_out, cloud_frame);
        query_in_cloud = Eigen::Vector3d(q_out.point.x, q_out.point.y, q_out.point.z);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(LOGGER, "TF transform failed: %s", ex.what());
        return false;
    }

    // nearest point search (brute force)
    auto points = parsePointCloud(*cloud_msg);
    RCLCPP_DEBUG(LOGGER, "Searching %zu cloud points (radius=%.2fm)...", points.size(), search_radius);

    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest_in_cloud;
    bool found = false;

    // O(N) 
    // for (const auto & pt : points) {
    //     double d = (pt - query_in_cloud).norm();
    //     if (d < min_dist && d < search_radius) {
    //         min_dist = d;
    //         nearest_in_cloud = pt;
    //         found = true;
    //     }
    // }

    // Instead i could compare squared distance 
    double min_sq = std::numeric_limits<double>::infinity();
    double r_sq = search_radius * search_radius;

    for (const auto & pt : points) {
        double sq = (pt - query_in_cloud).squaredNorm();
        if (sq < min_sq && sq < r_sq) {
            min_sq = sq;
            nearest_in_cloud = pt;
            found = true;
        }
    }
    min_dist = std::sqrt(min_sq);

    if (!found) {
        RCLCPP_WARN(LOGGER, "No cloud point within %.2fm.", search_radius);
        return false;
    }

    // transform nearest point back to planning frame
    try {
        geometry_msgs::msg::PointStamped n_in, n_out;
        n_in.header.frame_id = cloud_frame;
        n_in.header.stamp = cloud_msg->header.stamp;
        n_in.point.x = nearest_in_cloud.x();
        n_in.point.y = nearest_in_cloud.y();
        n_in.point.z = nearest_in_cloud.z();
        tf_buffer.transform(n_in, n_out, planning_frame);
        nearest_in_planning_frame = Eigen::Vector3d(n_out.point.x, n_out.point.y, n_out.point.z);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(LOGGER, "TF back-transform failed: %s", ex.what());
        return false;
    }

    RCLCPP_INFO(LOGGER, "Nearest obstacle: [%.3f, %.3f, %.3f] dist=%.3f",
        nearest_in_planning_frame.x(),
        nearest_in_planning_frame.y(),
        nearest_in_planning_frame.z(), min_dist);
    return true;
}