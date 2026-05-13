#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

#include "point_cloud_observer.hpp"
#include "bezier_arcs.hpp"
#include "arc_planner.hpp"
#include "path_visualizer.hpp"
#include "workspace_checker.hpp"
#include "feasibility.hpp"
#include "Ur10eCollisionModel.hpp"

// custom service definitions
#include "kiro_handover_interfaces/srv/cluster_handover_volume.hpp"
#include "kiro_handover_interfaces/srv/get_active_body_id.hpp"
#include "kiro_handover_interfaces/srv/activate_handover.hpp"

using namespace std::chrono_literals;
static const auto LOGGER = rclcpp::get_logger("handover_execution");

static constexpr double CLEARANCE = 0.30; // desired distance from obstacles
static constexpr double LATERAL_THRESHOLD = 0.20; // tube radius around line to check for obstacles
static constexpr int N_WAYPOINTS = 80;

static bool handover_requested = false; // adjusted based on the ActivateHandover service

static constexpr bool DEBUG = true; // for debugging purposes

static void clearOctomap(const rclcpp::Node::SharedPtr& node)
{
    auto client = node->create_client<std_srvs::srv::Empty>("/clear_octomap");
    if (!client->wait_for_service(3s)) {
        RCLCPP_WARN(LOGGER, "/clear_octomap not available — skipping.");
        return;
    }
    client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    RCLCPP_INFO(LOGGER, "Octomap clear requested.");
    rclcpp::sleep_for(100ms);
}

// handle the external trigger for handover pipeline start/stop
void handle_handover_trigger(
    const std::shared_ptr<kiro_handover_interfaces::srv::ActivateHandover::Request> request,
    std::shared_ptr<kiro_handover_interfaces::srv::ActivateHandover::Response> response) 
{
    handover_requested = request->handover_phase;
    response->success = true;
    response->message = handover_requested ? "Handover sequence queued." : "Handover stop requested.";
}

static std::string getActiveBodyId(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Client<kiro_handover_interfaces::srv::GetActiveBodyID>::SharedPtr& client)
{
    RCLCPP_INFO(LOGGER, "Requesting active body ID from /get_active_body_id ...");

    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(LOGGER, "Service /get_active_body_id not available!");
        return "";
    }

    auto request = std::make_shared<kiro_handover_interfaces::srv::GetActiveBodyID::Request>();

    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(LOGGER, "/get_active_body_id timed out after 5s");
        return "";
    }

    auto response = future.get();
    if (!response) {
        RCLCPP_ERROR(LOGGER, "/get_active_body_id returned null response");
        return "";
    }

    if (response->body_id.empty()) {
        RCLCPP_WARN(LOGGER, "No active body detected");
        return "";
    }

    RCLCPP_INFO(LOGGER, "Active body ID received: %s", response->body_id.c_str());
    return response->body_id;
}

static bool clusterHandoverVolume(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Client<kiro_handover_interfaces::srv::ClusterHandoverVolume>::SharedPtr& client,
    const std::string& body_id,
    int k,
    geometry_msgs::msg::PoseArray& out_candidates)
{
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(LOGGER, "Service /cluster_handover_volume not available!");
        return false;
    }

    auto request = std::make_shared<kiro_handover_interfaces::srv::ClusterHandoverVolume::Request>();
    request->body_id = body_id;
    request->k = k;

    auto future = client->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready) {
        RCLCPP_WARN(LOGGER, "/cluster_handover_volume timed out for k=%d", k);
        return false;
    }

    auto response = future.get();
    if (!response) {
        RCLCPP_WARN(LOGGER, "/cluster_handover_volume returned null response (k=%d)", k);
        return false;
    }

    if (!response->message.empty()) {
        RCLCPP_INFO(LOGGER, "Clustering message (k=%d): %s", k, response->message.c_str());
    }

    if (!response->success) {
        RCLCPP_WARN(LOGGER, "Clustering failed for k=%d: %s", k, response->message.c_str());
        return false;
    }

    if (response->clustered_points.poses.empty()) {
        RCLCPP_WARN(LOGGER, "No clustered points returned for k=%d", k);
        return false;
    }

    out_candidates = response->clustered_points;
    return true;
}

static bool planAndExecuteToTarget(
    const rclcpp::Node::SharedPtr& node,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& planning_frame,
    tf2_ros::Buffer& tf_buffer,
    PointCloudObserver& cloud_observer,
    PathVisualizer& visualizer,
    WorkspaceChecker& ws_checker,
    Ur10eCollisionModel& robot_capsule_model,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr capsule_pub,
    const geometry_msgs::msg::PoseStamped& start_pose_stamped,
    const geometry_msgs::msg::Pose& target_pose,
    bool use_collision_capsules = true)
{
    clearOctomap(node);

    if (DEBUG) {
        visualizer.clear();
        visualizer.publishStraightLine(start_pose_stamped.pose, target_pose, LATERAL_THRESHOLD);
    }

    RCLCPP_INFO(LOGGER, "Checking for obstacles along straight-line path...");
    auto cloud = cloud_observer.latest();
    auto obstacles = findObstaclesAlongLine(cloud, start_pose_stamped.pose, target_pose, planning_frame, tf_buffer, LATERAL_THRESHOLD);

    if (DEBUG) {
        visualizer.publishObstacles(obstacles);
    }

    geometry_msgs::msg::Point P1, P2;
    moveit_msgs::msg::RobotTrajectory traj;

    if (obstacles.empty())
    {
        RCLCPP_INFO(LOGGER, "Line is clear. Planning straight Cartesian path...");

        controlPoints_StraightLine(start_pose_stamped.pose, target_pose, P1, P2);
        
        auto wps = generateCubicBezier(start_pose_stamped.pose, target_pose, P1, P2, 1); // only need 1 waypoint (the target) since it's a straight line

        if (DEBUG) {
            visualizer.publishBezierPath(start_pose_stamped.pose, target_pose, P1, P2, /*is_avoidance=*/false);
        }

        double fraction = planArc(move_group, wps, traj, "Straight");

        if (fraction >= 0.99) {
            RCLCPP_INFO(LOGGER, "Straight path planned. Executing...");
            bool executed = executeTrajectory(move_group, traj, "Straight");
            if (executed) {
                return true;
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Straight path planned (fraction=%.1f%%) but execution FAILED. "
                    "This is usually controller abort/timeout, limits, or safety stop (not IK).", fraction * 100.0);
                return false;
            }
        }

        RCLCPP_ERROR(LOGGER, "Straight Cartesian path only %.1f%% complete — possible MoveIt collision or IK failure.", fraction * 100.0);
        return false;
    }

    // obstacles detected
    std::vector<double> clearances = { CLEARANCE, CLEARANCE * 2.0 };
    std::vector<std::string> labels = { "Avoidance arc 1x", "Avoidance arc 2x" };

    for (size_t i = 0; i < clearances.size(); ++i) 
    {
        RCLCPP_INFO(LOGGER, "Attempting %s with clearance %.2fm", labels[i].c_str(), clearances[i]);
        
        computeAvoidanceControlPoints(start_pose_stamped.pose, target_pose, obstacles, clearances[i], P1, P2);
        ws_checker.adjustControlPoints(start_pose_stamped.pose, target_pose, P1, P2);

        if (DEBUG) {
            visualizer.publishBezierPath(start_pose_stamped.pose, target_pose, P1, P2, true);
        }

        auto wps = generateCubicBezier(start_pose_stamped.pose, target_pose, P1, P2, N_WAYPOINTS);

        // --- beginning of collision capsule logic ---
        if (use_collision_capsules) {
            bool body_safe = true;
            auto current_state = move_group.getCurrentState(); 

            std::vector<geometry_msgs::msg::Point> obs_points;
            for(const auto& obs_info : obstacles) {
                geometry_msgs::msg::Point p;
                p.x = obs_info.position.x();
                p.y = obs_info.position.y();
                p.z = obs_info.position.z();
                obs_points.push_back(p);
            }
            
            // We check a subset of waypoints to save computation
            for (size_t j = 0; j < wps.size(); j += 5) {
                moveit::core::RobotState temp_state(*current_state);
                
                // solve IK for the specific waypoint pose
                if (temp_state.setFromIK(move_group.getRobotModel()->getJointModelGroup(move_group.getName()), wps[j])) {
                    temp_state.update();
                    robot_capsule_model.updatePoses(temp_state);

                    if (DEBUG) {
                        auto markers = robot_capsule_model.getCapsuleMarkers(planning_frame);
                        capsule_pub->publish(markers);
                        rclcpp::sleep_for(std::chrono::milliseconds(10));
                    }

                    if (robot_capsule_model.isColliding(obs_points, 0.05)) {
                        body_safe = false;
                        break;
                    }
                }
            }

            if (!body_safe) {
                RCLCPP_WARN(LOGGER, "%s: End-effector path is clear, but Robot Body (elbow/arm) will collide. Trying next clearance...", labels[i].c_str());
                continue; 
            }
        } else {
            RCLCPP_INFO(LOGGER, "Skipping UR10e Robot Body capsule collision check.");
        }
        // --- end of collision capsule logic ---

        double fraction = planArc(move_group, wps, traj, labels[i]);
        if (fraction >= 0.99) {
            RCLCPP_INFO(LOGGER, "%s planned successfully. Executing...", labels[i].c_str());
            if (executeTrajectory(move_group, traj, labels[i])) {
                return true;
            }
        } else {
            RCLCPP_WARN(LOGGER, "%s failed planning (fraction=%.1f%%).", labels[i].c_str(), fraction * 100.0);
        }
    }

    RCLCPP_ERROR(LOGGER, "All path attempts failed. Obstacles too close or target unreachable for robot body.");
    return false;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("handover_execution", options);
    // node->set_parameter(rclcpp::Parameter("use_sim_time", false));

    if (!node->has_parameter("use_sim_time")) {
        node->declare_parameter<bool>("use_sim_time", false);
    }
    if (!node->has_parameter("move_group_name")) {
        node->declare_parameter<std::string>("move_group_name", "ur_manipulator");
    }
    if (!node->has_parameter("use_collision_capsules")) {
        node->declare_parameter<bool>("use_collision_capsules", true);
    }
    std::string move_group_name = node->get_parameter("move_group_name").as_string();
    bool use_collision_capsules = node->get_parameter("use_collision_capsules").as_bool();

    RCLCPP_INFO(LOGGER, "Params: Capsules: %s, Group: %s. Sim time: %s", 
                use_collision_capsules ? "ON" : "OFF", 
                move_group_name.c_str(),
                node->get_parameter("use_sim_time").as_bool() ? "ON" : "OFF");

    // Publisher for collision capsules for visualization
    auto capsule_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("robot_body_capsules", 10);

    // Reentrant to let external handover service trigger occur without deadlock
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group;
    PointCloudObserver cloud_observer(node, "/filtered_cloud", sub_options);

    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    psm->startSceneMonitor("/monitored_planning_scene");

    moveit::planning_interface::MoveGroupInterface move_group(node, move_group_name);
    move_group.startStateMonitor(5.0);
    const std::string planning_frame = move_group.getPlanningFrame(); // "kiro_base_link"

    PathVisualizer visualizer(node, planning_frame); // if you want to visualize the curves

    Eigen::Vector3d base_origin(0.0, 0.0, 0.0);
    try {
        auto tf_base = tf_buffer->lookupTransform(
            planning_frame, 
            "base_link", 
            tf2::TimePointZero,
            std::chrono::seconds(5));
        base_origin = {tf_base.transform.translation.x, tf_base.transform.translation.y, tf_base.transform.translation.z};  // position of the manipulator's base
        RCLCPP_INFO(LOGGER, "UR base in planning frame: [%.3f, %.3f, %.3f]", base_origin.x(), base_origin.y(), base_origin.z());
    } catch (...) { 
        RCLCPP_WARN(LOGGER, "Could not lookup base_link TF, using world origin."); 
    }
    WorkspaceChecker ws_checker(base_origin); // strict workspace (only forward plane from the robot)

    Ur10eCollisionModel robot_capsule_model; // collision capsule of the manipulator

    // clients communicating with the handover calculation container
    auto body_id_client = node->create_client<kiro_handover_interfaces::srv::GetActiveBodyID>("/get_active_body_id", rmw_qos_profile_services_default, callback_group);
    auto clustering_client = node->create_client<kiro_handover_interfaces::srv::ClusterHandoverVolume>("/cluster_handover_volume", rmw_qos_profile_services_default, callback_group);
    auto calc_activation_client = node->create_client<kiro_handover_interfaces::srv::ActivateHandover>("/activate_handover_calc", rmw_qos_profile_services_default, callback_group);

    // external trigger to start the handover pipeline (exec node -> calc node -> ...)
    auto trigger_service = node->create_service<kiro_handover_interfaces::srv::ActivateHandover>(
        "activate_handover",
        [&](const std::shared_ptr<kiro_handover_interfaces::srv::ActivateHandover::Request> request,
            std::shared_ptr<kiro_handover_interfaces::srv::ActivateHandover::Response> response) 
        {
            if (!request->handover_phase) {
                // stop requested
                auto stop_req = std::make_shared<kiro_handover_interfaces::srv::ActivateHandover::Request>();
                stop_req->handover_phase = false;
                calc_activation_client->async_send_request(stop_req);
                response->success = true;
                response->message = "Forced stop signal sent to calculation node.";
                return;
            }

            RCLCPP_INFO(LOGGER, ">>> External Trigger: Starting Handover Pipeline <<<");

            // >>> signal the calculation node to start <<<
            auto wake_req = std::make_shared<kiro_handover_interfaces::srv::ActivateHandover::Request>();
            wake_req->handover_phase = true;
            
            if (!calc_activation_client->wait_for_service(2s)) {
                response->success = false;
                response->message = "Calculation container service not available!";
                return;
            }

            auto wake_future = calc_activation_client->async_send_request(wake_req);
            if (wake_future.wait_for(3s) != std::future_status::ready || !wake_future.get()->success) {
                response->success = false;
                response->message = "Calculation node failed to initialize.";
                return;
            }

            // short sleep to let HRI node process the first incoming images
            rclcpp::sleep_for(2s); // necessary

            // >>> main Handover Execution Logic <<<
            bool final_success = false; // successful handover flag

            const std::string body_id = getActiveBodyId(node, body_id_client);
            if (!body_id.empty()) {
                const std::vector<int> k_values = {1, 2, 4, 8, 16};

                for (int k : k_values) {
                    RCLCPP_INFO(LOGGER, "--- Attempting handover with k=%d for body: %s ---", k, body_id.c_str());
                    
                    geometry_msgs::msg::PoseArray candidates;
                    if (!clusterHandoverVolume(node, clustering_client, body_id, k, candidates)) {
                        RCLCPP_WARN(LOGGER, "No candidates returned for k=%d. Trying next k...", k);
                        continue;
                    }

                    RCLCPP_INFO(LOGGER, "Received %zu candidate points for k=%d", candidates.poses.size(), k);
                    
                    int pose_num = 0;
                    for (const auto& pose : candidates.poses) {
                        pose_num++;

                        auto current = move_group.getCurrentPose();
                        geometry_msgs::msg::Pose target;
                        target.position = pose.position;
                        target.orientation = current.pose.orientation; // Keep current EE orientation

                        RCLCPP_INFO(LOGGER, "Trying target %d/%zu (k=%d): [%.4f, %.4f, %.4f]", pose_num, candidates.poses.size(), k, target.position.x, target.position.y, target.position.z);

                        // Checking IK feasibility and whether the target is not in collision with workspace obstacles
                        planning_scene_monitor::LockedPlanningSceneRO scene(psm);
                        
                        auto current_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState()); //move_group.getCurrentState(1.0);
                        if (!current_state) 
                            RCLCPP_WARN(LOGGER, "No current state available; skipping feasibility check.");
                        else {
                            auto feasibility_check = checkFeasibility(*current_state, scene, move_group.getName(), target, move_group.getEndEffectorLink(), 0.05);
                            
                            if (!feasibility_check.is_feasible or feasibility_check.in_collision) {
                                RCLCPP_WARN(LOGGER, "Target %d/%zu (k=%d) is infeasible: %s", pose_num, candidates.poses.size(), k, feasibility_check.message.c_str());
                                continue;
                            } else { // if feasible and not colliding with scene obstacles, continue with planning and executing
                                RCLCPP_INFO(LOGGER, "Target %d/%zu (k=%d) passed feasibility check.", pose_num, candidates.poses.size(), k);
                            }
                        }

                        // Plan and Execute the Bezier/Straight Arc
                        if (planAndExecuteToTarget(node, move_group, planning_frame, *tf_buffer, cloud_observer, visualizer, ws_checker, robot_capsule_model, capsule_pub, current, target, use_collision_capsules)) {
                            RCLCPP_INFO(LOGGER, "HANDOVER SUCCESSFUL at k=%d, candidate %d/%zu", k, pose_num, candidates.poses.size());
                            final_success = true;
                            break;
                        }
                        RCLCPP_WARN(LOGGER, "Candidate %d/%zu failed (k=%d). Trying next...", pose_num, candidates.poses.size(), k);
                    }
                    if (final_success) break;

                    RCLCPP_WARN(LOGGER, "All candidates failed for k=%d. Increasing k...", k);
                }
            } else {
                RCLCPP_ERROR(LOGGER, "Handover failed: No active human body detected by HRI.");
            }

            // signal calculation node back to sleep
            auto sleep_req = std::make_shared<kiro_handover_interfaces::srv::ActivateHandover::Request>();
            sleep_req->handover_phase = false;
            calc_activation_client->async_send_request(sleep_req);

            // send response to External Caller --> END OF HANDOVER PHASE
            response->success = final_success;
            response->message = final_success ? "Handover sequence completed successfully." : "Handover sequence failed.";
            RCLCPP_INFO(LOGGER, ">>> Pipeline Finished: %s <<<", response->message.c_str());
        },
        rmw_qos_profile_services_default,
        callback_group
    );

    // multi-threaded executor to allow the service callback to run while waiting for client responses
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(LOGGER, "Execution Node spinning. Ready for trigger on '/activate_handover'");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}