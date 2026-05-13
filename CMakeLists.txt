cmake_minimum_required(VERSION 3.8)
project(kiro_handover_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kiro_handover_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(handover_execution 
  src/handover_execution.cpp
  src/point_cloud_observer.cpp
  src/bezier_arcs.cpp
  src/arc_planner.cpp
  src/path_visualizer.cpp
  src/workspace_checker.cpp
  src/feasibility.cpp
  src/Ur10eCollisionModel.cpp
)

target_include_directories(handover_execution PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kiro_handover_execution
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(handover_execution
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  std_srvs
  kiro_handover_interfaces
  visualization_msgs
)

# install targets
install(TARGETS 
  handover_execution
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
