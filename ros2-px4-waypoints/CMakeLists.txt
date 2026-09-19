cmake_minimum_required(VERSION 3.8)
project(ros2-px4-waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(offboard_waypoints_node src/offboard_waypoints_node.cpp)
ament_target_dependencies(offboard_waypoints_node
  px4_msgs
  rclcpp
)

add_executable(offboard_takeoff_node src/offboard_takeoff_node.cpp)
ament_target_dependencies(offboard_takeoff_node
  px4_msgs
  rclcpp
)

add_executable(px4_odometry_tf_node src/px4_odometry_tf_node.cpp)
target_include_directories(px4_odometry_tf_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(px4_odometry_tf_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  offboard_waypoints_node
  offboard_takeoff_node
  px4_odometry_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude linters that do not apply to this package template.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
  )
  # xmllint rejects the legacy hyphenated package name (ROS 2 names only allow
  # the '[a-z](_?[a-z0-9]+)*' pattern); renaming would break launch and docs.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp)
  target_include_directories(test_frame_conversions PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_frame_conversions tf2)
endif()

ament_package()
