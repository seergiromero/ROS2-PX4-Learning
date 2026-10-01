cmake_minimum_required(VERSION 3.8)
project(exploration_planner_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- AStar3D library -------------------------------------------------------
add_library(astar_3d src/astar_3d.cpp)
target_include_directories(astar_3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(astar_3d PUBLIC ${OCTOMAP_LIBRARIES})

# --- exploration_planner_node ---------------------------------------------
add_executable(exploration_planner_node src/exploration_planner_node.cpp)
target_include_directories(exploration_planner_node PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(exploration_planner_node
  astar_3d
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(exploration_planner_node
  geometry_msgs
  nav_msgs
  octomap_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS exploration_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_3d
    src/astar_3d.cpp
    test/test_astar_3d.cpp
  )
  if(TARGET test_astar_3d)
    target_include_directories(test_astar_3d PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OCTOMAP_INCLUDE_DIRS}
    )
    target_link_libraries(test_astar_3d
      ${OCTOMAP_LIBRARIES}
    )
  endif()
endif()

ament_package()
