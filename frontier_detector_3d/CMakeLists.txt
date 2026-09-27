cmake_minimum_required(VERSION 3.8)
project(frontier_detector_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(frontier_detector_node
  src/frontier_detector.cpp
  src/frontier_detector_node.cpp
)

target_include_directories(frontier_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OCTOMAP_INCLUDE_DIRS}
)

ament_target_dependencies(frontier_detector_node
  octomap_msgs
  rclcpp
  visualization_msgs
)

target_link_libraries(frontier_detector_node
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS frontier_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frontier_detector
    src/frontier_detector.cpp
    test/test_frontier_detector.cpp
  )
  if(TARGET test_frontier_detector)
    target_include_directories(test_frontier_detector PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OCTOMAP_INCLUDE_DIRS}
    )
    target_link_libraries(test_frontier_detector
      ${OCTOMAP_LIBRARIES}
    )
  endif()
endif()

ament_package()