cmake_minimum_required(VERSION 3.8)
project(frontier_detector_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(frontier_detector_node
  src/octomap_mapper.cpp
  src/frontier_pipeline.cpp
  src/mean_shift.cpp
  src/frontier_detector.cpp
  src/frontier_visualizer.cpp
  src/frontier_detector_node.cpp
)

target_include_directories(frontier_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(frontier_detector_node
  geometry_msgs
  octomap_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
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
    src/mean_shift.cpp
    src/frontier_detector.cpp
    test/test_frontier_detector.cpp
  )
  if(TARGET test_frontier_detector)
    target_include_directories(test_frontier_detector PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OCTOMAP_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
    )
    ament_target_dependencies(test_frontier_detector
      geometry_msgs
    )
    target_link_libraries(test_frontier_detector
      ${OCTOMAP_LIBRARIES}
    )
  endif()

  ament_add_gtest(test_octomap_mapper
    src/octomap_mapper.cpp
    test/test_octomap_mapper.cpp
  )
  if(TARGET test_octomap_mapper)
    target_include_directories(test_octomap_mapper PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OCTOMAP_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
    )
    ament_target_dependencies(test_octomap_mapper
      sensor_msgs
    )
    target_link_libraries(test_octomap_mapper
      ${OCTOMAP_LIBRARIES}
    )
  endif()

  ament_add_gtest(test_frontier_pipeline
    src/octomap_mapper.cpp
    src/frontier_pipeline.cpp
    src/mean_shift.cpp
    src/frontier_detector.cpp
    test/test_frontier_pipeline.cpp
  )
  if(TARGET test_frontier_pipeline)
    target_include_directories(test_frontier_pipeline PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OCTOMAP_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
    )
    ament_target_dependencies(test_frontier_pipeline
      geometry_msgs
      rclcpp
      sensor_msgs
      tf2
      tf2_geometry_msgs
    )
    target_link_libraries(test_frontier_pipeline
      ${OCTOMAP_LIBRARIES}
    )
  endif()
endif()

ament_package()