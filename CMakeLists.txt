cmake_minimum_required(VERSION 3.14)
project(cones_detect)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/cones_detect/cones_detect.hpp
  include/cones_detect/visibility_control.hpp
  src/cones_detect.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/cones_detect/cones_detect_node.hpp
  src/cones_detect_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "cones_detect::ConesDetectNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_cones_detect
    test/test_cones_detect.cpp
  )
  ament_target_dependencies(test_cones_detect ${TEST_SOURCES})
  target_link_libraries(test_cones_detect ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
