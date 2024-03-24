cmake_minimum_required(VERSION 3.5)
project(cones_detect)


SET(SPECIAL_OS_LIBS "")

find_package(autoware_cmake REQUIRED)
autoware_package()


find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(${PROJECT_NAME} SHARED
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE SRC_FILES src/*.c*)
FILE(GLOB_RECURSE HDR_FILES include/*.h*)

# cuda_add_executable(${PROJECT_NAME}_node_exe ${HDR_FILES} ${SRC_FILES})

# link_libraries(/usr/local/cuda-12.3/targets/x86_64-linux/lib/)
# tensorrt
include_directories(/usr/include/)
link_directories(/usr/lib/)

# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)


IF(NOT WIN32)
     SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings)
ENDIF()

add_definitions(-O3 -D_MWAITXINTRIN_H_INCLUDED -Wno-deprecated-declarations)


link_directories(${CUDA_LIBRARY_DIRS})

link_libraries(${CUDA_LIBRARIES} ${OpenCV_LIBS} nvinfer nvinfer_plugin nvparsers nvonnxparser ${SPECIAL_OS_LIBS})

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
