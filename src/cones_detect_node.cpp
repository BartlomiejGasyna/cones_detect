// Copyright 2024 BartlomiejGasyna
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cones_detect/cones_detect_node.hpp"

cv::Rect get_rect(BBox box) {
    return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
}


namespace cones_detect
{

ConesDetectNode::ConesDetectNode(const rclcpp::NodeOptions & options)
:  Node("cones_detect", options)
{
  cones_detect_ = std::make_unique<cones_detect::ConesDetect>();
  param_name_ = this->declare_parameter("param_name", 456);

  build_engine = this->declare_parameter("build_engine", false);
  onnx_path = this->declare_parameter("model_path", "model.onnx");
  engine_path = this->declare_parameter("engine_path", "engine.engine");

  std::cout << "onnx_path: " << onnx_path << std::endl;
  std::cout << "engine_path: " << engine_path << std::endl;

  setenv("CUDA_MODULE_LOADING", "LAZY", 1);

  // rclcpp::init(argc, argv);
  // auto node = std::make_shared<rclcpp::Node>("yolo_publisher");

  // auto yellow_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("yellow_cones", 10);
  // auto blue_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("blue_cones", 10);
  // auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("cones_img", 1);

  // if (argc == 1) {
  //       std::cout << "Usage: \n 1. ./yolo_onnx_zed -s yolov8s.onnx yolov8s.engine\n 2. ./yolo_onnx_zed -s yolov8s.onnx yolov8s.engine images:1x3x512x512\n 3. ./yolo_onnx_zed yolov8s.engine <SVO path>" << std::endl;
  //       return 0;
  //  }
    

    if (build_engine){
  // Check Optim engine first
  // if (std::string(argv[1]) == "-s" && (argc >= 4)) {
      // std::string onnx_path = std::string(argv[2]);
      // std::string engine_path = std::string(argv[3]);

      OptimDim dyn_dim_profile;

      // if (argc == 5) {
      //     std::string optim_profile = std::string(argv[4]);
      //     bool error = dyn_dim_profile.setFromString(optim_profile);
      //     if (error) {
      //         std::cerr << "Invalid dynamic dimension argument, expecting something like 'images:1x3x512x512'" << std::endl;
      //         return EXIT_FAILURE;
      //     }
      // }

      Yolo::build_engine(onnx_path, engine_path, dyn_dim_profile);
      std::cout << "Build finished" << std::endl;

  // }

}
}

}  // namespace cones_detect

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_detect::ConesDetectNode)
