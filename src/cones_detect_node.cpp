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
#include <sensor_msgs/msg/image.hpp>

#define NMS_THRESH 0.4
#define CONF_THRESH 0.3


cv::Rect get_rect(BBox box) {
    return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
}


namespace cones_detect
{
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

ConesDetectNode::ConesDetectNode(const rclcpp::NodeOptions & options)
:  Node("cones_detect", options)
{
  cones_detect_ = std::make_unique<cones_detect::ConesDetect>();
  param_name_ = this->declare_parameter("param_name", 456);

  build_engine = this->declare_parameter("build_engine", false);
  onnx_path = this->declare_parameter("model_path", "model.onnx");
  engine_path = this->declare_parameter("engine_path", "engine.engine");

  // rclcpp::QoS qos_settings(10);
  // qos_settings.best_effort();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/sensing/camera/image_raw",
    custom_qos,
    std::bind(&ConesDetectNode::imageCallback, this, 
    std::placeholders::_1));

  bboxes_pub_ = this->create_publisher<cones_interfaces::msg::Cones>("output_bboxes", custom_qos);
  
  
  setenv("CUDA_MODULE_LOADING", "LAZY", 1);

  if (build_engine){
    OptimDim dyn_dim_profile;
    Yolo::build_engine(onnx_path, engine_path, dyn_dim_profile);
    std::cout << "Build finished" << std::endl;
  }

  if (detector.init(engine_path)) {
    std::cerr << "Detector init failed" << std::endl;
  }
  else {
    std::cout << "Detector init success" << std::endl;
  }

}

void ConesDetectNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::cout << "Image received" << std::endl;
  cv::Mat frame_cv;
  frame_cv = cv_bridge::toCvCopy(msg, "bgr8")->image;

  auto detections = detector.run(frame_cv, frame_cv.rows, frame_cv.cols, CONF_THRESH);



    // Displaying 'raw' objects
  for (size_t j = 0; j < detections.size(); j++) {
      cv::Rect r = get_rect(detections[j].box);
      cv::rectangle(frame_cv, r, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
      cv::putText(frame_cv, std::to_string((int) detections[j].label), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
  cv::imshow("Cones", frame_cv);
  cv::waitKey(1);

  // publish detected bboxes
  cones_interfaces::msg::Cones cones;
  cones.header = msg->header;
  for (size_t j = 0; j < detections.size(); j++) {
    cones_interfaces::msg::BoundingBox bbox;
    bbox.x1 = detections[j].box.x1;
    bbox.y1 = detections[j].box.y1;
    bbox.x2 = detections[j].box.x2;
    bbox.y2 = detections[j].box.y2;
    bbox.label = detections[j].label == 0 ? 'Y' : 'B';

    cones.bboxes.push_back(bbox);
  }

  bboxes_pub_->publish(cones);
}


}  // namespace cones_detect

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_detect::ConesDetectNode)


