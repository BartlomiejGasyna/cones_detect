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

namespace cones_detect
{

ConesDetectNode::ConesDetectNode(const rclcpp::NodeOptions & options)
:  Node("cones_detect", options)
{
  cones_detect_ = std::make_unique<cones_detect::ConesDetect>();
  param_name_ = this->declare_parameter("param_name", 456);
  cones_detect_->foo(param_name_);
}

}  // namespace cones_detect

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cones_detect::ConesDetectNode)
