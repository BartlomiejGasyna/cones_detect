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

#include "gtest/gtest.h"
#include "cones_detect/cones_detect.hpp"

TEST(TestConesDetect, TestHello) {
  std::unique_ptr<cones_detect::ConesDetect> cones_detect_ =
    std::make_unique<cones_detect::ConesDetect>();
  auto result = cones_detect_->foo(999);
  EXPECT_EQ(result, 999);
}
