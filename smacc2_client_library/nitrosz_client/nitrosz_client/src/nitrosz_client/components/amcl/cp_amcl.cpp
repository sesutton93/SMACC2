// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <nitrosz_client/components/amcl/cp_amcl.hpp>

#include <string>

namespace cl_nitrosz
{
CpAmcl::CpAmcl() {}

CpAmcl::~CpAmcl() {}

std::string CpAmcl::getName() const { return "AMCL"; }

void CpAmcl::onInitialize()
{
  initalPosePub_ = getNode()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(10));
}

void CpAmcl::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose)
{
  initalPosePub_->publish(initialpose);
}

}  // namespace cl_nitrosz
