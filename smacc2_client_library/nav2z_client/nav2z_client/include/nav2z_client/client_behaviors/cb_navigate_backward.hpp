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
#pragma once

#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>

#include <optional>

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator.hpp>
#include "cb_nav2z_client_behavior_base.hpp"

namespace cl_nav2z
{
// It sends the mobile base some distance backwards
struct CbNavigateBackwardOptions
{
  // just a stub to show how to use parameterless constructor
  std::optional<float> backwardSpeed;

  // this may be useful in the case you want to do a straight line with some known direction
  // and the robot may not have that specific initial orientation at that moment.
  // If it is not set, the orientation of the straight line is the orientation of the initial (current) state.
  std::optional<geometry_msgs::msg::Quaternion> forceInitialOrientation;

  // the name of the goal checker selected in the navigation2 stack
  std::optional<std::string> goalChecker_;
};

// Performs a relative motion backwards
class CbNavigateBackward : public CbNav2ZClientBehaviorBase
{
public:
  CbNavigateBackwardOptions options;

  CbNavigateBackward();

  CbNavigateBackward(float backwardDistance);

  CbNavigateBackward(geometry_msgs::msg::PoseStamped goalPosition); 
  ~CbNavigateBackward();


  void onEntry() override;

  void onExit() override;

  void setBackwardDistance(float distance_meters);

protected:
  // required component
  odom_tracker::CpOdomTracker * odomTracker_;

  std::optional<geometry_msgs::msg::PoseStamped> goalPose_;

  std::optional<float> backwardDistance_;
};
}  // namespace cl_nav2z
