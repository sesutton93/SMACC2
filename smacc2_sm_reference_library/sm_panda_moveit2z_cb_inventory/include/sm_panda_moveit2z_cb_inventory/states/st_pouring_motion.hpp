// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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
 *****************************************************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

namespace sm_panda_moveit2z_cb_inventory
{
// SMACC2 classes
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;
using namespace cl_keyboard;

// STATE DECLARATION
struct StPouringMotion : smacc2::SmaccState<StPouringMotion, SmPandaMoveit2zCbInventory>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef boost::mpl::list<
      Transition<EvCbSuccess<CbCircularPouringMotion, OrArm>, StMoveLastTrajectoryInitialState, SUCCESS>,
      Transition<EvCbFailure<CbCircularPouringMotion, OrArm>, StMoveLastTrajectoryInitialState, ABORT>,

      Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StMoveKnownState, PREVIOUS>,  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StMoveLastTrajectoryInitialState, NEXT>  
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   geometry_msgs::msg::Point relativePivotPoint;
   relativePivotPoint.x = -0.01;
   double deltaHeight = 0.05;
   std::string tipLink = "panda_rightfinger";
   std::string globalFrame = "panda_rightfinger";

    configure_orthogonal<OrArm, CbCircularPouringMotion>(relativePivotPoint, deltaHeight, tipLink, globalFrame);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StPouringMotion"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_panda_moveit2z_cb_inventory
