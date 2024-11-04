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

#include <nav2z_client/common.hpp>

#include <nav2z_client/client_behaviors/cb_navigate_backward.hpp>
#include <nav2z_client/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>

namespace cl_nav2z
{

using ::cl_nav2z::odom_tracker::CpOdomTracker;
using ::cl_nav2z::odom_tracker::WorkingMode;

using ::cl_nav2z::Pose;

CbNavigateBackward::CbNavigateBackward(float distance_meters) : backwardDistance_(distance_meters) {}
CbNavigateBackward::CbNavigateBackward() {}

CbNavigateBackward::CbNavigateBackward(geometry_msgs::msg::PoseStamped goal) : goalPose_(goal) {}

CbNavigateBackward::~CbNavigateBackward() {}


void CbNavigateBackward::setBackwardDistance(float distance_meters)
{
  if (distance_meters < 0)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] negative backward distance: " << distance_meters
                       << ". Resetting to 0.");
    distance_meters = 0;
  }
  backwardDistance_ = distance_meters;

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] setting bw motion distance: " << *backwardDistance_);
}

void CbNavigateBackward::onEntry()
{
  // straight motion distance
  if (backwardDistance_)
  {
    setBackwardDistance(*backwardDistance_);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] Straight motion distance: " << *backwardDistance_);
  }

  // get current pose
  auto p = nav2zClient_->getComponent<Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();


  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "]"
                     << "current pose: " << currentPoseMsg);

  // force global orientation if it is requested
  if (options.forceInitialOrientation)
  {
    currentPoseMsg.orientation = *(options.forceInitialOrientation);
    RCLCPP_WARN_STREAM(
      getLogger(),
      "[" << getName() << "]"
          << "Forcing initial straight motion orientation: " << currentPoseMsg.orientation);
  }

  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);

  tf2::Transform targetPose;
  if (goalPose_)
  {
    tf2::fromMsg(goalPose_->pose, targetPose);
  }
  else if (backwardDistance_)
  {
    // compute forward goal pose
    tf2::Transform backwardDeltaTransform;
    backwardDeltaTransform.setIdentity();
    backwardDeltaTransform.setOrigin(tf2::Vector3(-*backwardDistance_, 0, 0));

    targetPose = currentPose * backwardDeltaTransform;
  }
  else
  {
    RCLCPP_WARN_STREAM(
      getLogger(),
      "[" << getName() << "]"
          << "No goal Pose or Distance is specified. Aborting. No action request is sent."
          << currentPoseMsg.orientation);

    return;
  }


 
  // action goal
  ClNav2Z::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  //goal.pose.header.stamp = getNode()->now();
  tf2::toMsg(targetPose, goal.pose.pose);
  RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateBackward] TARGET POSE BACKWARDS: " << goal.pose);

  // current pose
  geometry_msgs::msg::PoseStamped currentStampedPoseMsg;
  currentStampedPoseMsg.header.frame_id = referenceFrame;
  currentStampedPoseMsg.header.stamp = getNode()->now();

  tf2::toMsg(currentPose, currentStampedPoseMsg.pose);

  odomTracker_ = nav2zClient_->getComponent<CpOdomTracker>();
  if (odomTracker_ != nullptr)
  {
    auto pathname = this->getCurrentState()->getName() + " - " + getName();
    odomTracker_->pushPath(pathname);
    odomTracker_->setStartPoint(currentStampedPoseMsg);
    odomTracker_->setCurrentMotionGoal(goal.pose);
    odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);
  }

  auto plannerSwitcher = nav2zClient_->getComponent<CpPlannerSwitcher>();
  plannerSwitcher->setBackwardPlanner();

  auto goalCheckerSwitcher = nav2zClient_->getComponent<CpGoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("backward_goal_checker");

  this->sendGoal(goal);
}

void CbNavigateBackward::onExit()
{
  if (odomTracker_)
  {
    this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
  }
}

}  // namespace cl_nav2z
