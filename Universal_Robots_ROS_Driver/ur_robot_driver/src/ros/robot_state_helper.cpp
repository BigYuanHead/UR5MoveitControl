// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/ros/robot_state_helper.h>

#include <std_srvs/Trigger.h>
namespace ur_driver
{
  RobotStateHelper::RobotStateHelper(const ros::NodeHandle& nh) : nh_(nh), set_mode_as_(nh_, "set_mode", false)
  {
    // Topic on which the robot_mode is published by the driver
    robot_mode_sub_ = nh_.subscribe("robot_mode", 1, &RobotStateHelper::robotModeCallback, this);
    // Topic on which the safety is published by the driver
    safety_mode_sub_ = nh_.subscribe("safety_mode", 1, &RobotStateHelper::safetyModeCallback, this);

    // Service to unlock protective stop
    unlock_protective_stop_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/unlock_protective_stop");
    // Service to restart safety
    restart_safety_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/restart_safety");
    // Service to power on the robot
    power_on_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/power_on");
    // Service to power off the robot
    power_off_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/power_off");
    // Service to release the robot's brakes
    brake_release_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/brake_release");
    // Service to stop UR program execution on the robot
    stop_program_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/stop");
    // Service to start UR program execution on the robot
    play_program_srv_ = nh_.serviceClient<std_srvs::Trigger>("dashboard/play");

    set_mode_as_.registerGoalCallback(std::bind(&RobotStateHelper::setModeGoalCallback, this));
    set_mode_as_.registerPreemptCallback(std::bind(&RobotStateHelper::setModePreemptCallback, this));
    set_mode_as_.start();
}

void RobotStateHelper::robotModeCallback(const ur_dashboard_msgs::RobotMode& msg)
{
  if (robot_mode_ != static_cast<RobotMode>(msg.mode))
  {
    robot_mode_ = RobotMode(msg.mode);
    ROS_INFO_STREAM("Robot mode is now " << robotModeString(robot_mode_));
    updateRobotState();
  }
}

void RobotStateHelper::safetyModeCallback(const ur_dashboard_msgs::SafetyMode& msg)
{
  if (safety_mode_ != static_cast<SafetyMode>(msg.mode))
  {
    safety_mode_ = SafetyMode(msg.mode);
    ROS_INFO_STREAM("Robot's safety mode is now " << safetyModeString(safety_mode_));
    updateRobotState();
  }
}

void RobotStateHelper::updateRobotState()
{
  if (set_mode_as_.isActive())
  {
    // Update action feedback
    feedback_.current_robot_mode =
        static_cast<ur_dashboard_msgs::SetModeFeedback::_current_robot_mode_type>(robot_mode_);
    feedback_.current_safety_mode =
        static_cast<ur_dashboard_msgs::SetModeFeedback::_current_safety_mode_type>(safety_mode_);
    set_mode_as_.publishFeedback(feedback_);

    if (robot_mode_ < static_cast<RobotMode>(goal_->target_robot_mode) || safety_mode_ > SafetyMode::REDUCED)
    {
      // Transition to next mode
      ROS_DEBUG_STREAM("Current robot mode is " << robotModeString(robot_mode_) << " while target mode is "
                                                << robotModeString(static_cast<RobotMode>(goal_->target_robot_mode)));
      doTransition();
    }
    else if (robot_mode_ == static_cast<RobotMode>(goal_->target_robot_mode))
    {
      // Final mode reached
      result_.success = true;
      result_.message = "Reached target robot mode.";
      if (robot_mode_ == RobotMode::RUNNING && goal_->play_program)
      {
        // The dashboard denies playing immediately after switching the mode to RUNNING
        sleep(1);
        safeDashboardTrigger(&this->play_program_srv_);
      }

      // Action could be aborted, e.g. because of failing play request
      if (set_mode_as_.isActive())
      {
        set_mode_as_.setSucceeded(result_);
      }
    }
    else
    {
      result_.success = false;
      result_.message = "Robot reached higher mode than requested during recovery. This either means that something "
                        "went wrong or that a higher mode was requested from somewhere else (e.g. the teach "
                        "pendant.)";
      set_mode_as_.setAborted(result_);
    }
  }
}

void RobotStateHelper::doTransition()
{
  if (static_cast<RobotMode>(goal_->target_robot_mode) < robot_mode_)
  {
    // Go through power_off if lower mode is requested
    safeDashboardTrigger(&this->power_off_srv_);
  }
  else
  {
    switch (safety_mode_)
    {
      case SafetyMode::PROTECTIVE_STOP:
        safeDashboardTrigger(&this->unlock_protective_stop_srv_);
        break;
      case SafetyMode::SYSTEM_EMERGENCY_STOP:
      case SafetyMode::ROBOT_EMERGENCY_STOP:
        ROS_WARN_STREAM("The robot is currently in safety mode " << safetyModeString(safety_mode_)
                                                                 << ". Please release the EM-Stop to proceed.");
        break;
      case SafetyMode::VIOLATION:
      case SafetyMode::FAULT:
        safeDashboardTrigger(&this->restart_safety_srv_);
        break;
      default:
        switch (robot_mode_)
        {
          case RobotMode::CONFIRM_SAFETY:
            ROS_WARN_STREAM("The robot is currently in mode " << robotModeString(robot_mode_)
                                                              << ". It is required to interact with the teach pendant "
                                                                 "at this point.");
            break;
          case RobotMode::BOOTING:
            ROS_INFO_STREAM("The robot is currently in mode " << robotModeString(robot_mode_)
                                                              << ". Please wait until the robot is booted up...");
            break;
          case RobotMode::POWER_OFF:
            safeDashboardTrigger(&this->power_on_srv_);
            break;
          case RobotMode::POWER_ON:
            ROS_INFO_STREAM("The robot is currently in mode " << robotModeString(robot_mode_)
                                                              << ". Please wait until the robot is in mode "
                                                              << robotModeString(RobotMode::IDLE));
            break;
          case RobotMode::IDLE:
            safeDashboardTrigger(&this->brake_release_srv_);
            break;
          case RobotMode::BACKDRIVE:
            ROS_INFO_STREAM("The robot is currently in mode "
                            << robotModeString(robot_mode_) << ". It will automatically return to mode "
                            << robotModeString(RobotMode::IDLE) << " once the teach button is released.");
            break;
          case RobotMode::RUNNING:
            ROS_INFO_STREAM("The robot has reached operational mode " << robotModeString(robot_mode_));
            break;
          default:
            ROS_WARN_STREAM("The robot is currently in mode " << robotModeString(robot_mode_)
                                                              << ". This won't be handled by this helper. Please "
                                                                 "resolve "
                                                                 "this manually.");
        }
    }
  }
}

void RobotStateHelper::setModeGoalCallback()
{
  goal_ = set_mode_as_.acceptNewGoal();

  RobotMode target_mode = static_cast<RobotMode>(goal_->target_robot_mode);

  // Do some input sanitation first.
  switch (target_mode)
  {
    case RobotMode::POWER_OFF:
    case RobotMode::IDLE:
    case RobotMode::RUNNING:
      if (robot_mode_ != target_mode || safety_mode_ > SafetyMode::REDUCED)
      {
        if (goal_->stop_program)
        {
          safeDashboardTrigger(&this->stop_program_srv_);
        }
        doTransition();
      }
      else
      {
        // There is no transition to do here, but we have to start the program again, if desired.
        // This happens inside updateRobotState()
        updateRobotState();
      }
      break;
    case RobotMode::NO_CONTROLLER:
    case RobotMode::DISCONNECTED:
    case RobotMode::CONFIRM_SAFETY:
    case RobotMode::BOOTING:
    case RobotMode::POWER_ON:
    case RobotMode::BACKDRIVE:
    case RobotMode::UPDATING_FIRMWARE:
      result_.message =
          "Requested target mode " + robotModeString(target_mode) + " which cannot be explicitly selected.";
      result_.success = false;
      set_mode_as_.setAborted(result_);
      break;
    default:
      result_.message = "Requested illegal mode.";
      result_.success = false;
      set_mode_as_.setAborted(result_);
  }
}

void RobotStateHelper::setModePreemptCallback()
{
  ROS_INFO_STREAM("Current goal got preempted.");
  set_mode_as_.setPreempted();
}

bool RobotStateHelper::safeDashboardTrigger(ros::ServiceClient* srv_client)
{
  assert(srv_client != nullptr);
  std_srvs::Trigger srv;
  srv_client->call(srv);
  ROS_INFO_STREAM(srv.response.message);
  if (!srv.response.success)
  {
    result_.success = false;
    result_.message = "An error occurred while switching the robot mode. The respective dashboard service returned "
                      "non-success: " +
                      srv.response.message;
    set_mode_as_.setAborted(result_);
  }
  return srv.response.success;
}
}  // namespace ur_driver
