/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdio.h>
#include "motion_module_tutorial/motion_module_tutorial.h"

using namespace thormang3;

MotionModuleTutorial::MotionModuleTutorial()
  : control_cycle_msec_(8)
{
  enable_       = false;
  module_name_  = "test_motion_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["r_sho_roll"] = new robotis_framework::DynamixelState();
  result_["r_el"] = new robotis_framework::DynamixelState();
}

MotionModuleTutorial::~MotionModuleTutorial()
{
  queue_thread_.join();
}

void MotionModuleTutorial::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&MotionModuleTutorial::queueThread, this));
}

void MotionModuleTutorial::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  sub1_ = ros_node.subscribe("/tutorial_topic", 10, &MotionModuleTutorial::topicCallback, this);

  /* publisher */
  pub1_ = ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void MotionModuleTutorial::topicCallback(const std_msgs::Int16::ConstPtr &msg)
{
  std_msgs::Int16 msg_int16;
  msg_int16.data = msg->data;
  pub1_.publish(msg_int16);
}

void MotionModuleTutorial::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end();
      state_iter++)
  {
    int32_t p_pos = dxls[state_iter->first]->dxl_state_->present_position_;
    int32_t g_pos = dxls[state_iter->first]->dxl_state_->goal_position_;
  }

  // ...

  result_["r_sho_pitch"]->goal_position_ = 0;
  result_["r_sho_roll"]->goal_position_ = 0;
  result_["r_el"]->goal_position_ = 0;
}

void MotionModuleTutorial::stop()
{
  return;
}

bool MotionModuleTutorial::isRunning()
{
  return false;
}

