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

#include "thormang3_alarm_module/alarm_module.h"

using namespace thormang3;

AlarmModule::AlarmModule()
  : control_cycle_msec_(8)
{
  
}

AlarmModule::~AlarmModule()
{
  queue_thread_.join();
}

void AlarmModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  queue_thread_ = boost::thread(boost::bind(&AlarmModule::queueThread, this));

  initializeAlarmModule(robot);

  // read limits and make table(map)
  std::string config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/overload.yaml";
  getOverloadLimit(config_path);
}

void AlarmModule::initializeAlarmModule(robotis_framework::Robot *robot)
{
  for (auto& it : robot->dxls_)
  {
    std::string joint_name = it.first;
    robotis_framework::Dynamixel *dxl = it.second;

    if(joint_name.find("leg") != std::string::npos)
    {
      overload_calc_[joint_name] = new JointOverload(dxl->model_name_, joint_name, control_cycle_msec_ / 1000.0);
      overload_limits_[joint_name] = 100;
    }
  }
}

void AlarmModule::getOverloadLimit(const std::string &path)
{
  // get overload limit from config file
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());

    // get overload
    YAML::Node overload_node = doc["overload"];
    for (YAML::iterator it = overload_node.begin(); it != overload_node.end(); ++it)
    {
      std::string joint_name;
      double overload_ratio;

      joint_name = it->first.as<std::string>();
      overload_ratio = it->second.as<double>();

      std::map<std::string, double>::iterator limit_it = overload_limits_.find(joint_name);
      if(limit_it != overload_limits_.end())
        limit_it->second = overload_ratio;
    }
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load overload config file.");
    return;
  }
//  overload_limits_["r_leg_hip_y"] = 100;
//  overload_limits_["r_leg_hip_r"] = 100;
//  overload_limits_["r_leg_hip_p"] = 100;
//  overload_limits_["r_leg_kn_p" ] = 100;
//  overload_limits_["r_leg_an_p" ] = 100;
//  overload_limits_["r_leg_an_r" ] = 100;

//  overload_limits_["l_leg_hip_y"] = 100;
//  overload_limits_["l_leg_hip_r"] = 100;
//  overload_limits_["l_leg_hip_p"] = 100;
//  overload_limits_["l_leg_kn_p" ] = 100;
//  overload_limits_["l_leg_an_p" ] = 100;
//  overload_limits_["l_leg_an_r" ] = 100;

  // set overload limit
  std::cout << "Overload Limit : " << std::endl;
  for (auto& it : overload_limits_)
  {
    std::string joint_name = it.first;
    double limit_value = it.second;

    overload_calc_[joint_name]->setOverloadLimit(limit_value);
    std::cout << "    [" << joint_name << "] : " << limit_value << std::endl;
  }
}

void AlarmModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Alarm";
  status_msg.status_msg = msg;

  thormang3_alarm_pub_.publish(status_msg);
}

void AlarmModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  ros::Subscriber overload_command_sub_ = ros_node.subscribe("/robotis/overload/command", 1, &AlarmModule::overloadCommandCallback, this);

  /* publisher */
  thormang3_alarm_pub_  = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  thormang3_overload_pub_ = ros_node.advertise<thormang3_alarm_module_msgs::JointOverload>("/robotis/overload/data", 1);
  thormang3_overload_status_pub_ = ros_node.advertise<thormang3_alarm_module_msgs::JointOverloadStatus>("/robotis/overload/status", 1);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void AlarmModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors)
{
  std::stringstream msg_ss;
  bool warn = false;
  thormang3_alarm_module_msgs::JointOverload overload_msg;
  thormang3_alarm_module_msgs::JointOverloadStatus overload_status_msg;
  overload_msg.header.stamp = ros::Time::now();
  int warning_count, error_count, error_status;

  // loop dxl to find joints to check overload limit
  for (auto& it : dxls)
  {
    std::string joint_name = it.first;
    robotis_framework::Dynamixel* dxl = it.second;

    if(joint_name.find("leg") != std::string::npos)
    {
      std::string present_current_name = dxl->present_current_item_->item_name_;
      uint16_t present_current_data = dxl->dxl_state_->bulk_read_table_[present_current_name];
      int16_t data = present_current_data;
      double p_current = data / 1000.0;  // mA -> A
      double total_overload = 0.0;

      double overload = overload_calc_[joint_name]->addPresentCurrent(p_current, total_overload);
      //      bool is_overload = overload_calc_[joint_name]->isOverloadError();

      bool status_changed = overload_calc_[joint_name]->getOverloadError(error_status, warning_count, error_count);

      if(status_changed == true)
      {
        std::string status_msg = "";
        if(error_status == JointOverload::NORMAL)
          status_msg = "NORMAL";
        if(error_status == JointOverload::WARNING)
          status_msg = "WARNING";
        if(error_status == JointOverload::ERROR)
          status_msg = "ERROR";
        if(error_status == JointOverload::EMERGENCY)
          status_msg = "EMERGENCY";

        msg_ss << "<" << status_msg << "> " << joint_name << "- OVERLOAD:" <<total_overload << ", CURRENT: " << p_current
               << ", W[" << warning_count << "], E[" << error_count << "]" << std::endl;
        warn = true;

        overload_status_msg.name.push_back(joint_name);
        overload_status_msg.status.push_back(error_status);
        overload_status_msg.warning_count.push_back(warning_count);
        overload_status_msg.error_count.push_back(error_count);
      }

      overload_msg.name.push_back(joint_name);
      overload_msg.current.push_back(p_current);
      overload_msg.overload.push_back(total_overload);
    }
  }

  if(warn == true)
  {
    std::string msg = msg_ss.str();
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, msg);
    if(overload_status_msg.name.size() != 0)
    {
      overload_status_msg.header.stamp = ros::Time::now();
      thormang3_overload_status_pub_.publish(overload_status_msg);
    }
  }

  thormang3_overload_pub_.publish(overload_msg);
}

void AlarmModule::overloadCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "reset")
  {
    // reset set overload limit
    for (auto& it : overload_calc_)
    {
      it.second->clearOverload();
    }
  }
  else if(msg->data == "load_limit")
  {
    std::string config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/overload.yaml";
    getOverloadLimit(config_path);
  }
}
