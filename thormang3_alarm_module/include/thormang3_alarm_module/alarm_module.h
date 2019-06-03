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

#ifndef THORMANG3_ALARM_MODULE_ALARM_MODULE_H_
#define THORMANG3_ALARM_MODULE_ALARM_MODULE_H_

#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/sensor_module.h"
#include "thormang3_alarm_module_msgs/JointOverload.h"
#include "thormang3_alarm_module_msgs/JointOverloadStatus.h"

#include "joint_overload.h"

namespace thormang3
{

class AlarmModule : public robotis_framework::SensorModule, public robotis_framework::Singleton<AlarmModule>
{
public:
  AlarmModule();
  ~AlarmModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  void queueThread();

  void initializeAlarmModule(robotis_framework::Robot *robot);
  // read overload threshold
  void getOverloadLimit(const std::string &path);

//  double calcOverLoadRate(double load_rate, double load_time, double load_ratio);
  void publishStatusMsg(unsigned int type, std::string msg);
  void overloadCommandCallback(const std_msgs::String::ConstPtr &msg);


  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;

  ros::Publisher  thormang3_alarm_pub_;
  ros::Publisher  thormang3_overload_pub_;
  ros::Publisher  thormang3_overload_status_pub_;
  ros::Subscriber overload_command_sub_;

  std::map<std::string, double> overload_limits_;
  std::map<std::string, JointOverload*> overload_calc_;
};


}
#endif /* THORMANG3_ALARM_MODULE_ALARM_MODULE_H_ */
