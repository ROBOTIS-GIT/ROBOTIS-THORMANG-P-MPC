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

/*
 * HeadControlModule.h
 *
 *  Created on: 2016. 1. 27.
 *      Author: Kayman
 */

#ifndef THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_
#define THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "thormang3_head_control_module_msgs/HeadJointPose.h"

namespace thormang3
{

class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
{
 public:
  HeadControlModule();
  virtual ~HeadControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

 private:
  const double SCAN_START_ANGLE = -10 * M_PI / 180;
  const double SCAN_END_ANGLE = 85 * M_PI / 180;

  /* ROS Topic Callback Functions */
  void get3DLidarCallback(const std_msgs::String::ConstPtr &msg);
  void get3DLidarRangeCallback(const std_msgs::Float64::ConstPtr &msg);
  void setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setHeadJointTimeCallback(const thormang3_head_control_module_msgs::HeadJointPose::ConstPtr &msg);

  void queueThread();
  void jointTraGeneThread();
  void lidarJointTraGeneThread();

  void beforeMoveLidar(double start_angle);
  void startMoveLidar(double target_angle);
  void afterMoveLidar();
  void publishLidarMoveMsg(std::string msg_data);
  void publishDoneMsg(const std::string done_msg);

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                        double vel_end, double accel_end, double smp_time, double mov_time);

  Eigen::MatrixXd calcLinearInterpolationTra(double pos_start, double pos_end, double smp_time, double mov_time);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;
  ros::Publisher moving_head_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher movement_done_pub_;
  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_direct_control_;
  int tra_count_, tra_size_;
  double moving_time_;
  int current_state_;
  double original_position_lidar_;
  double scan_range_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;

  enum HeadLidarMode
  {
    None,
    BeforeStart,
    StartMove,
    EndMove,
    AfterMove,
    ModeCount
  };
};
}

#endif /* THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_ */
