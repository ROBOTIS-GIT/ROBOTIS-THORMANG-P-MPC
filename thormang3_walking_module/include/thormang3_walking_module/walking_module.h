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
 * online_walking_module.h
 *
 *  Created on: 2016. 9. 30.
 *      Author: Jay Song
 */

#ifndef THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_
#define THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "thormang3_walking_module/thormang3_online_walking.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/StartWalking.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/SetJointFeedBackGain.h"

#include "robotis_framework_common/motion_module.h"

#define WALKING_TUNE
#ifdef WALKING_TUNE
#include "thormang3_walking_module_msgs/WalkingJointStatesStamped.h"
#endif

namespace thormang3
{

class OnlineWalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<OnlineWalkingModule>
{
public:
  OnlineWalkingModule();
  virtual ~OnlineWalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  double gyro_roll_, gyro_pitch_;
  double orientation_roll_, orientation_pitch_;
  double r_foot_fx_N_,  r_foot_fy_N_,  r_foot_fz_N_;
  double r_foot_Tx_Nm_, r_foot_Ty_Nm_, r_foot_Tz_Nm_;
  double l_foot_fx_N_,  l_foot_fy_N_,  l_foot_fz_N_;
  double l_foot_Tx_Nm_, l_foot_Ty_Nm_, l_foot_Tz_Nm_;

private:
  void publishRobotPose(void);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
#ifdef WALKING_TUNE
  void publishWalkingTuningData();
#endif

  /* ROS Topic Callback Functions */
  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);

  /* ROS Service Callback Functions */
  bool setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
                                      thormang3_walking_module_msgs::SetBalanceParam::Response &res);

  bool setJointFeedBackGainServiceCallback(thormang3_walking_module_msgs::SetJointFeedBackGain::Request  &req,
                                           thormang3_walking_module_msgs::SetJointFeedBackGain::Response &res);

  bool getReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request  &req,
                                            thormang3_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request  &req,
                                   thormang3_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(thormang3_walking_module_msgs::StartWalking::Request  &req,
                                   thormang3_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request  &req,
                                thormang3_walking_module_msgs::IsRunning::Response &res);
  bool removeExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
                                             thormang3_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, thormang3_walking_module_msgs::StepData& des);

  void setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg);

  void updateBalanceParam();

  bool checkBalanceOnOff();

  void queueThread();

  void setJointFeedBackGain(thormang3_walking_module_msgs::JointFeedBackGain& msg);
  void updateJointFeedBackGain();

  std::map<std::string, int> joint_name_to_index_;

  bool            gazebo_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    process_mutex_;

  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;
  Eigen::MatrixXd desired_matrix_g_to_cob_;
  Eigen::MatrixXd desired_matrix_g_to_rfoot_;
  Eigen::MatrixXd desired_matrix_g_to_lfoot_;

  bool previous_running_, present_running;

  /* ROS Topic Publish Functions */
  int r_foot_ft_publish_checker_;
  int l_foot_ft_publish_checker_;
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher pelvis_base_msg_pub_;
  ros::Publisher done_msg_pub_;
#ifdef WALKING_TUNE
  ros::Publisher walking_joint_states_pub_;
  ros::Publisher imu_orientation_states_pub_;
  ros::Publisher ft_states_pub_;
  thormang3_walking_module_msgs::WalkingJointStatesStamped walking_joint_states_msg_;
#endif

  thormang3_walking_module_msgs::RobotPose  robot_pose_msg_;
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  Eigen::MatrixXd balance_update_polynomial_coeff_;


  bool joint_feedback_update_with_loop_;
  double joint_feedback_update_duration_;
  double joint_feedback_update_sys_time_;
  Eigen::MatrixXd joint_feedback_update_polynomial_coeff_;

  thormang3_walking_module_msgs::JointFeedBackGain previous_joint_feedback_gain_;
  thormang3_walking_module_msgs::JointFeedBackGain current_joint_feedback_gain_;
  thormang3_walking_module_msgs::JointFeedBackGain desired_joint_feedback_gain_;

  thormang3_walking_module_msgs::BalanceParam previous_balance_param_;
  thormang3_walking_module_msgs::BalanceParam current_balance_param_;
  thormang3_walking_module_msgs::BalanceParam desired_balance_param_;

};

}

#endif /* THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_ */
