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
 *  manipulation_module.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_
#define THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"


namespace thormang3
{

class ManipulationModule: public robotis_framework::MotionModule,
                          public robotis_framework::Singleton<ManipulationModule>
{
public:
  ManipulationModule();
  virtual ~ManipulationModule();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void jointPoseMsgCallback(const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg);
  void kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg);

  bool getJointPoseCallback(thormang3_manipulation_module_msgs::GetJointPose::Request &req,
                            thormang3_manipulation_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req,
                                 thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res);

  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();
  void jointTrajGenerateProc();
  void taskTrajGenerateProc();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  KinematicsDynamics *robotis_;

private:
  void queueThread();

  void parseData(const std::string &path);
  void parseIniPoseData(const std::string &path);

  bool arm_angle_display_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::thread  *traj_generate_tread_;

  std_msgs::String movement_done_msg_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  movement_done_pub_;

  /* joint state */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd init_joint_position_;

  /* trajectory */
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;

  Eigen::MatrixXd goal_joint_tra_;
  Eigen::MatrixXd goal_task_tra_;

    /* msgs */
  thormang3_manipulation_module_msgs::JointPose       goal_joint_pose_msg_;
  thormang3_manipulation_module_msgs::KinematicsPose  goal_kinematics_pose_msg_;

  /* inverse kinematics */
  bool  ik_solving_;
  int   ik_id_start_;
  int   ik_id_end_;

  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_;
  Eigen::MatrixXd ik_target_rotation_;
  Eigen::MatrixXd ik_weight_;

  void setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation);

  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_ */
