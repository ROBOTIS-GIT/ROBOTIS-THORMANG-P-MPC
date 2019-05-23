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

#ifndef THORMANG3_BASE_MODULE_BASE_MODULE_STATE_H_
#define THORMANG3_BASE_MODULE_BASE_MODULE_STATE_H_

#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

namespace thormang3
{

class BaseModuleState
{
public:
  BaseModuleState();
  ~BaseModuleState();

  bool is_moving_;

  int cnt_; // counter number

  double mov_time_; // movement time
  double smp_time_; // sampling time

  int all_time_steps_; // all time steps of movement time

  Eigen::MatrixXd calc_joint_tra_; // calculated joint trajectory

  Eigen::MatrixXd joint_ini_pose_;
  Eigen::MatrixXd joint_pose_;

  int via_num_;

  Eigen::MatrixXd joint_via_pose_;
  Eigen::MatrixXd joint_via_dpose_;
  Eigen::MatrixXd joint_via_ddpose_;

  Eigen::MatrixXd via_time_;
};

}

#endif /* THORMANG3_BASE_MODULE_BASE_MODULE_STATE_H_ */
