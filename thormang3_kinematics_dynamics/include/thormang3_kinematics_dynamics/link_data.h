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
 * link_data.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_
#define THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"

namespace thormang3
{

class LinkData
{
public:

  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::MatrixXd relative_position_;
  Eigen::MatrixXd joint_axis_;
  Eigen::MatrixXd center_of_mass_;
  Eigen::MatrixXd inertia_;
  Eigen::MatrixXd joint_center_of_mass_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::MatrixXd position_;
  Eigen::MatrixXd orientation_;
  Eigen::MatrixXd transformation_;

};

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_ */
