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
 * kinematcis_dynamics.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "kinematics_dynamics_define.h"
#include "link_data.h"

namespace thormang3
{

enum TreeSelect {
  Manipulation,
  Walking,
  WholeBody
};

class KinematicsDynamics
{

public:
  KinematicsDynamics();
  ~KinematicsDynamics();
  KinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMassCenter(int joint_id);
  Eigen::MatrixXd calcCenterOfMass(Eigen::MatrixXd mc);

  void calcJointsCenterOfMass(int joint_id);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);

  LinkData *thormang3_link_data_ [ ALL_JOINT_ID + 1 ];

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
