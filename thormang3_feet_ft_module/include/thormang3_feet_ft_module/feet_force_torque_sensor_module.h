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
 * feet_force_torque_sensor_module.h
 *
 *  Created on: 2016. 3. 22.
 *      Author: Jay Song
 */

#ifndef THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_
#define THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_

#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_feet_ft_module_msgs/BothWrench.h"

#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "ati_ft_sensor/ati_force_torque_sensor.h"

namespace thormang3
{

class FeetForceTorqueSensor : public robotis_framework::SensorModule, public robotis_framework::Singleton<FeetForceTorqueSensor>
{
public:
  FeetForceTorqueSensor();
  ~FeetForceTorqueSensor();


  double r_foot_fx_raw_N_,  r_foot_fy_raw_N_,  r_foot_fz_raw_N_;
  double r_foot_tx_raw_Nm_, r_foot_ty_raw_Nm_, r_foot_tz_raw_Nm_;
  double l_foot_fx_raw_N_,  l_foot_fy_raw_N_,  l_foot_fz_raw_N_;
  double l_foot_tx_raw_Nm_, l_foot_ty_raw_Nm_, l_foot_tz_raw_Nm_;

  double r_foot_fx_scaled_N_,  r_foot_fy_scaled_N_,  r_foot_fz_scaled_N_;
  double r_foot_tx_scaled_Nm_, r_foot_ty_scaled_Nm_, r_foot_tz_scaled_Nm_;
  double l_foot_fx_scaled_N_,  l_foot_fy_scaled_N_,  l_foot_fz_scaled_N_;
  double l_foot_tx_scaled_Nm_, l_foot_ty_scaled_Nm_, l_foot_tz_scaled_Nm_;

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  void queueThread();

  void initializeFeetForceTorqueSensor();
  void saveFTCalibrationData(const std::string &path);

  void ftSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishBothFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left);


  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;


  KinematicsDynamics* thormang3_kd_;


  bool exist_r_leg_an_r_, exist_r_leg_an_p_;
  bool exist_l_leg_an_r_, exist_l_leg_an_p_;

  ATIForceTorqueSensorTWE r_foot_ft_sensor_;
  ATIForceTorqueSensorTWE l_foot_ft_sensor_;

  Eigen::MatrixXd r_foot_ft_air_, l_foot_ft_air_;
  Eigen::MatrixXd r_foot_ft_gnd_, l_foot_ft_gnd_;

  double r_foot_ft_current_voltage_[6];
  double l_foot_ft_current_voltage_[6];


  double total_mass_;
  double r_foot_ft_scale_factor_, l_foot_ft_scale_factor_;

  bool  has_ft_air_;
  bool  has_ft_gnd_;
  int   ft_command_;
  int   ft_period_;
  int   ft_get_count_;

  const int FT_NONE;
  const int FT_AIR;
  const int FT_GND;
  const int FT_CALC;


  ros::Publisher  thormang3_foot_ft_status_pub_;
  ros::Publisher  thormang3_foot_ft_both_ft_pub_;
};


}


#endif /* THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
