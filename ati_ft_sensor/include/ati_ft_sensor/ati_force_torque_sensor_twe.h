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

#ifndef ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_
#define ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_math/robotis_math.h"

namespace thormang3
{

class ATIForceTorqueSensorTWE
{
public:
  ATIForceTorqueSensorTWE();
  ~ATIForceTorqueSensorTWE();

  bool initialize(const std::string& ft_data_path,			  const std::string& ft_data_key,
                  const std::string& ft_frame_id,
                  const std::string& ft_raw_publish_name,	const std::string& ft_scaled_publish_name);

  void setScaleFactror(double ft_scale_factor);
  void setNullForceTorque(Eigen::MatrixXd _ft_null);
  void setScaleParam(double ft_scale_factor, Eigen::MatrixXd ft_null);


  void setCurrentVoltageOutput(double voltage0, double voltage1, double voltage2,
                               double voltage3, double voltage4, double voltage5);
  void setCurrentVoltageOutput(Eigen::MatrixXd voltage);

  Eigen::MatrixXd getCurrentForceTorqueRaw();
  Eigen::MatrixXd getCurrentForceTorqueScaled();

  void getCurrentForceTorqueRaw(double *force_x_N,   double *force_y_N,   double *force_z_N,
                                double *torque_x_Nm, double *torque_y_Nm, double *torque_z_Nm);
  void getCurrentForceTorqueScaled(double *force_x_N,   double *force_y_N,   double *force_z_N,
                                   double *torque_x_Nm, double *torque_y_Nm, double *torque_z_Nm);

  void setCurrentVoltageOutputPublish(double voltage0, double voltage1, double voltage2,
                                      double voltage3, double voltage4, double voltage5);
  void setCurrentVoltageOutputPublish(Eigen::MatrixXd voltage);

private:
  bool parseFTData(const std::string& ft_data_path, const std::string& ft_data_key);

  Eigen::MatrixXd ft_coeff_mat_;
  Eigen::MatrixXd ft_unload_volatge_;
  Eigen::MatrixXd ft_current_volatge_;
  Eigen::MatrixXd ft_null_;
  Eigen::MatrixXd ft_raw_;
  Eigen::MatrixXd ft_scaled_;

  boost::mutex    ft_scale_param_mutex_;


  double ft_scale_factor_;

  std::string ft_frame_id_;
  std::string ft_raw_publish_name_;
  std::string ft_scaled_publish_name_;

  bool is_ft_raw_published_;
  bool is_ft_scaled_published_;


  ros::Publisher ft_raw_pub_;
  ros::Publisher ft_scaled_pub_;

  geometry_msgs::WrenchStamped ft_raw_msg_;
  geometry_msgs::WrenchStamped ft_scaled_msg_;
};

}


#endif /* ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_ */
