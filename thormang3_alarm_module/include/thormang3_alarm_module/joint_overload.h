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

#ifndef JOINT_OVERLOAD_H
#define JOINT_OVERLOAD_H

#include <list>
#include <string>

#include "robotis_device/dynamixel.h"

namespace thormang3
{

class JointOverload
{
public:
  enum
  {
    NORMAL = 0,
    WARNING = 1,
    ERROR = 2,
    EMERGENCY = 3,
  };
  JointOverload(const std::string &model_name, std::string joint_name, double control_cycle);
  void clearOverload();
  void setOverloadLimit(double limit_value) {overload_limit_ratio_ = limit_value / 100;}
  double addPresentCurrent(double value, double &total_overload);
  //bool isOverloadError() {return overload_value_ > overload_limit_ratio_;}
  bool isOverloadError() {return filtered_total_overload_ > overload_limit_ratio_;}
  bool getOverloadError(int &error_status, int &warning_count, int &error_count);

private:
  const double H54P_200_NOMINAL;
  const double H54P_100_NOMINAL;
  const double H42P_020_NOMINAL;
  const std::string H54P_200_MODEL;
  const std::string H54P_100_MODEL;
  const std::string H42P_020_MODEL;
//  const double TOTAL_CHECK_TIME;
  const double PEAK_TIME;
  const double MAX_CURRENT_RATE;
  const double BASE_CURRENT_RATE;

  double calcOverloadFromCurrent(double current_value);
  void checkOverloadError();
  // void calcTotalOverload();

  std::string joint_name_;
  double control_cycle_time_;
  double nominal_current_;
  double overload_limit_ratio_;
  double filtered_total_overload_;
  int warning_count_;
  int error_count_;
  int error_status_;
  bool status_changed_;
};
}
#endif // JOINT_OVERLOAD_H
