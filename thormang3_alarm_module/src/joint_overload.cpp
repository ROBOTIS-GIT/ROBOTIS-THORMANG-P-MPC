#include "thormang3_alarm_module/joint_overload.h"
#include <iostream>
#include <cmath>

namespace thormang3
{

JointOverload::JointOverload(const std::string &model_name, std::string joint_name, double control_cycle)
  : H54P_200_NOMINAL(7.58),
    H54P_100_NOMINAL(5.3),
    H42P_020_NOMINAL(1.5),
    H54P_200_MODEL("H54P-200"),
    H54P_100_MODEL("H54P-100"),
    H42P_020_MODEL("H42P-020"),
    PEAK_TIME(0.7),
    MAX_CURRENT_RATE(3.0),
    BASE_CURRENT_RATE(1.0),
    joint_name_(joint_name),
    control_cycle_time_(control_cycle),
    filtered_total_overload_(0.0),
    warning_count_(0),
    error_count_(0),
    error_status_(NORMAL),
    status_changed_(false)
{
  //check nominal current
  if(model_name.find(H54P_200_MODEL) != std::string::npos)
    nominal_current_ = H54P_200_NOMINAL;
  else if(model_name.find(H54P_100_MODEL) != std::string::npos)
    nominal_current_ = H54P_100_NOMINAL;
  else if(model_name.find(H42P_020_MODEL) != std::string::npos)
    nominal_current_ = H42P_020_NOMINAL;
  else {
    nominal_current_ = H42P_020_NOMINAL;
  }
}

void JointOverload::clearOverload()
{
  filtered_total_overload_ = 0.0;
  warning_count_ = 0;
  error_count_ = 0;
  error_status_ = NORMAL;
  status_changed_ = false;
}

double JointOverload::addPresentCurrent(double value, double &total_overload)
{
  // convert current to overload_rate
  double abs_value = fabs(value);
  double present_overload = calcOverloadFromCurrent(abs_value);

  // add present overload
  filtered_total_overload_ = filtered_total_overload_ * 0.999 + present_overload;

  //total_overload = overload_value_;
  total_overload = filtered_total_overload_;

  // check overload state
  checkOverloadError();

  return present_overload;
}

double JointOverload::calcOverloadFromCurrent(double current_value)
{
  // method 1
  // double overload_rate = current_value / nominal_current_ - 1.0;
  // if(overload_rate < 0.2)
  //   return 0.0;

  // double load_limit = ((2.01 - 1.4) / (0.3 - 2.0) * (overload_rate - 2.0) + 1.4);
  // return (overload_rate * control_cycle_time_) / load_limit;

  // method 2
  double overload_l2 = (current_value / nominal_current_) * (current_value / nominal_current_) - BASE_CURRENT_RATE * BASE_CURRENT_RATE;
  if(overload_l2 <= 0.44)
    return 0.0;

  double l2t_limit = (MAX_CURRENT_RATE * MAX_CURRENT_RATE - BASE_CURRENT_RATE * BASE_CURRENT_RATE) * PEAK_TIME;   // l^2 * t
  return (overload_l2 * control_cycle_time_) / l2t_limit;
}

void JointOverload::checkOverloadError()
{
  double overload_rate = filtered_total_overload_ / overload_limit_ratio_ * 100.0;

  if(overload_rate <= 90)
  {
    if(error_status_ != NORMAL)
    {
      error_status_ = NORMAL;
      status_changed_ = true;
    }
    else
    {
      status_changed_ = false;
    }
  }
  else if(overload_rate <= 100)
  {
    if(error_status_ < WARNING)
      warning_count_ += 1;

    if(error_status_ != WARNING)
    {
      error_status_ = WARNING;
      status_changed_ = true;
    }
    else
    {
      status_changed_ = false;
    }
  }
  else if(overload_rate <= 200)
  {
    if(error_status_ < ERROR)
      error_count_ += 1;

    if(error_status_ != ERROR)
    {
      error_status_ = ERROR;
      status_changed_ = true;
    }
    else
    {
      status_changed_ = false;
    }
  }
  else
  {
    if(error_status_ != EMERGENCY)
    {
      error_status_ = EMERGENCY;
      status_changed_ = true;
    }
    else
    {
      status_changed_ = false;
    }
  }
}

bool JointOverload::getOverloadError(int &error_status, int &warning_count, int &error_count)
{
  warning_count = warning_count_;
  error_count = error_count_;
  error_status = error_status_;

  return status_changed_;
}

}
