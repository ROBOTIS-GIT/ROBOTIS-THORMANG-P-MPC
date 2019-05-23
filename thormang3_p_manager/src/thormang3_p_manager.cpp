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
 * thormang3_manager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: Zerom
 */

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "thormang3_feet_ft_module/feet_force_torque_sensor_module.h"

/* Motion Module Header */
#include "thormang3_base_module/base_module.h"
#include "thormang3_action_module/action_module.h"
#include "thormang3_head_control_module/head_control_module.h"
#include "thormang3_manipulation_module/manipulation_module.h"
#include "thormang3_walking_module/walking_module.h"
#include "thormang3_gripper_module/gripper_module.h"
#include "thormang3_tuning_module/tuning_module.h"

using namespace thormang3;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "THORMANG3_P_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    /* Add Sensor Module */
    controller->addSensorModule((robotis_framework::SensorModule*)FeetForceTorqueSensor::getInstance());

    /* Add Motion Module */
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ActionModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ManipulationModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)GripperModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)HeadControlModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)OnlineWalkingModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)TuningModule::getInstance());

    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
