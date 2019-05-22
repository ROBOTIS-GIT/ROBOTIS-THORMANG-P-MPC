################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_p_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller
  thormang3_feet_ft_module
  thormang3_base_module
  thormang3_action_module
  thormang3_head_control_module
  thormang3_manipulation_module
  thormang3_walking_module
  thormang3_gripper_module
  thormang3_tuning_module
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    robotis_controller
    thormang3_feet_ft_module
    thormang3_base_module
    thormang3_action_module
    thormang3_head_control_module
    thormang3_manipulation_module
    thormang3_walking_module
    thormang3_gripper_module
    thormang3_tuning_module
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(thormang3_p_manager src/thormang3_p_manager.cpp)

add_dependencies(thormang3_p_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thormang3_p_manager ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS thormang3_p_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
