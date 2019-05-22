^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_base_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2018-03-27)
------------------
* modified build option and dependencies configuration
* modified cmake & package setting for yaml-cpp
* changed depend pkg for catkin dependencies and package.xml format to v2
* changed include path for dependencies and license to Apache v2
* Contributors: SCH, Pyo

0.1.3 (2017-05-23)
------------------
* updated cmake file for ros install
* Contributors: SCH

0.1.2 (2017-04-24)
------------------
* added done msg to base_module
* - fixed package dependencies
* - optimized cpu usage by spin loop (by astumpf)
* wholebody modified
* - reduce CPU consumption
* wholebody inipose added
* update thormang3_head_control_module : scan lidar using range
* change init pose
* Contributors: Jay Song, Zerom, Kayman

0.1.1 (2016-08-19)
------------------
* none

0.1.0 (2016-08-18)
------------------
* first public release for Kinetic
* modified package information for release
* thormang3_base_module : rename RobotisState -> BaseModuleState
* applying coding style
* file name changed.
* modified coding style..
* ROS c++ coding stylie is applying, first thormang3_kinematics_dynamics complete
* fixed high CPU consumption due to busy waits
* added Singleton template
* Contributors: Alexander Stumpf, Zerom, SCH, Pyo
