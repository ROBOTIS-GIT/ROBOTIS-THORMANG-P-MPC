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
 *  kinematics_dynamics_define.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

namespace thormang3
{

#define MAX_JOINT_ID    (31) // 29 + 2
#define ALL_JOINT_ID    (46)

#define MAX_ARM_ID      (7)
#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

#define ID_HEAD_END     (29)
#define ID_COB          (44)
#define ID_TORSO        (27)

#define ID_R_ARM_START  (1)
#define ID_L_ARM_START  (2)
#define ID_R_ARM_END    (35)
#define ID_L_ARM_END    (34)

#define ID_R_LEG_START  (15)
#define ID_L_LEG_START  (16)
#define ID_R_LEG_END    (45)
#define ID_L_LEG_END    (46)

#define ID_R_LEG_FT     (37)
#define ID_L_LEG_FT     (36)

#define ID_BASE         (0)
#define ID_PELVIS       (44)

#define ID_PELVIS_POS_X (38)
#define ID_PELVIS_POS_Y (39)
#define ID_PELVIS_POS_Z (40)
#define ID_PELVIS_ROT_X (41)
#define ID_PELVIS_ROT_Y (42)
#define ID_PELVIS_ROT_Z (43)


#define GRAVITY_ACCELERATION (9.8)

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_ */
