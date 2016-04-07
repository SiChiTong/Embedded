/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: sbus_ppm_config.h
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1   V1.0           creat this file
* Description:
*
***********************************************************************************************************************/

#ifndef __sbus_ppm_config_H__
#define __sbus_ppm_config_H__

#include "main_config.h"
#include "BSP_TOP.h"

#ifndef LIB_Robot_Wheel
#error "!!! the LIB_Sbus_Ppm package must dependent on LIB_Robot_Wheel package at least"
#endif

#ifdef LIB_Robot_Wheel
#include "robot_wheel_top.h"
#endif
#ifdef    Driver_Servo
#include "servo_top.h"
#endif

#endif    //#ifndef __sbus_ppm_config_H__

