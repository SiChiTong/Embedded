/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robot_wheel_config.h
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

#ifndef __Servo_config_H__
#define __Servo_config_H__

#include "main_config.h"
#include "BSP_TOP.h"

//数字舵机把位置分成1024等分,取值 0~0X3FF,逆时针为正
//HEAD  Pitch>0 抬头   ; Pitch<0 低头 ; YAW>0 左转  ; YAW<0 右转

#define SYSTEM_SUPPORT_Servo_Digital   1u     //是否编译数字舵机控制代码
#define SYSTEM_SUPPORT_Servo_Analog    1u     //是否编译模拟舵机控制代码

#define SYSTEM_SUPPORT_Servo_Debug     0u      //是否使能printf 打印调试信息

#endif   //#ifndef __Servo_config_H__

