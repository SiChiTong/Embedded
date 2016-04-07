/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: motor_config.h
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license. 
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke      2015.10.1    V1.0          creat this file
*
* Description:   
***********************************************************************************************************************/

#ifndef __motor_config_H__
#define __motor_config_H__

#include "main_config.h"
#include "BSP_TOP.h"

/***********************************************************************************************************************
是否种采用电机仿真  使能则不需要实体机器人 只需要和主控电路板连上就可以仿真测试机器人移动
这样可以很方便的联合ROS仿真工具调试
失能0 则把输出映射到实体电机上 需要连接电机
***********************************************************************************************************************/

#define  SYSTEM_SUPPORT_Motor_Simulation  0u

#if  SYSTEM_SUPPORT_Motor_Simulation > 0u
#define Simulation_Max_Angle_Speed  5000   //仿真电机的最大速度限制  °/S
#define Simulation_Motor_Curve      1      //电机仿真曲线 1为线
#endif

#define  Motor_PWM_MAX      5000     //电机PWM周期
#define  Motor_PWM_Limite   4500     //脉宽幅度限制
#define  Motor_Dead_Zone    5        //设置死区范围  pwm输出在该范围内则认为pwm=0  电机失能

/***********************************************************************************************************************
自定义电机参数
***********************************************************************************************************************/
#define  Motor1_IO_Enable  1u        //使能电机x的初始化         
#define  Motor2_IO_Enable  1u              
#define  Motor3_IO_Enable  1u               
#define  Motor_Num  3                //初始化的电机个数  
#define  MOTOR_PID_T        0.02f    //电机PID的时间间隔  单位s
#define  Motor_ENCODER_NUM  1560     //轮子转一圈，编码器得到的计数值

#endif  //#ifndef __motor_config_H__

