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

#ifndef __robot_wheel_config_H__
#define __robot_wheel_config_H__

#include "main_config.h"
#include "BSP_TOP.h"

#ifndef LIB_Robot_Wheel
#error "!!! the LIB_Robot_Wheel package must dependent on Driver_Motor_Contorl package at least"
#endif

#ifdef    Driver_Motor_Contorl
#include "motor_top.h"
#endif

#ifdef    Driver_IMU
#define  Coordinate_Calculation_IMU  1    //是否使用IMU辅助解算机器人坐标 0不使用  1使用
#include "imu_top.h"
#endif


#define  ROBOT_WHEEL_MODEL    3     //2代表2轮机器人  3代表3轮式万向轮机器人  4代表4个麦克纳姆轮全向机器人
#define  PI_   3.141592653f

#if ROBOT_WHEEL_MODEL == 2

// 适用车型1： 后面两个动力轮，前面一个脚轮或者钢珠的车，例如turtlebot
// 适用车型2： 2轮平衡车
// 适用车型3： 4驱车,实际上和2轮车一个模式
#define  WHEEL_R  0.0325f   //轮子的半径
#define  ROBOT_R  0.161f	 //两轮中心间距的1/2

//坐标校准系数，因为理论解算和实际运动的坐标有所误差，则需要以下实测参数校准，1代表完全信任理论值
#define  Coordinate_Calibration_X  1.00f
#define  Coordinate_Calibration_Y  1.00f
#define  Coordinate_Calibration_Z  1.00f

#endif


#if ROBOT_WHEEL_MODEL == 3

//3轮式万向轮机器人 俯视图逆时针方向 分别为 1号 2号 3号电机   1号电机 为正前方   
//侧视图  电机顺时针转为正
//俯视图  机器人逆时针转为正      
#define  WHEEL_R  0.029f    //万向轮的半径
#define  ROBOT_R  0.161f		//机器人半径  即轮子中心到机器人中心的距离

//坐标校准系数，因为理论解算和实际运动的坐标有所误差，则需要以下实测参数校准，1代表完全信任理论值
#define  Coordinate_Calibration_X  1.00f
#define  Coordinate_Calibration_Y  1.00f
#define  Coordinate_Calibration_Z  1.00f

#endif


#if ROBOT_WHEEL_MODEL == 4


#endif


#endif    //#ifndef __robot_wheel_config_H__

