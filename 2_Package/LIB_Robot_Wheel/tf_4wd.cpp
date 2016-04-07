/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: tf_3wd.cpp
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
* Description:  本文件定义了三轮式 万向轮机器人 底盘运动的坐标变换数学函数  即三个轮的速度 	机器人坐标  世界坐标
*               两两之间的变换公式   该文件在LIB_Robot_3WD package里面  依赖 LIB_Motor_Contorl package
***********************************************************************************************************************/

#include "tf_4wd.h"

#if ROBOT_WHEEL_MODEL == 4


TF_4WD tf_4w;

TF_4WD::TF_4WD()
{

}

void TF_4WD::Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r)   
{


}


void TF_4WD::Motor_To_Robot_tf(float *Motor , float *Robot , float robot_r)   
{


}


void TF_4WD::Global_To_Robot_tf( float *Global , float *Robot , float R_theta)   
{


}




void TF_4WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)   
{


}



void TF_4WD::Global_To_Motor_tf(float *Global , float *Motor ,float R_theta ,float robot_r)   
{


}


void TF_4WD::Motor_To_Global_tf(float *Motor , float *Global ,float R_theta ,float robot_r)   
{

}

#endif  //#if ROBOT_WHEEL_MODEL == 4

