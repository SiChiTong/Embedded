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
* Description:  本文件定义了2轮差速底盘运动的坐标变换数学函数  即轮的速度 	机器人坐标  世界坐标
*               两两之间的变换公式
*               适用车型1： 后面两个动力轮，前面一个脚轮或者钢珠的车，例如turtlebot
*               适用车型2： 2轮平衡车
*               适用车型3： 4驱车实际上和2轮车一个模式

* !! 这种车型  速度只有前后和旋转，也就是X和Z   不能全向走
* !! 定义左边为电机1  右边为电机2
***********************************************************************************************************************/

#include "tf_2wd.h"

#if ROBOT_WHEEL_MODEL == 2


TF_2WD tf_2w;

TF_2WD::TF_2WD()
{

}

/***********************************************************************************************************************
* Function:     void TF_3WD::Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r)  
*
* Scope:        public
*
* Description:  机器人坐标到2个轮子的变换
*
* Arguments:
* Robot   为机器人坐标的速度向量XYZ   Y=0  
* Motor   为输出的两个电机的线速度v1  v2   
* robot_r 机器人的半径  
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_2WD::Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r)   
{
    *Motor = -1 * (*Robot) + 0 * (*(Robot+1)) + robot_r * (*(Robot+2)) ;
    *(Motor + 1) =  1 * (*Robot) + 0 * (*(Robot+1)) + robot_r * (*(Robot+2)) ;
}


/***********************************************************************************************************************
* Function:     void TF_3WD::Motor_To_Robot_tf(float *Motor , float *Global , float robot_r)  
*
* Scope:        public
*
* Description:  三个轮子到机器人坐标的变换
*
* Arguments:
* Motor   为输出的三个电机的线速度v1 v2   
* Robot   为机器人坐标的速度向量XYZ   Y=0 
* robot_r 机器人的半径
* Return:
*
* Cpu_Time:  stm32F1 20us   stm32F4+FPU 0~2us
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_2WD::Motor_To_Robot_tf(float *Motor , float *Robot , float robot_r)   
{
    *Robot = -0.5f * (*Motor) + 0.5f* (*(Motor+1));
    *(Robot + 1) =  0 * (*Motor)  + 0 * (*(Motor+1));
    *(Robot + 2) =  ( 0.5f/ (float)robot_r ) * ( (*Motor) + (*(Motor+1)) );
}



/***********************************************************************************************************************
* Function:     void TF_2WD::Global_To_Robot_tf( float *Global , float *Robot , float R_theta)    
*
* Scope:        public
*
* Description:  世界坐标到机器人坐标的变换
*
* Arguments:
* Global为Global坐标的速度向量xyw
* Robot为Robot坐标的速度向量xyw 
* R_theta 为机器人当前方向  即坐标值的Z
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_2WD::Global_To_Robot_tf( float *Global , float *Robot , float R_theta)   
{
    *Robot =  (float)( cos(R_theta) * (*Global) + sin(R_theta) * (*(Global+1)) + 0 * (*(Global+2)) );
    *(Robot + 1) = (float)( -sin(R_theta) * (*Global) + cos(R_theta) * (*(Global+1)) + 0 * (*(Global+2)) );
    *(Robot + 2) =   0 * (*Global) + 0 * (*(Global+1)) + 1 * (*(Global+2)) ;
}


/***********************************************************************************************************************
* Function:     void TF_2WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)  
*
* Scope:        public
*
* Description:  机器人坐标到世界坐标的变换
*
* Arguments:
* Global为Global坐标的速度向量xyw
* Robot为Robot坐标的速度向量xyw 
* R_theta 为机器人当前方向  即坐标值的Z
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_2WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)   
{
    *Global = cos(R_theta)*(*Robot) - sin(R_theta) * (*(Robot+1)) +  0 * (*(Robot+2)) ;
    *(Global + 1) =  sin(R_theta) * (*Robot) + cos(R_theta) * (*(Robot+1))  + 0 * (*(Robot+2)) ;
    *(Global + 2) =  0 * (*Robot) + 0 * (*(Robot+1)) + 1 * (*(Robot+2)) ;
}



void TF_2WD::Global_To_Motor_tf(float *Global , float *Motor ,float R_theta ,float robot_r)   
{


}


void TF_2WD::Motor_To_Global_tf(float *Motor , float *Global ,float R_theta ,float robot_r)   
{
    *Global = (-cos(R_theta)/2)*(*Motor) + (cos(R_theta)/2)*(*(Motor+1)) ;
    *(Global + 1) =  (-sin(R_theta)/2) * (*Motor) +  (sin(R_theta)/2) * (*(Motor+1)) ;
    *(Global + 2) =  (0.5f/robot_r) * (*Motor) + (0.5f/robot_r) * (*(Motor+1)) ;
}



/***********************************************************************************************************************
***                                                                                                                  ***
***                                          Code of External Interface                                              ***
***                                                                                                                  ***
***********************************************************************************************************************/
//set global speed
//2WD 无法走全局速度
void TF_2WD::Global_Speed_Set(float* Global_Speed , float* Expect_Motor_Speed , 
                              float Global_Coordinat_Z)
{

	
}


//set Robot speed
void TF_2WD::Robot_Speed_Set(float* Robot_Speed , float* Expect_Motor_Speed)  
{
    static float Motor_Line_Speed[3] ;
    Robot_To_Motor_tf(Robot_Speed , Motor_Line_Speed , ROBOT_R) ;
    *Expect_Motor_Speed  = ( Motor_Line_Speed[0] / WHEEL_R ) * ( 180/PI_ );
    *(Expect_Motor_Speed+1)  = ( Motor_Line_Speed[1] / WHEEL_R ) * ( 180/PI_ );
}

//get Robot Global Coordinate
void TF_2WD::Get_Global_Coordinate(float* D_Len_M_Filter , float* Mesure_Global_Coordinate)  
{
    static float D_Global_Coordinate[3] ;
    Motor_To_Global_tf(D_Len_M_Filter , D_Global_Coordinate , *(Mesure_Global_Coordinate+2) , ROBOT_R);
    *Mesure_Global_Coordinate += D_Global_Coordinate[0]*Coordinate_Calibration_X;
    *(Mesure_Global_Coordinate+1) += D_Global_Coordinate[1]*Coordinate_Calibration_Y;
    *(Mesure_Global_Coordinate+2) += D_Global_Coordinate[2]*Coordinate_Calibration_Z;
}	

//get Robot Robot Coordinate
void TF_2WD::Get_Robot_Coordinate(float* D_Len_M_Filter , float* Mesure_Robot_Coordinate)  
{
    static float D_Robot_Coordinate[3] ;
    Motor_To_Robot_tf( D_Len_M_Filter , D_Robot_Coordinate , ROBOT_R) ;
    *Mesure_Robot_Coordinate += D_Robot_Coordinate[0]*Coordinate_Calibration_X;
    *(Mesure_Robot_Coordinate+1) += D_Robot_Coordinate[1]*Coordinate_Calibration_Y;
    *(Mesure_Robot_Coordinate+2) += D_Robot_Coordinate[2]*Coordinate_Calibration_Z;
}	

#endif   //#if ROBOT_WHEEL_MODEL == 2
