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

#include "tf_3wd.h"

#if ROBOT_WHEEL_MODEL == 3 


TF_3WD tf_3wd;

TF_3WD::TF_3WD()
{

}

/***********************************************************************************************************************
* Function:     void TF_3WD::Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r)  
*
* Scope:        public
*
* Description:  机器人坐标到三个轮子的变换
*
* Arguments:
* Motor   为输出的三个电机的线速度v1 v2 v3  
* Robot   为机器人坐标的速度向量xyw 
* robot_r 机器人的半径  
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/

void TF_3WD::Robot_To_Motor_tf(float *Robot , float *Motor , float robot_r)   
{
    *Motor = 0 * (*Robot) + 1 * (*(Robot+1)) + robot_r * (*(Robot+2)) ;
    *(Motor + 1) =  -0.866025f * (*Robot) - 0.5f * (*(Robot+1)) + robot_r * (*(Robot+2)) ;
    *(Motor + 2) =  0.866025f * (*Robot) - 0.5f * (*(Robot+1)) + robot_r * (*(Robot+2)) ;
}

/***********************************************************************************************************************
* Function:     void TF_3WD::Motor_To_Robot_tf(float *Motor , float *Global , float robot_r)  
*
* Scope:        public
*
* Description:  三个轮子到机器人坐标的变换
*
* Arguments:
* Motor   为输出的三个电机的线速度v1 v2 v3  
* Robot   为机器人坐标的速度向量xyw 
* robot_r 机器人的半径
* Return:
*
* Cpu_Time:  stm32F1 20us   stm32F4+FPU 0~2us
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_3WD::Motor_To_Robot_tf(float *Motor , float *Robot , float robot_r)   
{
    *Robot = 0 * (*Motor) + -0.57735f * (*(Motor+1)) + 0.57735f * (*(Motor+2)) ;
    *(Robot + 1) =  0.666667f* (*Motor)  -0.333333f * (*(Motor+1)) + -0.333333f * (*(Motor+2)) ;
    *(Robot + 2) =  ( 0.333333f/ (float)robot_r ) * ( (*Motor) + (*(Motor+1)) +  (*(Motor+2)) ) ;
}


/***********************************************************************************************************************
* Function:     void TF_3WD::Global_To_Robot_tf( float *Global , float *Robot , float R_theta) 
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
void TF_3WD::Global_To_Robot_tf( float *Global , float *Robot , float R_theta)   
{
    *Robot =  (float)( cos(R_theta) * (*Global) + sin(R_theta) * (*(Global+1)) + 0 * (*(Global+2)) );
    *(Robot + 1) = (float)( -sin(R_theta) * (*Global) + cos(R_theta) * (*(Global+1)) + 0 * (*(Global+2)) );
    *(Robot + 2) =   0 * (*Global) + 0 * (*(Global+1)) + 1 * (*(Global+2)) ;
}


/***********************************************************************************************************************
* Function:     void TF_3WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta) 
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
void TF_3WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)   
{
    *Global = cos(R_theta)*(*Robot) - sin(R_theta) * (*(Robot+1)) +  0 * (*(Robot+2)) ;
    *(Global + 1) =  sin(R_theta) * (*Robot) + cos(R_theta) * (*(Robot+1)) + 0 * (*(Robot+2)) ;
    *(Global + 2) =  0 * (*Robot) + 0 * (*(Robot+1)) + 1 * (*(Robot+2)) ;
}


/***********************************************************************************************************************
* Function:     void TF_3WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)  
*
* Scope:        public
*
* Description:  世界坐标到三个轮子的变换
*
* Arguments:
* Global为Global坐标的速度向量xyw
* Robot为Robot坐标的速度向量xyw 
* R_theta 为机器人当前方向  即坐标值的Z
* robot_r机器人的半径
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_3WD::Global_To_Motor_tf(float *Global , float *Motor ,float R_theta ,float robot_r)   
{
    *Motor =  -sin(R_theta) * (*Global) + cos(R_theta) * (*(Global+1)) + robot_r * (*(Global+2)) ;
    *(Motor + 1) =  -sin(PI_/3 - R_theta) * (*Global) - cos(PI_/3 - R_theta) * (*(Global+1)) + robot_r * (*(Global+2)) ;
    *(Motor + 2) =  sin(PI_/3 + R_theta) * (*Global) - cos(PI_/3 + R_theta) * (*(Global+1)) + robot_r * (*(Global+2)) ;
}

/***********************************************************************************************************************
* Function:     void TF_3WD::Robot_To_Global_tf(float *Robot , float *Global ,float R_theta)  
*
* Scope:        public
*
* Description:  三个轮子线速度或者线里程微分到世界坐标的变换 
                                函数  void Motor_To_Global_tf(float *Motor , float *Global ,float R_theta)
                                三个万向轮式的机器人坐标到全局坐标的转化

                                输入 机器人坐标的三个变量    全局坐标的三个变量的保存地址   机器人的和全局坐标的夹角
                                Gvx Gvy Gvz  v1 v2 v3
                                Global 为全局坐标的速度x ，y ，z
                                *Motor  为三个电机的速度 v1 ， v2 ，v3

                                或者

 Global 为全局坐标的微分  dx ，dy ，dz
 *Motor 为三个电机的里程微分 d s1 ， d s2 ，d s3
*
* Arguments:
* Global为Global坐标的速度向量xyw
* Robot为Robot坐标的速度向量xyw 
* R_theta 为机器人当前方向  即坐标值的Z
* robot_r机器人的半径
* Return:
*
* Cpu_Time:  stm32F1 330us   stm32F4+NOFPU 64~80us    stm32F4+FPU 8~16us
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void TF_3WD::Motor_To_Global_tf(float *Motor , float *Global ,float R_theta ,float robot_r)   
{
    *Global = -0.666667f * sin(R_theta)*(*Motor) + (-0.577350f*cos(R_theta) + 0.333333f*sin(R_theta))*(*(Motor+1)) + (0.577350f*cos(R_theta) + 0.333333f*sin(R_theta))*(*(Motor+2)) ;
    *(Global + 1) =  0.666667f * cos(R_theta)*(*Motor) + (-0.577350f*sin(R_theta) - 0.333333f*cos(R_theta))*(*(Motor+1)) + (0.577350f*sin(R_theta) - 0.333333f*cos(R_theta))*(*(Motor+2)) ;
    *(Global + 2) =  ( 1 / (float)(3*robot_r) ) * ( (*Motor) + (*(Motor+1)) +(*(Motor+2)) ) ;
}


/***********************************************************************************************************************
***                                                                                                                  ***
***                                          Code of External Interface                                              ***
***                                                                                                                  ***
***********************************************************************************************************************/
//set global speed
void TF_3WD::Global_Speed_Set(float* Global_Speed , float* Expect_Motor_Speed , 
                              float Global_Coordinat_Z)
{
    static float Motor_Line_Speed[3] ;
    Global_To_Motor_tf( Global_Speed , Motor_Line_Speed , Global_Coordinat_Z ,ROBOT_R);
    *Expect_Motor_Speed  = ( Motor_Line_Speed[0] / WHEEL_R ) * ( 180/PI_ );
    *(Expect_Motor_Speed+1)  = ( Motor_Line_Speed[1] / WHEEL_R ) * ( 180/PI_ );
    *(Expect_Motor_Speed+2)  = ( Motor_Line_Speed[2] / WHEEL_R ) * ( 180/PI_ );
}

//set Robot speed
void TF_3WD::Robot_Speed_Set(float* Robot_Speed , float* Expect_Motor_Speed)  
{
    static float Motor_Line_Speed[3] ;
    Robot_To_Motor_tf(Robot_Speed , Motor_Line_Speed , ROBOT_R) ;
    *Expect_Motor_Speed  = ( Motor_Line_Speed[0] / WHEEL_R ) * ( 180/PI_ );
    *(Expect_Motor_Speed+1)  = ( Motor_Line_Speed[1] / WHEEL_R ) * ( 180/PI_ );
    *(Expect_Motor_Speed+2)  = ( Motor_Line_Speed[2] / WHEEL_R ) * ( 180/PI_ );
}


//get Robot Global Coordinate
void TF_3WD::Get_Global_Coordinate(float* D_Len_M_Filter , float* Mesure_Global_Coordinate)  
{
    static float D_Global_Coordinate[3] ;
    Motor_To_Global_tf(D_Len_M_Filter , D_Global_Coordinate , *(Mesure_Global_Coordinate+2) , ROBOT_R);
    *Mesure_Global_Coordinate += D_Global_Coordinate[0]*Coordinate_Calibration_X;
    *(Mesure_Global_Coordinate+1) += D_Global_Coordinate[1]*Coordinate_Calibration_Y;
    *(Mesure_Global_Coordinate+2) += D_Global_Coordinate[2]*Coordinate_Calibration_Z;
}	

//get Robot Robot Coordinate
void TF_3WD::Get_Robot_Coordinate(float* D_Len_M_Filter , float* Mesure_Robot_Coordinate)  
{
    static float D_Robot_Coordinate[3] ;
    Motor_To_Robot_tf( D_Len_M_Filter , D_Robot_Coordinate , ROBOT_R) ;
    *Mesure_Robot_Coordinate += D_Robot_Coordinate[0]*Coordinate_Calibration_X;
    *(Mesure_Robot_Coordinate+1) += D_Robot_Coordinate[1]*Coordinate_Calibration_Y;
    *(Mesure_Robot_Coordinate+2) += D_Robot_Coordinate[2]*Coordinate_Calibration_Z;
}	

#endif     //#if ROBOT_WHEEL_MODEL == 3 
