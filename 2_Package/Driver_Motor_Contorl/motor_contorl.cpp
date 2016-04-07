/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
*
* FileName: motor_driver.cpp
*
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
* Description:  The file defined a Class for motor PID control
***********************************************************************************************************************/

#include "motor_contorl.h"

MOTOR_PID::MOTOR_PID()
{

		expect_angle_speed=0;
		expect_angle_speed=0;
		expect_unit_encoder=0;
		expect_total_encoder=0;
		mesure_unit_encoder=0;
		mesure_total_encoder=0;
		mesure_angle_speed=0;
		past_angle=0;

    PID.I_Flag=0;
    PID. P1=0;
    PID.I1=0;
    PID.D1=0;
    PID.P2=0;
    PID.I2=0;
    PID.D2=0;
    PID.O_error_now=0;
    PID.O_error_last=0;
    PID.O_error_before_last=0;
    PID.O_error_P=0;
    PID.O_error_I=0;
    PID.O_error_D=0;
    PID.O_Pidout=0;
    PID.I_error_now=0;
    PID.I_error_last=0;
    PID.I_error_before_last=0;
    PID.I_error_P=0;
    PID.I_error_I=0;
    PID.I_error_D=0;
    PID.I_Pidout=0;

#if  SYSTEM_SUPPORT_Motor_Simulation > 0u   //the pid parameters for handsfree simulation motor

//	PID.I_Flag=1;  //PID_Series
//	PID.P1=1;
//	PID.I1=1;
//	PID.D1=0;
//	PID.P2=4.5;
//	PID.I2=0;
//	PID.D2=0;

#endif    //#if  SYSTEM_SUPPORT_Motor_Simulation > 0u              

//the pid parameters for real motor of handsfree robot 3wd mini
#if  SYSTEM_SUPPORT_Motor_Simulation == 0u 

//	PID.I_Flag=1;  //PID_Series
//	PID.P1=1;
//	PID.I1=1;
//	PID.D1=0;
//	PID.P2=25;
//	PID.I2=0;
//	PID.D2=0;

	PID.I_Flag=1;   //PID_Ordinary
	PID.P1=0;
	PID.I1=0;
	PID.D1=0;
	PID.P2=60;
	PID.I2=500;
	PID.D2=0.2;

#endif

}

/***********************************************************************************************************************
* Function:     float MOTOR_PID::PID_Limit(float Data , float Range)
*
* Scope:        private
*
* Description:  data limited
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
float MOTOR_PID::PID_Limit(float Data , float Range)
{
    if( Data > Range ) return Range;
    else  if( Data <-Range ) return -Range;
    else return Data;
}

/***********************************************************************************************************************
* Function:     PID_Ordinary_Cal(....)
*
* Scope:        private
*
* Description:  增量式pid  
*         Outside_Expect 外环的期望值   Outside_Mesure 外环的测量值
*         Inside_Expect   内环的期望值   Inside_Mesure 内环的测量值    Range 输出的最大幅度   -Range ~~ +Range
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/

void MOTOR_PID::PID_Ordinary_Cal(float Outside_Expect , float Outside_Mesure
                             , float Inside_Expect , float Inside_Mesure , float Range)
{

    PID.I_error_before_last = PID.I_error_last;    //速度环
    PID.I_error_last = PID.I_error_now;
    PID.I_error_now = Inside_Expect - Inside_Mesure;

    PID.I_error_P = PID.P2 * PID.I_error_now;
	  PID.I_error_P = PID_Limit( PID.I_error_P , Range ) ;   //限制幅度是防止速度超过最大速度形成累积
	  if(PID.I_Flag == 1 && (Inside_Expect > 1 || Inside_Expect < -1) )  
	  PID.I_error_I += PID.I2*PID.I_error_now * MOTOR_PID_T;
    else  PID.I_error_I = 0;
    PID.I_error_I = PID_Limit( PID.I_error_I , Range) ;   //限制幅度是防止速度超过最大速度形成累积
    PID.I_error_D = PID.D2*(PID.I_error_now-PID.I_error_last) * (1/(float)MOTOR_PID_T);
	  //PID.I_error_D = PID.D2*(PID.I_error_now-2*PID.I_error_last+PID.I_error_before_last) * (1/(float)MOTOR_PID_T);
    PID.I_error_D = PID_Limit( PID.I_error_D , Range ) ;    //限制幅度是防止速度超过最大速度形成累积
	
    PID.I_Pidout = PID.I_error_P +  PID.I_error_I + PID.I_error_D;
	  PID.I_Pidout = PID_Limit( PID.I_Pidout , Range ) ;      //限制幅度是防止速度超过最大速度形成累积

}

/***********************************************************************************************************************
* Function:     PID_Series_Cal(....)
*
* Scope:        private
*
* Description:  位置式串级pid     外环：里程   内环：速度
*         Outside_Expect 外环的期望值   Outside_Mesure 外环的测量值
*         Inside_Expect   内环的期望值   Inside_Mesure 内环的测量值    Range 输出的最大幅度   -Range ~~ +Range
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
void MOTOR_PID::PID_Series_Cal(float Outside_Expect , float Outside_Mesure ,
                           float Inside_Expect , float Inside_Mesure , float Range)
{

    PID.O_error_before_last = PID.O_error_last;         //位置环
    PID.O_error_last = PID.O_error_now;
    PID.O_error_now = Outside_Expect - Outside_Mesure;

    PID.O_error_P = PID.P1 * PID.O_error_now;
    if(PID.I_Flag == 1 && (Inside_Expect > 1 || Inside_Expect < -1)) 	PID.O_error_I += PID.I1 * PID.O_error_now * MOTOR_PID_T;
    else  PID.O_error_I = 0;
    PID.O_error_I = PID_Limit( PID.O_error_I , Range/2 ) ;     //积分限制幅度
    PID.O_error_D = PID.D1*(PID.O_error_now-PID.O_error_last) * (1/(float)MOTOR_PID_T);
    PID.O_Pidout = PID.O_error_P +  PID.O_error_I + PID.O_error_D;      // 外环的输出   作为内环的输入

    PID.I_error_before_last = PID.I_error_last;           //速度环
    PID.I_error_last = PID.I_error_now;
    PID.I_error_now = PID.O_Pidout- Inside_Mesure ;	      // 外环的输出   作为内环的输入

    PID.I_error_P = PID.P2 * PID.I_error_now;
    PID.I_error_P = PID_Limit( PID.I_error_P , Range ) ;   //限制幅度是防止速度超过最大速度形成累积
    PID.I_error_I += PID.I2*PID.I_error_now * MOTOR_PID_T;
    PID.I_error_I = PID_Limit( PID.I_error_I , Range/2) ;   //限制幅度是防止速度超过最大速度形成累积
    PID.I_error_D = PID.D2*(PID.I_error_now-PID.I_error_last) * (1/(float)MOTOR_PID_T);
    PID.I_error_D = PID_Limit( PID.I_error_D , Range ) ;    //限制幅度是防止速度超过最大速度形成累积

    PID.I_Pidout = PID.I_error_P +  PID.I_error_I + PID.I_error_D;
    PID.I_Pidout = PID_Limit( PID.I_Pidout , Range ) ;      //限制幅度是防止速度超过最大速度形成累积

}

/***********************************************************************************************************************
* Function:     void MOTOR_PID::Motor_Control(float *Count , float MAX_PWM_OUT)
*
* Scope:        public
*
* Description:  电机控制 循环调用  PID.I_Pidout内环输出即为输出的PWM值  正负代表方向
*    float *Count  编码器的计数值的地址
*    float expect_angle_speed  期望角速度
*    float MAX_PWM_OUT   PWM最大的输出值  用于限制幅度
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
***********************************************************************************************************************/
float MOTOR_PID::Motor_Control(float E_Speed , float Unit_Count )
{
       
    /********** 接口变量赋值**********/
    expect_angle_speed = E_Speed;
#if  SYSTEM_SUPPORT_Motor_Simulation > 0u   //采用电机仿真
#if Simulation_Motor_Curve == 1     //仿真使用线性关系
    mesure_unit_encoder = ((float)Simulation_Max_Angle_Speed/(float)360)
            *( PID.I_Pidout/Motor_PWM_MAX ) *(float)Motor_ENCODER_NUM*(float)MOTOR_PID_T;
#endif
#else                //控制实物电机
    mesure_unit_encoder =  Unit_Count;
#endif

    // 一个周期内的编码器的期望计数
    expect_unit_encoder = ( expect_angle_speed / 360 ) * Motor_ENCODER_NUM * MOTOR_PID_T;
    expect_total_encoder += expect_unit_encoder  ;        //期望的总编码器计数  定时清0
    mesure_total_encoder += mesure_unit_encoder ;
		//记录走过的总角度,正负代表方向,用于解算坐标,由解算坐标的程序清0
		past_angle += (mesure_unit_encoder/Motor_ENCODER_NUM)*360;
		//测量的速度   单位化  度每秒
		mesure_angle_speed  = mesure_unit_encoder * 360 / ( Motor_ENCODER_NUM*MOTOR_PID_T);
		//测定的总编码器计数  定时清0
       	
		PID_Ordinary_Cal(expect_total_encoder , mesure_total_encoder
						, expect_unit_encoder , mesure_unit_encoder , Motor_PWM_MAX);		
	
    return  PID.I_Pidout;
}

