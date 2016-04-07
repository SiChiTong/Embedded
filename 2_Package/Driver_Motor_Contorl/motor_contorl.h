#ifndef __motor_driver_H__
#define __motor_driver_H__

#include "motor_config.h"

typedef struct PID_Data{
    unsigned char I_Flag;         // PID积分使能标志       油门大于一定值才开始积分
    float P1;                     // 1是外环参数  2是内环参数
    float I1;
    float D1;
    float P2;
    float I2;
    float D2;
    float O_error_now;           // O是外环参数  I是内环参数
    float O_error_last;
    float O_error_before_last;
    float O_error_P;
    float O_error_I;
    float O_error_D;
    float O_Pidout;
    float I_error_now;
    float I_error_last;
    float I_error_before_last;
    float I_error_P;
    float I_error_I;
    float I_error_D;
    float I_Pidout;
}PID_Data;

class MOTOR_PID
{

public:
    MOTOR_PID(); //构造函数对PID参数都赋值 为0
    float  expect_angle_speed;	       //输入目的角速度 单位数据
    float  expect_unit_encoder;        //目的角速度转化成的编码器计数，
    float  expect_total_encoder;       //期望的总编码器计数  定时清0
    float  mesure_unit_encoder;        //编码器测定的计数		    用于PID控制
    float  mesure_total_encoder;       //测定的总编码器计数  定时清0   用于PID控制
    float  mesure_angle_speed;	       //测量的的角速度  单位数据 回馈给上位机   用于PID控制
    float  past_angle;                 //记录走过的总角度,正负代表方向,用于解算坐标,由解算坐标的程序清0 
    //电机控制 循环调用  PID.I_Pidout内环输出即为输出的PWM值  正负代表方向
    //Input:   float E_Speed  期望角速度    float *Count  编码器的计数值的地址
    //float MAX_PWM_OUT   PWM最大的输出值
    //Output :  PID的最终输出  也是电机的目标PWM值
    float Motor_Control(float E_Speed , float Unit_Count);
private:

    PID_Data PID;           //串级位置式PID 参数表
    float PID_Limit(float Data , float Range);
    void PID_Ordinary_Cal(float Outside_Expect , float Outside_Mesure
                          , float Inside_Expect , float Inside_Mesure , float Range);
    void PID_Series_Cal(float Outside_Expect , float Outside_Mesure
                        , float Inside_Expect , float Inside_Mesure , float Range);

};

#endif

