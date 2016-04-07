#ifndef __robot_3wd_H__
#define __robot_3wd_H__

#include "robot_wheel_config.h"

#if  ROBOT_WHEEL_MODEL==2
#include "tf_2wd.h"
#endif		
#if  ROBOT_WHEEL_MODEL==3
#include "tf_3wd.h"	
#endif		
#if  ROBOT_WHEEL_MODEL==4
#include "tf_4wd.h"	
#endif

#if  ROBOT_WHEEL_MODEL==2
class ROBOT_WHEEL:public TF_2WD		
#endif		
#if  ROBOT_WHEEL_MODEL==3
class ROBOT_WHEEL:public TF_3WD		
#endif		
#if  ROBOT_WHEEL_MODEL==4
class ROBOT_WHEEL:public TF_4WD		
#endif		
{
public:

    unsigned char Communication_Quality;  // 0~100  if<30 Communication_Quality is bad and Control Disable  , otherwise control enable
    unsigned char Control_EN;             // Whether or not enable motor control
    float L_Filter;                       //速度低通滤波系数 取值范围 0--1  数值越小 低通效果越大
    unsigned char Global_Speed_Renew,Robot_Speed_Renew,Motor_Speed_Renew;

		typedef struct ROBOT_MOTOR{
				float  M1;
				float  M2;
				float  M3;} ROBOT_MOTOR;

		typedef struct ROBOT_XYZ{
				float  X;
				float  Y;
				float  Z;} ROBOT_XYZ;

    //机器人的速度向量 单位 X Y : M/S  X为前后坐标 前正后负  Y为左右坐标 右正左负     Z: 弧度/S  逆时针为正
    ROBOT_XYZ  Expect_Global_Speed , Expect_Robot_Speed;
    ROBOT_MOTOR      Expect_Motor_Angle_Speed;
    ROBOT_XYZ  Mesure_Global_Coordinate , Mesure_Robot_Coordinate;  //测量的坐标 X Y单位为M  Z的单位为弧度
    ROBOT_XYZ  Mesure_Global_Speed , Mesure_Robot_Speed;
    ROBOT_MOTOR Mesure_Motor_Speed,Mesure_Motor_Distance;                 //电机速度 单位是角度/S  测量的里程值 单位是M

    ROBOT_WHEEL();
    void Robot_Wheel_Top_Init(void);                          // 机器人初始化
    void Robot_Wheel_Top_Call(void);

private:
    void Chassis_Control(void);           //robot底盘控制  循环调用
    void Coordinate_Calculation(void);    //robot坐标解算  循环调用
};

extern ROBOT_WHEEL Hands_Free_3WD;

#endif  //#ifndef __robot_3wd_H__

