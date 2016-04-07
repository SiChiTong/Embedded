/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robot_wheel_top.c
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
* Description:  该文件在LIB_Robot_3WD package里面  依赖 Motor_Contorl package
***********************************************************************************************************************/

#include "robot_wheel_top.h"

ROBOT_WHEEL Hands_Free_3WD;
ROBOT_WHEEL::ROBOT_WHEEL()
{
    Communication_Quality=0;  // 0~100  if<30 Communication_Quality is bad and Control Disable  , otherwise control enable
    L_Filter = 0.4;    //速度低通滤波系数
    Control_EN=1;	     //默认使能机器人控制

    Global_Speed_Renew=0;
    Robot_Speed_Renew=0;
    Motor_Speed_Renew=0;

    Expect_Global_Speed.X=0;
    Expect_Global_Speed.Y=0;
    Expect_Global_Speed.Z=0;

    Expect_Robot_Speed.X=0;
    Expect_Robot_Speed.Y=0;
    Expect_Robot_Speed.Z=0;

    Expect_Motor_Angle_Speed.M1=0;
    Expect_Motor_Angle_Speed.M2=0;
    Expect_Motor_Angle_Speed.M3=0;

    Mesure_Global_Coordinate.X=0;
    Mesure_Global_Coordinate.Y=0;
    Mesure_Global_Coordinate.Z=0;

    Mesure_Robot_Coordinate.X=0;
    Mesure_Robot_Coordinate.Y=0;
    Mesure_Robot_Coordinate.Z=0;

    Mesure_Global_Speed.X=0;
    Mesure_Global_Speed.Y=0;
    Mesure_Global_Speed.Z=0;

    Mesure_Robot_Speed.X =0;
    Mesure_Robot_Speed.Y =0;
    Mesure_Robot_Speed.Z =0;

    Mesure_Motor_Speed.M1=0;
    Mesure_Motor_Speed.M2=0;
    Mesure_Motor_Speed.M3=0;

    Mesure_Motor_Distance.M1=0;
    Mesure_Motor_Distance.M2=0;
    Mesure_Motor_Distance.M3=0;

}

/***********************************************************************************************************************
* Function:     void ROBOT_WHEEL::Robot_3WD_Init(void)   
*
* Scope:        public
*
* Description:  robot init
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
void ROBOT_WHEEL::Robot_Wheel_Top_Init(void)          
{

}

/***********************************************************************************************************************
* Function:     void ROBOT_WHEEL::Robot_Wheel_Top_Call(void)   
*
* Scope:        public
*
* Description:   坐标解算 不需要PID那么大的频率 可以设置为PID频率的 2分之1 或者 4分之1 都行
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
void ROBOT_WHEEL::Robot_Wheel_Top_Call(void)          
{
    Chassis_Control();            // robot Chassis Control         底盘控制
    Coordinate_Calculation();     // robot Coordinate Calculation  坐标解算   
}


/***********************************************************************************************************************
* Function:     void ROBOT_WHEEL::Chassis_Control(void)           
*
* Scope:        private
*
* Description:  底盘控制 Chassis Control call ， 100hz is adviced for   
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.10.1   creat
* by   mawenke   2015.1.23   update
***********************************************************************************************************************/
void ROBOT_WHEEL::Chassis_Control(void)    
{ 	 

		Communication_Quality--;	             //Communication_Quality reduce 1 every call
		if(Communication_Quality >= 100){
			Communication_Quality=100;
		}
		else if(Communication_Quality <= 5){
			Communication_Quality = 5;
		}
		if( Global_Speed_Renew ==1 ){
			Global_Speed_Renew = 0;
			Communication_Quality += 30;        //Communication_Quality add 30
			Global_Speed_Set( (float*)&Expect_Global_Speed , (float*)&Expect_Motor_Angle_Speed ,
															 Mesure_Global_Coordinate.Z);
		}
		else if( Robot_Speed_Renew ==1 ){
			Robot_Speed_Renew = 0;
			Communication_Quality += 30;
			Robot_Speed_Set((float*)&Expect_Robot_Speed , (float*)&Expect_Motor_Angle_Speed);
		}
		else if( Motor_Speed_Renew ==1 ){
			Motor_Speed_Renew = 0;
			Communication_Quality += 30;
		}
		if(Communication_Quality <=30 ){       //if Communication_Quality is too low
	    Control_EN = 0; //motor control disable
		} 
	  else if(Communication_Quality >=70 ){  //if Communication_Quality is too low
	    Control_EN = 1; //motor control disable
		}
							
    if(Control_EN == 0)                        //motor control disable
    {
			motor_top.Expect_Angle_Speed_M1=0;     //set motor Expect Speed 0
			motor_top.Expect_Angle_Speed_M2=0;
			#if ROBOT_WHEEL_MODEL > 2     
			motor_top.Expect_Angle_Speed_M3=0;
			#endif
			#if ROBOT_WHEEL_MODEL > 3         
			motor_top.Expect_Angle_Speed_M4=0;
			#endif
    }
    else if(Control_EN == 1)
    {
        motor_top.Enable_M1=1 ;                //motor enable
        motor_top.Enable_M2=1 ;
				#if ROBOT_WHEEL_MODEL > 2  
				motor_top.Enable_M3=1 ;
				#endif
				#if ROBOT_WHEEL_MODEL > 3  
				motor_top.Enable_M4=1 ;
				#endif

				motor_top.Expect_Angle_Speed_M1 = (1-L_Filter) *motor_top.Expect_Angle_Speed_M1 +
				L_Filter* Expect_Motor_Angle_Speed.M1 ;                  //Speed Low-pass filter
				motor_top.Expect_Angle_Speed_M2 = (1-L_Filter) *motor_top.Expect_Angle_Speed_M2 +
				L_Filter* Expect_Motor_Angle_Speed.M2 ;

				#if ROBOT_WHEEL_MODEL > 2  
				motor_top.Expect_Angle_Speed_M3 = (1-L_Filter) *motor_top.Expect_Angle_Speed_M3 +
				L_Filter* Expect_Motor_Angle_Speed.M3 ;
				#endif
				#if ROBOT_WHEEL_MODEL > 3  
				motor_top.Expect_Angle_Speed_M4 = (1-L_Filter) *motor_top.Expect_Angle_Speed_M4 +
				L_Filter* Expect_Motor_Angle_Speed.M4 ;
				#endif
    }

}

/***********************************************************************************************************************
* Function:     void ROBOT_WHEEL::Coordinate_Calculation_3WD(void)
*
* Scope:        private
*
* Description:  3轮式底盘坐标解算  编码器的数据分析 坐标解算  解算出x，y，w,给上位机
*               建议100HZ运行
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
void ROBOT_WHEEL::Coordinate_Calculation(void)
{

		static ROBOT_MOTOR D_Len_M_Filter;
	
    D_Len_M_Filter.M1 = ( motor_top.motor1.past_angle/360 ) * 2 * PI_ * WHEEL_R;
		motor_top.motor1.past_angle=0;       //清0记录的总角度
    Mesure_Motor_Distance.M1 += D_Len_M_Filter.M1 ;

		
    D_Len_M_Filter.M2 = ( motor_top.motor2.past_angle/360 ) * 2 * PI_ * WHEEL_R;
		motor_top.motor2.past_angle=0;
    Mesure_Motor_Distance.M2 += D_Len_M_Filter.M2 ;

		#if ROBOT_WHEEL_MODEL > 2  
		D_Len_M_Filter.M3 = ( motor_top.motor3.past_angle/360 ) * 2 * PI_ * WHEEL_R;   
		motor_top.motor3.past_angle=0;
		Mesure_Motor_Distance.M3 += D_Len_M_Filter.M3 ;
		#endif
	
		#if ROBOT_WHEEL_MODEL > 3  
		D_Len_M_Filter.M4 = ( motor_top.motor4.past_angle/360 ) * 2 * PI_ * WHEEL_R;   
		motor_top.motor4.past_angle=0; 
		Mesure_Motor_Distance.M4 += D_Len_M_Filter.M4 ;
		#endif
		
#if Coordinate_Calculation_IMU == 1 
    //    Mesure_Global_Coordinate.Z=imu_arithmetic_model.Fus_Angle.yaw ;   //如果IMU在位
    //    Mesure_Robot_Coordinate.Z=imu_arithmetic_model.Fus_Angle.yaw ;
#endif

    //need time stm32F1 330us  stm32F4+NOFPU 64~80us   stm32F4+FPU 8~16us
    Get_Global_Coordinate( (float *)&D_Len_M_Filter , (float*)&Mesure_Global_Coordinate);
    //need time stm32F1 20us  stm32F4+FPU 0~2us
    Get_Robot_Coordinate( (float *)&D_Len_M_Filter , (float*)&Mesure_Robot_Coordinate);

}



