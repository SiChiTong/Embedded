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
#include "servo_top.h"

SERVO_TOP hf_head;

#define PITCH_OFFSET  512
#define PITCH_Range  30
#define YAW_OFFSET    512
#define YAW_Range  80

//extern "C"  void USART3_IRQHandler(void)
//{
//    unsigned char data;

//#if SYSTEM_SUPPORT_OS == 1
//    OSIntEnter();
//#endif

//    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        data=USART_ReceiveData(USART3);
//        digital_servo.gbpRxInterruptBuffer[(digital_servo.gbRxBufferWritePointer++)] = data;
//        USART_ClearITPendingBit(USART3,USART_IT_RXNE);  //clear interrupt flag
//    }

//#if SYSTEM_SUPPORT_OS == 1
//    OSIntExit();
//#endif
//}

SERVO_TOP::SERVO_TOP()
{
	Set_Head_State_Renew=0;
	Read_Head_State_Renew=0;
	Expect_Head_State.Pitch=0;
	Expect_Head_State.Yaw=0;
	Mesure_Head_State.Pitch=0;
	Mesure_Head_State.Yaw=0;
}

void SERVO_TOP::Init(void)
{
		AX_Servo_Init();
}

void SERVO_TOP::Head_Top_Call(void)
{
  static unsigned char First_Call=0;
	if(First_Call==0)
	{
		First_Call=1;
		getServoConnective();
		axSendPosition(1, PITCH_OFFSET, 0x50);
		axSendPosition(2, YAW_OFFSET , 0x50);
  }
	else
	{
		if(Set_Head_State_Renew == 1 )
		{	
			Set_Head_State_Renew = 0;			
			if(Expect_Head_State.Pitch >= PITCH_Range) Expect_Head_State.Pitch=PITCH_Range;
			if(Expect_Head_State.Pitch <= -PITCH_Range) Expect_Head_State.Pitch=-PITCH_Range;
			if(Expect_Head_State.Yaw >= YAW_Range) Expect_Head_State.Pitch=YAW_Range;
			if(Expect_Head_State.Yaw <= -YAW_Range) Expect_Head_State.Pitch=-YAW_Range;		
			Set_State(Expect_Head_State.Pitch , Expect_Head_State.Yaw);		
		}	
		if(Read_Head_State_Renew == 1 )
		{
			Read_Head_State_Renew = 0;
			Get_State((float*)&Mesure_Head_State.Pitch , (float*)&Mesure_Head_State.Yaw);			
		}						
  }
}

void SERVO_TOP::Set_State(float Pitch , float Yaw)
{
  unsigned short int set_pitch, set_yaw;	
	set_pitch=-(Pitch/300)*1024+PITCH_OFFSET; 
	set_yaw=(Yaw/300)*1024+YAW_OFFSET; 
	axSendPosition(1, set_pitch , 0x50);
	axSendPosition(2, set_yaw , 0x50);	
}

void SERVO_TOP::Get_State(float *Pitch , float *Yaw)
{
//	unsigned short int read_pitch , read_yaw;
//  read_pitch = axReadPosition(1);
//	read_yaw = axReadPosition(2);
}
