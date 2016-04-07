/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: sbus_ppm_driver.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1    V1.0          creat this file
*
* Description:  this file include decoding functions of sbus and ppm remote signal
*               one sbus message continued 3ms  and two sbus message's interval is 4ms，so 7ms a frame
*               and we can decoding it base this information
*
*
***********************************************************************************************************************/

#include "sbus_ppm_driver.h"

SBUS sbus ;
PPM ppm ;

//extern "C" void USART2_IRQHandler(void)
//{
//    unsigned char data;
//	
//#if SYSTEM_SUPPORT_OS == 1
//    OSIntEnter();
//#endif

//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//      data = USART_ReceiveData(USART2);
//      sbus.Sbus_IRQ(data);    
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);  // clear interrupt flag
//    }

//#if SYSTEM_SUPPORT_OS == 1
//    OSIntExit();
//#endif	
//}


/***********************************************************************************************************************
***                                                                                                                  ***
***                                           sbus Decoding func as below                                            ***
***                                                                                                                  ***
***********************************************************************************************************************/

SBUS::SBUS()
{
    Sbus_Frequency=0;
    Sbus_Mes_Count=0;
    Sbus_RX_Buffer[0]=0;
}

void SBUS::Sbus_Call(void)
{
	static unsigned char Call_i;
	Call_i++;
	if(Call_i >= 5)
	{
			Call_i=0;		

	}

}


/***********************************************************************************************************************
* Function:    static void BSP_Data_init(void)
*
* Scope:       private  
*
* Description: 
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(2 us)
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/
void SBUS::Sbus_Data_Anl(void)
{
    Sbus_Channel[0] = (Sbus_RX_Buffer[ 2] & 0x7) << 8 | Sbus_RX_Buffer[ 1];
    Sbus_Channel[8] = (Sbus_RX_Buffer[ 13] & 0x7) << 8 | Sbus_RX_Buffer[ 12];
    Sbus_Channel[1] = (Sbus_RX_Buffer[ 3] & 0x3F) << 5 | Sbus_RX_Buffer[ 2] >> 3;
    Sbus_Channel[9] = (Sbus_RX_Buffer[ 14] & 0x3F) << 5 | Sbus_RX_Buffer[ 13] >> 3;
    Sbus_Channel[2] = (Sbus_RX_Buffer[ 5] & 0x1) << 10 | Sbus_RX_Buffer[ 4] << 2 | Sbus_RX_Buffer[ 3] >> 6;
    Sbus_Channel[10] = (Sbus_RX_Buffer[ 16] & 0x1) << 10 | Sbus_RX_Buffer[ 15] << 2 | Sbus_RX_Buffer[ 14] >> 6;
    Sbus_Channel[3] = (Sbus_RX_Buffer[ 6] & 0xF) << 7 | Sbus_RX_Buffer[ 5] >> 1;
    Sbus_Channel[11] = (Sbus_RX_Buffer[ 17] & 0xF) << 7 | Sbus_RX_Buffer[ 16] >> 1;
    Sbus_Channel[4] = (Sbus_RX_Buffer[ 7] & 0x7F) << 4 | Sbus_RX_Buffer[ 6] >> 4;
    Sbus_Channel[12] = (Sbus_RX_Buffer[ 18] & 0x7F) << 4 | Sbus_RX_Buffer[ 17] >> 4;
    Sbus_Channel[5] = (Sbus_RX_Buffer[ 9] & 0x3) << 9 | Sbus_RX_Buffer[ 8] << 1 | Sbus_RX_Buffer[ 7] >> 7;
    Sbus_Channel[13] = (Sbus_RX_Buffer[ 20] & 0x3) << 9 | Sbus_RX_Buffer[ 19] << 1 | Sbus_RX_Buffer[ 18] >> 7;
    Sbus_Channel[6] = (Sbus_RX_Buffer[ 10] & 0x1F) << 6 | Sbus_RX_Buffer[ 9] >> 2;
    Sbus_Channel[14] = (Sbus_RX_Buffer[ 21] & 0x1F) << 6 | Sbus_RX_Buffer[ 20] >> 2;
    Sbus_Channel[7] = Sbus_RX_Buffer[ 11] << 3 | Sbus_RX_Buffer[ 10] >> 5;
    Sbus_Channel[15] = Sbus_RX_Buffer[ 22] << 3 | Sbus_RX_Buffer[ 21] >> 5;
    Sbus_Flag = Sbus_RX_Buffer[ 23];
		if ( Sbus_Flag == 0){
			if( Sbus_Channel[5] >= 1000 )
			{
				#ifdef LIB_Robot_Wheel
				Hands_Free_3WD.Expect_Robot_Speed.X = (-(Sbus_Channel[1] - 992)*0.001);
				Hands_Free_3WD.Expect_Robot_Speed.Y = (-(Sbus_Channel[0] - 992)*0.001);
				Hands_Free_3WD.Expect_Robot_Speed.Z = (-(Sbus_Channel[3] - 992)*0.001)*3;
				Hands_Free_3WD.Robot_Speed_Renew=1;
				#endif
			}			
			else if( Sbus_Channel[5] <= 200 )
			{
				#ifdef Driver_Servo
				hf_head.Expect_Head_State.Pitch =(Sbus_Channel[1] - 992)*0.05;
				hf_head.Expect_Head_State.Yaw  = -(Sbus_Channel[0] - 992)*0.05;
				hf_head.Set_Head_State_Renew = 1;
				#endif
			}
			else
			{
			
			}
			
		}
}

/***********************************************************************************************************************
* Function:    void SBUS::Sbus_IRQ(unsigned int Data)    put this function in serial port interrupt func
*
* Scope:       public  
*
* Description: 
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(3 us)
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/
void SBUS::Sbus_IRQ(unsigned int Data)    
{
    static float last_time=0,this_time=0,d_time;
    static unsigned char Sbus_Bufi,Sbus_Mes_i;

    this_time=GET_System_time();
    d_time = this_time - last_time ;
    last_time = this_time ;
    Sbus_Bufi++;
    if( d_time > 	Sbus_D_TIME)          //if right , This bytes is the first of sbus message
    {
        Sbus_Bufi=0;
    }
    if( Sbus_Bufi < 25){

        Sbus_RX_Buffer[Sbus_Bufi]  = Data ;
        if( Sbus_Bufi == 24 && ( Sbus_RX_Buffer[0] == 0x0f ) )
        {
            Sbus_Mes_Count++;
            Sbus_Frequency=Sbus_Frequency_Measure();   //Measure the sbus message's Frequency , about 100hz
            Sbus_RX_Update = 1 ;                       //renew a sbus message and wait main func to use
            Sbus_Mes_i++;
            if(Sbus_Mes_i >= 20)
            {
                Sbus_Mes_i=0;
                Sbus_Data_Anl();
            }
        }
    }
    else {
        Sbus_Bufi=0;
    }
}

/***********************************************************************************************************************
* Function:    static void BSP_Data_init(void)
*
* Scope:       private  
*
* Description: Measure the sbus message's Frequency , about 100hz
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/
float SBUS::Sbus_Frequency_Measure(void) 
{
    static float last_time=0,this_time=0,d_time_new,d_time;
    this_time=GET_System_time();
    d_time_new = this_time - last_time ;
    last_time = this_time ;
    d_time = 0.9f*d_time+0.1f*d_time_new  ;   //Low pass filter
    return (1000000/d_time);
}	


void SBUS::Sbus_Init(void)
{
    Sbus_USART_Init;
}

/***********************************************************************************************************************
***                                                                                                                  ***
***                                            ppm Decoding func as below                                            ***
***                                                                                                                  ***
***********************************************************************************************************************/



PPM::PPM()
{


}

/***********************************************************************************************************************
* Function:      void PPM::PPM_IRQ(float pwm_value)
*
* Scope:         public
*
* Description:   pwm_value is the Sample of ppm's Pulse width , unit:us
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
void PPM::PPM_IRQ(float pwm_value)
{

    static uint8_t RC_Ch = 0;
    RC_Ch++;
    if( pwm_value > 3000)
    {
        RC_Ch = 0;         //if right , This value is the first of ppm message
    }
    PPM_RC_Value[RC_Ch] = pwm_value;

}

