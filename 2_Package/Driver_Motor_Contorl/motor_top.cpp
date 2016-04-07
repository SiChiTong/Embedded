/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: motor_top.c
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
* Description: The file defined a Class for motor control ， you can use it Conveniently and modify a litter 
*              for a different circuit board
***********************************************************************************************************************/

#include "motor_top.h"

MOTOR_TOP motor_top;

MOTOR_TOP::MOTOR_TOP()
{
    Enable_M1=0;
    Enable_M2=0;
    Enable_M3=0;
    Expect_Angle_Speed_M1=0;
    Expect_Angle_Speed_M2=0;
    Expect_Angle_Speed_M3=0;
}

/***********************************************************************************************************************
* Function:     void Motor_Top_Init(void)
*
* Scope:        private
*
* Description:  The underlying interface of motor control，you need to modify it for a different circuit board
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/

void MOTOR_TOP::Motor_Set_PWM(unsigned char Motor_ID , float Pid_out)
{

    if( Motor_ID ==1 ){
        if( Enable_M1 !=1 )
        {
            MOTOREN1_OFF();      //motor disable
            Pwm_Set_TIM1_CCR1(0);
            Pwm_Set_TIM1_CCR2(0);
            return;
        }
        else if( ( Pid_out < Motor_Dead_Zone ) && (Pid_out > -Motor_Dead_Zone) )   // enter dead line
        {
            MOTOREN1_OFF();     //motor disable
            Pwm_Set_TIM1_CCR1(0);
            Pwm_Set_TIM1_CCR2(0);
            return;
        }
        else if( Pid_out > 0) {
            MOTOREN1_ON();      //motor enable
            Pwm_Set_TIM1_CCR1((unsigned short int)Pid_out);
            Pwm_Set_TIM1_CCR2(0);
            return;
        }
        else{
            MOTOREN1_ON();      //motor enable
            Pwm_Set_TIM1_CCR1(0);
            Pwm_Set_TIM1_CCR2((unsigned short int)-Pid_out);
            return;
        }
    }
    else if(Motor_ID == 2 ){
        if( Enable_M2 !=1 )
        {
            MOTOREN2_OFF();      //motor disable
            Pwm_Set_TIM1_CCR3(0);
            Pwm_Set_TIM1_CCR4(0);
            return;
        }
        else if( ( Pid_out < Motor_Dead_Zone ) && (Pid_out > -Motor_Dead_Zone) )   // enter dead line
        {
            MOTOREN2_OFF();      //motor disable
            Pwm_Set_TIM1_CCR3(0);
            Pwm_Set_TIM1_CCR4(0);
            return;
        }
        else if( Pid_out > 0) {
            MOTOREN2_ON();       //motor enable
            Pwm_Set_TIM1_CCR3((unsigned short int)Pid_out);
            Pwm_Set_TIM1_CCR4(0);
            return;
        }
        else{
            MOTOREN2_ON();       //motor enable
            Pwm_Set_TIM1_CCR3(0);
            Pwm_Set_TIM1_CCR4((unsigned short int)-Pid_out);
            return;
        }
    }
    else if(Motor_ID ==3 ){
        if( Enable_M3 !=1 )
        {
            MOTOREN3_OFF();      //motor disable
            Pwm_Set_TIM12_CCR1(0);
            Pwm_Set_TIM12_CCR2(0);
            return;
        }
        else if( ( Pid_out < Motor_Dead_Zone ) && (Pid_out > - Motor_Dead_Zone) )   // enter dead line
        {
            MOTOREN3_OFF();      //motor disable
            Pwm_Set_TIM12_CCR1(0);
            Pwm_Set_TIM12_CCR2(0);
            return;
        }
        else if( Pid_out > 0) {
            MOTOREN3_ON();       //motor enable
            Pwm_Set_TIM12_CCR1((unsigned short int)Pid_out);
            Pwm_Set_TIM12_CCR2(0);
            return;
        }
        else{
            MOTOREN3_ON();       //motor enable
            Pwm_Set_TIM12_CCR1(0);
            Pwm_Set_TIM12_CCR2((unsigned short int)-Pid_out);
            return;
        }
    }

}	

/***********************************************************************************************************************
* Function:     void Motor_Top_Init(void)
*
* Scope:        public
*
* Description:  motor control interface init
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/
void MOTOR_TOP::Motor_Top_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

#if Hardware_Platform == 1  //if the MCU is STM32F1 Series

#endif

#if Hardware_Platform == 4  //if the MCU is STM32F4 Series

#if  SYSTEM_SUPPORT_Motor_Simulation != 1u   //if not use motor_simulation
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

		if ( Motor1_IO_Enable >= 0u )
		{
			RCC_AHB1PeriphClockCmd(RCC_MOTOREN1 , ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_MOTOREN1_Pin ;
			GPIO_Init(GPIO_MOTOREN1 , &GPIO_InitStructure);
			MOTOREN1_OFF() ;
			HF_Encoder_Init(TIM2,0);	              //encoder init for PID speed control
		}
		if ( Motor1_IO_Enable >= 0u )
		{
			RCC_AHB1PeriphClockCmd(RCC_MOTOREN2 , ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_MOTOREN2_Pin ;
			GPIO_Init(GPIO_MOTOREN2 , &GPIO_InitStructure);
			MOTOREN2_OFF() ;
			HF_Encoder_Init(TIM3,1);
		}
		if ( Motor1_IO_Enable >= 0u )
		{
			RCC_AHB1PeriphClockCmd(RCC_MOTOREN3 , ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_MOTOREN3_Pin ;
			GPIO_Init(GPIO_MOTOREN3 , &GPIO_InitStructure);
			MOTOREN3_OFF() ;
			HF_Encoder_Init(TIM4,1);
		}
    
    /*****here we can init ADC for collect motor current electricity ******/

		
    //TIM1 motor pwm frequency  = (168M/4) / Motor_PWM_MAX  = 16.8K
    HF_PwmOut_Init(TIM1 , 2-1 , Motor_PWM_MAX , 1);
    //TIM12 motor pwm frequency = (84M/2) / Motor_PWM_MAX  = 16.8K
    HF_PwmOut_Init(TIM12 , 0 , Motor_PWM_MAX , 0);
#endif   //#if  SYSTEM_SUPPORT_Motor_Simulation != 1u   //if not use motor_simulation

#endif   //if the MCU is STM32F4 Series

    Enable_M1 = 1 ;   //enable motor control
    Enable_M2 = 1 ;
    Enable_M3 = 1 ;
}	

/***********************************************************************************************************************
* Function:     void Motor_Top_Call(void)
*
* Scope:        public
*
* Description:  motor control function ，cycle call it ，and the call frequency is PID frequency
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

void MOTOR_TOP::Motor_Top_Call(void)
{

    /* here comments code is for motor test ，you do not include·it in normal unless you need to test the motor*/

    //    static unsigned short int i;
    //    i++;
    //    if(i<1000)
    //    {
    //        Expect_Angle_Speed_M1=2000;
    //        Expect_Angle_Speed_M2=2000;
    //        Expect_Angle_Speed_M3=2000;
    //    }
    //    else if (i>1000 && i<1500)
    //    {
    //        Expect_Angle_Speed_M1=0;
    //        Expect_Angle_Speed_M2=0;
    //        Expect_Angle_Speed_M3=0;
    //    }
    //    else if (i>1500 && i<2500)
    //    {
    //        Expect_Angle_Speed_M1=-1000;
    //        Expect_Angle_Speed_M2=-1000;
    //        Expect_Angle_Speed_M3=-1000;
    //    }
    //    else if(i>2500) i=0;

    // *Count--the address of encoder D_value   MAX_PWM_OUT--PWM max value
    // need time    stm32f4+nofpu 25us  stm32f4+fpu 5us

    Motor_Set_PWM( 1 , motor1.Motor_Control(Expect_Angle_Speed_M1 , Get_Encoder_CNT_TIM2() ) );
    Motor_Set_PWM( 2 , motor2.Motor_Control(Expect_Angle_Speed_M2 , Get_Encoder_CNT_TIM3() ) );
    Motor_Set_PWM( 3 , motor3.Motor_Control(Expect_Angle_Speed_M3 , Get_Encoder_CNT_TIM4() ) );
}


