/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: encoder.c
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
*
* Description: 
*       STM32F4--------------
*       occupy hardware source : timer2 timer3 timer4
***********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif 

#include "encoder.h"
#include "nvic.h"

ENCODER_Data encoder_data_r;

#define Encoder_IRQ_MDE_EN   0

/***********************************************************************************************************************
* Function:      void HF_Encoder_Init(TIM_TypeDef* TIMx)
*
* Scope:         public
*
* Description:   Initialize encoder mode, input parameter  TIM2 TIM3 TIM4
*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
***********************************************************************************************************************/
void HF_Encoder_Init(TIM_TypeDef* TIMx , unsigned char GPIO_AF)	
{
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    
    if( TIMx == TIM2){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
        
        if(GPIO_AF == 0){
            //---------------------------------------------------------------TIM2 CHI1 CHI2---PA0 PA1;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            
        }
#if Encoder_IRQ_MDE_EN==1
        TIM2_NVIC_Configuration();    //enable interrupt
#endif
    }
    
    if( TIMx == TIM3){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
        
        if(GPIO_AF == 0){
            //---------------------------------------------------------------TIM3 CHI1 CHI2---PA6 PA7;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            //---------------------------------------------------------------TIM3 CHI1 CHI2---PB4 PB5;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_TIM3);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_TIM3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 2){
            //---------------------------------------------------------------TIM3 CHI1 CHI2---PC6 PC7;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
#if Encoder_IRQ_MDE_EN == 1
        TIM3_NVIC_Configuration();
#endif
    }
    
    if( TIMx == TIM4){
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
        
        if(GPIO_AF == 0){
            //---------------------------------------------------------------TIM4 CHI1 CHI2---PB6 PB7;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,  GPIO_AF_TIM4);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,  GPIO_AF_TIM4);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else if(GPIO_AF == 1){
            //---------------------------------------------------------------TIM4 CHI1 CHI2---PD12 PD13;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
            GPIO_PinAFConfig(GPIOD, GPIO_PinSource12,  GPIO_AF_TIM4);
            GPIO_PinAFConfig(GPIOD, GPIO_PinSource13,  GPIO_AF_TIM4);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
        }
#if Encoder_IRQ_MDE_EN == 1
        TIM4_NVIC_Configuration();
#endif
    }

#if Encoder_IRQ_MDE_EN == 1
    //TIM_DeInit(TIMx);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = 4-1 ;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIMx, ENABLE);
#endif		

#if Encoder_IRQ_MDE_EN == 0
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 1;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
    TIM_Cmd(TIMx, ENABLE);
    TIMx->CNT = 0x7fff;
#endif
    encoder_data_r.TIM2_Count =0;
    encoder_data_r.TIM3_Count =0;
    encoder_data_r.TIM4_Count =0;
}

/************************************************************************************************************************
***                                                                                                                   ***
***                                          Interrupt Function                                                       ***
***                                                                                                                   ***
*************************************************************************************************************************/

float Get_Encoder_CNT_TIM2(void)
{
    float cnt;
#if Encoder_IRQ_MDE_EN == 1
    cnt=encoder_data_r.TIM2_Count;
    encoder_data_r.TIM2_Count=0;
#endif
#if Encoder_IRQ_MDE_EN == 0
    encoder_data_r.TIM2_Count  = (float)((unsigned short int)0x7fff) - (float)((unsigned short int)(TIM2->CNT)) ;  //! (float) is must
    cnt=encoder_data_r.TIM2_Count;
    TIM2->CNT = 0x7fff;
#endif
    return cnt;
}

float Get_Encoder_CNT_TIM3(void)
{
    float cnt;
#if Encoder_IRQ_MDE_EN == 1
    cnt=encoder_data_r.TIM3_Count;
    encoder_data_r.TIM3_Count=0;
#endif
#if Encoder_IRQ_MDE_EN == 0
    encoder_data_r.TIM3_Count  = (float)((unsigned short int)0x7fff) - (float)((unsigned short int)(TIM3->CNT)) ;
    cnt=encoder_data_r.TIM3_Count;
    TIM3->CNT = 0x7fff;
#endif
    return cnt;
}

float Get_Encoder_CNT_TIM4(void)
{
    float cnt;
#if Encoder_IRQ_MDE_EN == 1
    cnt=encoder_data_r.TIM4_Count;
    encoder_data_r.TIM4_Count=0;
#endif
#if Encoder_IRQ_MDE_EN == 0
    encoder_data_r.TIM4_Count  = (float)((unsigned short int)0x7fff) - (float)((unsigned short int)(TIM4->CNT)) ;
    cnt=encoder_data_r.TIM4_Count;
    TIM4->CNT = 0x7fff;
#endif
    return cnt;
}

#if Encoder_IRQ_MDE_EN == 1

void TIM2_IRQHandler(void)	                         
{ 
    if( TIM_GetCounter(TIM2) == 0 ){
        encoder_data_r.TIM2_Count--;
    }
    else{
        encoder_data_r.TIM2_Count++;
    }
    TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);     // clear interrupt flag
}

void TIM3_IRQHandler(void)	                          
{ 
    if( TIM_GetCounter(TIM3) == 0 ){
        encoder_data_r.TIM3_Count--;
    }
    else{
        encoder_data_r.TIM3_Count++;
    }
    TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);     // clear interrupt flag
}

void TIM4_IRQHandler(void)	                          
{ 
    if( TIM_GetCounter(TIM4) == 0 ){
        encoder_data_r.TIM4_Count--;
    }
    else{
        encoder_data_r.TIM4_Count++;
    }
    TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);     // clear interrupt flag
}

#endif

#ifdef __cplusplus
}
#endif 

