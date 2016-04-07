/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: spi.c
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
*        STM32F4--------------
*        SPI1 --- PA5,6,7   SPI2 --- PB13,14,15
***********************************************************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif 

#include "spi.h"
#include "nvic.h"	 

/***********************************************************************************************************************
* Function:     HF_SPI_Init(SPI_TypeDef *SPIx , unsigned char GPIO_AF) 
*
* Scope:        public
*
* Description:  Initialize serial external interface of SPI, config SPI working on master mode	


*
* Arguments:
*
* Return:
*
* Cpu_Time:  
*
* History:
***********************************************************************************************************************/
void HF_SPI_Init(SPI_TypeDef *SPIx , unsigned char GPIO_AF)
{	 
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;   	//initialize SPI structure
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //Multiplexing
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //push-pull output
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;    //pull down
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    if(SPIx == SPI1)
    {
        RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1, ENABLE );
        
        if(GPIO_AF == 0)
        {
            RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOA , ENABLE );
            GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
            GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
            GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
        }
        else if(GPIO_AF == 1)
        {
            
        }
        //	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//reset SPI1
        //	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//stop reset SPI1
    }
    else if(SPIx == SPI2)
    {
        RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2, ENABLE );
        
        if(GPIO_AF == 0)
        {
            RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOB , ENABLE );
            GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
            GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
        }
        else if(GPIO_AF == 1)
        {
            
        }                                                       
        //	 RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);  //reset SPI2
        //	 RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE); //stop reset SPI2
    }
    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为两线全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                    //设置SPI为主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                //SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                      //串行时钟在不操作时，时钟为低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	                      //第一个时钟沿开始采样数据
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                        //NSS信号由软件（使用SSI位）管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //SPI波特率预分频值为8
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	                          //CRC值计算的多项式
    SPI_Init(SPIx, &SPI_InitStructure);                                 //根据SPI_InitStruct中指定的参数初始化外设SPI1寄存器
    SPI_Cmd(SPIx, ENABLE);                                              //使能SPI1外设
    
    HF_SPI_ReadWriteByte(SPIx,0xff);                                    //启动传输
} 	 

/***********************************************************************************************************************
* Function:     unsigned char HF_SPI_ReadWriteByte(SPI_TypeDef *SPIx , unsigned char TxData) 
*
* Scope:        public
*
* Description:  SPI读写数据函数


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
unsigned char HF_SPI_ReadWriteByte(SPI_TypeDef *SPIx , unsigned char TxData)                    
{
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);     //发送缓存标志位为空
    SPI_I2S_SendData(SPIx, TxData);                                     //通过外设SPI发送一个数据
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);    //接收缓存标志位不为空
    return SPI_I2S_ReceiveData(SPIx);                                   //通过SPI返回接收数据
}	

#ifdef __cplusplus
}
#endif 

