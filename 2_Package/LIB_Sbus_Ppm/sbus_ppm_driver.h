#ifndef __sbus_ppm_driver_H__
#define __sbus_ppm_driver_H__

#include "sbus_ppm_config.h"

#define   Sbus_USART_Init		HF_Usart_Init(USART2,100000,1)			   //Sbus USART Initialize
#define   Sbus_D_TIME		    3000			      //us   Sbus message Time interval

class SBUS
{
public:
    SBUS();
    void Sbus_Init(void);
    void Sbus_IRQ(unsigned int Data);         //put this function in serial port interrupt func
    void Sbus_Call(void);

    unsigned char   Sbus_RX_Update;
    unsigned short int  Sbus_Channel[16];     //temp sbus decode channel data
    unsigned char   Sbus_Flag;                //flag of sbus remote control signal

private:
    void Sbus_Data_Anl(void);
    float Sbus_Frequency;
    float Sbus_Mes_Count;
    unsigned char   Sbus_RX_Buffer[25];
    float Sbus_Frequency_Measure(void);
};
extern SBUS sbus ;

class PPM
{
public:
    PPM();
    void PPM_IRQ(float pwm_value);
    unsigned short int 	PPM_RC_Value[16];

private:

};

extern PPM ppm ;

#endif

