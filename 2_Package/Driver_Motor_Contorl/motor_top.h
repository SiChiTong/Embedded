#ifndef __motor_top_H__
#define __motor_top_H__

#include "motor_config.h"
#include "motor_contorl.h"

/*******************MOTOR1************************/	
#define RCC_MOTOREN1            RCC_AHB1Periph_GPIOE
#define GPIO_MOTOREN1           GPIOE
#define GPIO_MOTOREN1_Pin       GPIO_Pin_8
#define MOTOREN1_OFF()          GPIO_ResetBits(GPIO_MOTOREN1 , GPIO_MOTOREN1_Pin)
#define MOTOREN1_ON()           GPIO_SetBits(GPIO_MOTOREN1 , GPIO_MOTOREN1_Pin)
/*********************************************/	

/*******************MOTOREN2************************/	
#define RCC_MOTOREN2            RCC_AHB1Periph_GPIOE
#define GPIO_MOTOREN2           GPIOE
#define GPIO_MOTOREN2_Pin       GPIO_Pin_10
#define MOTOREN2_OFF()          GPIO_ResetBits(GPIO_MOTOREN2 , GPIO_MOTOREN2_Pin)
#define MOTOREN2_ON()           GPIO_SetBits(GPIO_MOTOREN2 , GPIO_MOTOREN2_Pin)
/*********************************************/		

/*******************MOTOREN2************************/	
#define RCC_MOTOREN3            RCC_AHB1Periph_GPIOE
#define GPIO_MOTOREN3           GPIOE
#define GPIO_MOTOREN3_Pin       GPIO_Pin_12
#define MOTOREN3_OFF()          GPIO_ResetBits(GPIO_MOTOREN3 , GPIO_MOTOREN3_Pin)
#define MOTOREN3_ON()           GPIO_SetBits(GPIO_MOTOREN3 , GPIO_MOTOREN3_Pin)
/*********************************************/		

class MOTOR_TOP
{
public:
    MOTOR_PID motor1,motor2,motor3;
    unsigned char Enable_M1,Enable_M2,Enable_M3 ;    //Whether or not enable motor control
    float Expect_Angle_Speed_M1,Expect_Angle_Speed_M2,Expect_Angle_Speed_M3;

    MOTOR_TOP();
    void Motor_Top_Init(void);
    void Motor_Top_Call(void);

private:
    void Motor_Set_PWM(unsigned char Motor_ID , float Pid_out);
};

extern MOTOR_TOP motor_top;

#endif

