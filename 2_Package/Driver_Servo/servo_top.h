#ifndef __servo_top_H__
#define __servo_top_H__

#include "servo_config.h"
#include "servo_digital_driver.h"

typedef struct ROBOT_HEAD{
	float  Pitch;
	float  Yaw;} ROBOT_HEAD;

class SERVO_TOP:public SERVO_D
{
	public:
	unsigned char Set_Head_State_Renew,Read_Head_State_Renew;
	ROBOT_HEAD  Expect_Head_State , Mesure_Head_State;                  
	SERVO_TOP();
	void Init(void);
	void Head_Top_Call(void);
	private:
	void Set_State(float Pitch , float Yaw);
	void Get_State(float *Pitch , float *Yaw);
};

extern SERVO_TOP hf_head;

#endif






