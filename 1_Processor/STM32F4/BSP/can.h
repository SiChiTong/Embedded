#ifndef __can_H__
#define __can_H__

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx.h"
#include "main_config.h"

#define MYCAN_ID  0x01

void HF_Can_Init(CAN_TypeDef *CANx, unsigned char GPIO_AF);
void Can_Tx_Message(CAN_TypeDef* CANx, uint8_t MY_ID, uint8_t Receiver_ID, uint8_t *TxBuf, uint8_t Length);


#ifdef __cplusplus
}
#endif 

#endif //__can_H__


