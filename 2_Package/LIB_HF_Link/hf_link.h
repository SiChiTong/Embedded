#ifndef __hf_link_H__
#define __hf_link_H__

#include <string.h>
#include "queue.h"
#include "hf_link_config.h"

#define HF_Link_USART     USART1
#define HF_Link_ACK_EN    1       //enable hflink ack

//uasrt init  Parameter 1 USARTx , Parameter 1 baud rate
#define HF_Link_Usart_Init   HF_Usart_Init(HF_Link_USART,921600,0)
#define MY_ID    0X11              //按照协议标准  这个ID=0X11 属于从机范围 
#define Message_Bufer_Size   100   //定义缓冲池大小

/****structure for communications protocol , read Hands Free Link Manua.doc for detail****/
typedef struct HF_Message{
    unsigned char Sender_ID;        //对于接收而言 这个ID是发出该包的ID 对于发送而言 这个ID就是MY_ID
    unsigned char Receiver_ID;      //对于接收而言 这个ID必须是MY_ID 对于发送而言 这个ID就是要发给哪个ID号
    unsigned short int Length;
    unsigned char Data[Message_Bufer_Size];
}HF_Message;

//状态机
enum Recstate{  
    WAITING_FF1, WAITING_FF2,SENDID,RECID,RECLEN_H,RECLEN_L,RECPACKAGE,RECCHECK
};

enum COMMAND{Shaking_Hands , Read_BSP_Info
             ,Set_Global_Speed , Read_Global_Speed , Set_Robot_Speed ,Read_Robot_Speed
             ,Set_Motor_Speed , Read_Motor_Speed , Read_Motor_Mileage
             ,Read_Global_Coordinate , Clear_Coordinate_Data
             ,Set_Arm_1 , Read_Arm_1 , Set_Arm_2 , Read_Arm_2
             ,Set_Head_1 , Read_Head_1 ,Set_Head_2 , Read_Head_2
            };

class HF_Link:public Queue
{
public:
    HF_Link();
    void HF_Link_Init(void);
    void HF_Link_Call(void);
    void HF_Link_Receive_Byte(unsigned char Rx_Byte);
    unsigned char Receive_Message_Update;
    unsigned char Shaking_Hands_State;    // 1 Success   0 Failed
    unsigned char Control_Signal_Update;
    
private:
    Recstate   Receive_State;
    COMMAND    HF_Command_State;
    HF_Message RX_Message , TX_Message;
    void Receive_Finite_States(unsigned char Rx_data);
    void Package_Analysis_3WD(void);
    void Send_Message(void);
};

extern HF_Link hf_link;

#endif  //#ifndef __HF_Link_H__

