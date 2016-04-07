/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
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
* Description: This file defined hands_free_robot simple communications protocol
*              please read Hands Free Link Manua.doc for detail
***********************************************************************************************************************/

#include "hf_link.h"

HF_Link hf_link;

/**********************************************************************************************************************
***                                                                                                                 ***
***                                           driver Function                                                       ***
***                                                                                                                 ***
***********************************************************************************************************************/

HF_Link::HF_Link()
{
    Receive_Message_Update=0;
    Shaking_Hands_State=0;
    Control_Signal_Update=0;
    
    Receive_State=WAITING_FF1;
    HF_Command_State=Shaking_Hands;
    RX_Message.Sender_ID=0;
    RX_Message.Receiver_ID=0;
    RX_Message.Length=0;
    TX_Message.Sender_ID=0;
    TX_Message.Receiver_ID=0;
    TX_Message.Length=0;
}

void HF_Link::HF_Link_Receive_Byte(unsigned char Rx_Byte)
{
    if( full_check()) {
        return;
    }
    else{
        put_data(Rx_Byte);
    }
}


void HF_Link::HF_Link_Init(void)  //usart1 was init in bspinit function
{
    //HF_Link_Usart_Init;
    Queue_Init();
}

void HF_Link::HF_Link_Call(void)
{
    static unsigned char Rx_Byte;
    if( empty_check() ){
        return;
    }
    else {
        Rx_Byte=get_data();
        Receive_Finite_States(Rx_Byte);  //状态机
        if( Receive_Message_Update ==1 )
        {
            Receive_Message_Update = 0;
            Package_Analysis_3WD();       //包的内容解析
        }

    }
}

/***********************************************************************************************************************
***                                                                                                                  ***
***                                              APP Function                                                        ***
***                                                                                                                  ***
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function:     void HF_Link::Receive_Analysis(unsigned char Rx_data)
*
* Scope:        Analysis the data from hf_link host
*
* Description:
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
void HF_Link::Receive_Finite_States(unsigned char Rx_data)
{
    static unsigned int check_sum;
    static short int Length;
    static short int cur;

    switch (Receive_State)
    {
    case WAITING_FF1:
        if (Rx_data == 0xff)
        {
            Receive_State = WAITING_FF2;
            check_sum =0;
            Length = 0;
            cur=0;
            check_sum += Rx_data;
        }
        break;

    case WAITING_FF2:
        if (Rx_data == 0xff)
        {
            Receive_State = SENDID;
            check_sum += Rx_data;
        }
        else
            Receive_State = WAITING_FF1;
        break;

    case SENDID:
        RX_Message.Sender_ID = 	Rx_data ;
        if ( Rx_data == 0X01 )             //发送者ID 终端ID
        {
            check_sum += Rx_data;
            Receive_State = RECID;
        }
        else
            Receive_State = WAITING_FF1;
        break;

    case RECID:
        RX_Message.Receiver_ID = Rx_data ;
        if (Rx_data == MY_ID) 			  //我的ID 接收者ID
        {
            check_sum += Rx_data;
            Receive_State = RECLEN_H;
        }
        else
            Receive_State = WAITING_FF1;
        break;

    case RECLEN_H:
        check_sum += Rx_data;
        Length |= Rx_data<<8;
        Receive_State = RECLEN_L;
        break;

    case RECLEN_L:
        check_sum += Rx_data;
        Length |= Rx_data;
        RX_Message.Length = Length;
        Receive_State = RECPACKAGE;
        break;

    case RECPACKAGE:
        check_sum += Rx_data;
        RX_Message.Data[cur++] = Rx_data;
        if(cur >= Length)
        {
            Receive_State = RECCHECK;
            check_sum=check_sum%255;
        }
        break;

    case RECCHECK:
        if(Rx_data == (unsigned char)check_sum)
        {
            Receive_Message_Update=1;
            check_sum=0;
            Receive_State = WAITING_FF1;
        }
        else
            Receive_State = WAITING_FF1;
        break;
    default:
        Receive_State = WAITING_FF1;
    }

}


/***********************************************************************************************************************
* Function:     void HF_Link::Package_Analysis_3WD(void)
*
* Scope:        对接收的包内容解析 适用于3WD机器人
*
* Description:
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
void HF_Link::Package_Analysis_3WD(void)
{

    static unsigned HF_Command_State;

    //		Set_Global_Speed  //给机器人发送 世界坐标系的速度向量  X  Y  Z   (float)
    //		Set_Robot_Speed   //给机器人发送 机器人坐标系的速度向量   X  Y  Z   (float)
    //		Read_Global_Coordinate  //读取机器人世界坐标系坐标值 返回 HF_Command_State=0X03  X Y Z (float)
    //		Set_Motor_Speed  //给机器人发送 电机速度   V1 V2 V3   (float)
    //		Read_Motor_Speed    //读取三个电机的角速度  返回  HF_Command_State=0X05  V1 V2 V3   (float)
    //		Read_Motor_Mileage //读取三个电机的里程值(单位 M)  返回  HF_Command_State=0X06  L1 L2 L3  (float)
    //		Clear_Coordinate_Data//清除底盘历史数据  即设置当前坐标 为坐标原点

    HF_Command_State = (COMMAND)RX_Message.Data[0];
    if(HF_Command_State != Shaking_Hands && Shaking_Hands_State==0)  //如果没有握手成功，则不解析包
    {
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Shaking_Hands;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                            //0  error  1 right
        Send_Message();

        return;
    }

    switch (HF_Command_State)
    {
    case Shaking_Hands :
        Shaking_Hands_State=1;
        memcpy(&Hands_Free_3WD.Mesure_Global_Coordinate , &RX_Message.Data[1] , 12); //receive the memory coordinat of PC
        break;

    case Read_BSP_Info :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_BSP_Info;    //HF_Command_State
        memcpy(&TX_Message.Data[1],&system_data.System_Time,4);
        memcpy(&TX_Message.Data[1+4],&system_data.CPU_Temperature,4);
        memcpy(&TX_Message.Data[1+4+4],&system_data.Battery_Voltage,4);
        Send_Message();
        break;

    case Set_Global_Speed :
        Control_Signal_Update = 1 ;               //means the Control Signal Update once
        memcpy(&Hands_Free_3WD.Expect_Global_Speed , &RX_Message.Data[1] , 12);
        Hands_Free_3WD.Global_Speed_Renew = 1;

#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Global_Speed;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Global_Speed :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_Global_Speed;    //HF_Command_State
        memcpy(&TX_Message.Data[1] , &Hands_Free_3WD.Mesure_Global_Speed ,12);
        Send_Message();
        break;


    case Set_Robot_Speed :  //给机器人发送 机器人坐标系的速度向量   X  Y  Z   (float)
        Control_Signal_Update = 1 ;                 //mean the Control Signal Update once
        memcpy(&Hands_Free_3WD.Expect_Robot_Speed , &RX_Message.Data[1] , 12);	//设置机器人坐标速度
        Hands_Free_3WD.Robot_Speed_Renew = 1;

#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Robot_Speed ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Robot_Speed :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_Robot_Speed;    //HF_Command_State
        memcpy(&TX_Message.Data[1] , &Hands_Free_3WD.Mesure_Robot_Speed ,12);
        Send_Message();
        break;

    case Read_Global_Coordinate : //读取机器人世界坐标系坐标值 返回 HF_Command_State=0X03  X Y Z (float)
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_Global_Coordinate;    //HF_Command_State
        memcpy(&TX_Message.Data[1],&Hands_Free_3WD.Mesure_Global_Coordinate,12);
        Send_Message();
        break;

    case Set_Motor_Speed :    //给机器人发送 电机速度   V1 V2 V3   (float)
        Control_Signal_Update = 1 ;              //mean the Control Signal Update once
        memcpy(&Hands_Free_3WD.Expect_Motor_Angle_Speed , &RX_Message.Data[1] , 12);
        Hands_Free_3WD.Motor_Speed_Renew = 1;

#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Motor_Speed ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Motor_Speed : //读取三个电机的角速度  返回  HF_Command_State=0X05  V1 V2 V3   (float)
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_Motor_Speed;    //HF_Command_State
        memcpy(&TX_Message.Data[1],&Hands_Free_3WD.Mesure_Motor_Speed ,12);
        Send_Message();
        break;

    case Read_Motor_Mileage : //读取三个电机的里程值(单位 M)  返回  HF_Command_State=0X06  L1 L2 L3  (float)
        TX_Message.Receiver_ID = RX_Message.Sender_ID;  //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=13;
        TX_Message.Data[0] = (unsigned char)Read_Motor_Mileage;    //HF_Command_State
        memcpy(&TX_Message.Data[1],&Hands_Free_3WD.Mesure_Motor_Distance ,12);
        Send_Message();
        break;

    case Clear_Coordinate_Data :  //清除底盘历史数据  即设置当前坐标 为坐标原点
        Hands_Free_3WD.Mesure_Global_Coordinate.X = 0 ;  //清除世界坐标信息
        Hands_Free_3WD.Mesure_Global_Coordinate.Y = 0 ;
        Hands_Free_3WD.Mesure_Global_Coordinate.Z = 0 ;
        Hands_Free_3WD.Mesure_Robot_Coordinate.X = 0 ;   //清除机器人坐标信息
        Hands_Free_3WD.Mesure_Robot_Coordinate.Y = 0 ;
        Hands_Free_3WD.Mesure_Robot_Coordinate.Z = 0 ;
        Hands_Free_3WD.Mesure_Motor_Distance.M1 = 0 ;    //清除电机里程值信息
        Hands_Free_3WD.Mesure_Motor_Distance.M2 = 0 ;
        Hands_Free_3WD.Mesure_Motor_Distance.M3 = 0 ;

        TX_Message.Receiver_ID = RX_Message.Sender_ID;   //谁给我发的 就给谁反馈一个ACK
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Clear_Coordinate_Data;   //HF_Command_State
        TX_Message.Data[1] = 0x01;                                   //0  error  1 right
        Send_Message();
        break;

    case Set_Arm_1 :

#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Arm_1 ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif

        break;

    case Read_Arm_1 :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;   //谁给我发的 就给谁反馈一个ACK
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Read_Arm_1;   //HF_Command_State
        TX_Message.Data[1] = 0x01;                        //0  error  1 right
        Send_Message();
        break;

    case Set_Arm_2 :
#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Arm_2 ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Arm_2 :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;   //谁给我发的 就给谁反馈一个ACK
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Read_Arm_2;   //HF_Command_State
        TX_Message.Data[1] = 0x01;                        //0  error  1 right
        Send_Message();
        break;

    case Set_Head_1 :
#ifdef Driver_Servo
        memcpy(&hf_head.Expect_Head_State , &RX_Message.Data[1] , 4*2);
        hf_head.Set_Head_State_Renew = 1;
#endif
#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Head_1 ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Head_1 :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;   //谁给我发的 就给谁反馈一个ACK
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Read_Head_1;   //HF_Command_State
        TX_Message.Data[1] = 0x01;                         //0  error  1 right
        Send_Message();
        break;

    case Set_Head_2 :
#if HF_Link_ACK_EN ==1
        TX_Message.Receiver_ID = RX_Message.Sender_ID; //send a ACK to the sender of this message
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Set_Head_2 ;    //HF_Command_State
        TX_Message.Data[1] = 0x01;                               //0  error  1 receive right data
        Send_Message();
#endif
        break;

    case Read_Head_2 :
        TX_Message.Receiver_ID = RX_Message.Sender_ID;   //谁给我发的 就给谁反馈一个ACK
        TX_Message.Sender_ID = MY_ID;
        TX_Message.Length=2;
        TX_Message.Data[0] = (unsigned char)Read_Head_2;   //HF_Command_State
        TX_Message.Data[1] = 0x01;                         //0  error  1 right
        Send_Message();
        break;

    default :
        break;

    }

}

/***********************************************************************************************************************
* Function:    void HF_Link::Send_Message(void)
*
* Scope:       send a message to hf_link node
*
* Description:
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
void HF_Link::Send_Message(void)
{
    unsigned short int TX_i;
    unsigned int check_sum=0;

    Usart_Put_Char(HF_Link_USART , 0xff);
    check_sum += 0xff;
    Usart_Put_Char(HF_Link_USART ,0xff);
    check_sum += 0xff;
    Usart_Put_Char(HF_Link_USART ,TX_Message.Sender_ID);
    check_sum += TX_Message.Sender_ID;
    Usart_Put_Char(HF_Link_USART , TX_Message.Receiver_ID);
    check_sum += TX_Message.Receiver_ID;
    Usart_Put_Char(HF_Link_USART , (unsigned char)( TX_Message.Length >> 8) );  //LEN_H
    check_sum += ( TX_Message.Length >> 8);
    Usart_Put_Char(HF_Link_USART , (unsigned char)TX_Message.Length );          //LEN_L
    check_sum += (unsigned char)TX_Message.Length;
    for(TX_i=0; TX_i < TX_Message.Length ; TX_i++)                              //package
    {
        Usart_Put_Char(HF_Link_USART , TX_Message.Data[TX_i] );
        check_sum += TX_Message.Data[TX_i];
    }
    check_sum=check_sum%255;
    Usart_Put_Char(HF_Link_USART, (unsigned char)check_sum);                   //check_sum
}

