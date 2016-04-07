/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: queue.cpp
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
#include "queue.h"

Queue::Queue()
{
    arr[0]=0;
    head=0;
    tail=0;
}

void Queue::Queue_Init(void)
{
    
}

unsigned char Queue::get_data(void)
{
    unsigned char data;
    data = arr[head];
    head++;
    if(head >= QUEUE_SIZE) head=0;
    return data;
}

void Queue::put_data(unsigned char ch)
{
    arr[tail] = ch;
    tail++;
    if(tail >= QUEUE_SIZE) tail=0;
}

unsigned char Queue::empty_check(void)
{
    if( head == tail )
        return 1;
    else
        return 0;
}

unsigned char Queue::full_check(void)
{
    if( tail + 1 == head )
        return 1;
    else
        return 0;
}
