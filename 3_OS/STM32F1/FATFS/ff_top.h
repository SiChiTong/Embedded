#ifndef __ff_lib_H__
#define __ff_lib_H__

#include "BSP_TOP.h"	
#include "ff.h"

#define FatFS_TEST_ENABLE	 1u   //ʹ��SD���ļ�ϵͳ����

#if FatFS_TEST_ENABLE > 0u   //ʹ��SD���ļ�ϵͳ����

void FatFS_Test(void);

#endif    //FatFS_TEST_ENABLE


#endif //ff_lib.h
