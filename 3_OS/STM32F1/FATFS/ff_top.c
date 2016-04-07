/************************************************************************************
* Copyright (C), MAWENKE
*
* FileName:   ff_top.c
*
* Author:     MAWENKE
*
* Version :  
*
* Date: 2015-11-3
*
* Description: 
*
* Function List: 
*
* 1. -------
*
* History: 
*
* <author> <time> <version > <desc>
* 

***********************************************************************************/

#include "ff_top.h"

#if FatFS_TEST_ENABLE > 0u    //ʹ��SD���ļ�ϵͳ����


int res;  
int a;
FIL fdst;
FATFS fs;   
UINT br, bw;            // File R/W count
BYTE buffer[512];       // file copy buffer
BYTE textFileBuffer[] = " HANDS FREE ��^_^ \r\n " ;

void FatFS_Test(void)
{	

 	printf("\r\n ����һ��MicroSD���ļ�ϵͳʵ��(FATFS R0.09)\n");
	printf ( "\r\n disk_initialize starting......\n " );

	f_mount(0,&fs);		    //����0����

	res = f_open(&fdst,"0:/Demo.TXT",FA_CREATE_NEW | FA_WRITE);
	 
	if ( res == FR_OK )
 	 { 
    /* ��������������д���ļ��� */
		res = f_write(&fdst, textFileBuffer, sizeof(textFileBuffer), &bw); 
	  printf( "\r\n �ļ������ɹ� \n" );        
    f_close(&fdst);   /*�ر��ļ� */      
   }
   else if ( res == FR_EXIST )
 	 { 
		printf( "\r\n �ļ��Ѿ����� \n" );
 	 }

   /*---------------- ���ո��½����ļ���������ݴ�ӡ�������ն� -----------------------*/
  /* ��ֻ���ķ�ʽ�򿪸ոմ������ļ� */
	res = f_open(&fdst, "0:/Demo.TXT", FA_OPEN_EXISTING | FA_READ); /* ���ļ� */		
	br = 1;
	a = 0;	
  for (;;) 
	{
		for ( a=0; a<512; a++ ) 	                          /* �建���� */
			buffer[a]=0;
			 
  	res = f_read( &fdst, buffer, sizeof(buffer), &br ); /* ���ļ���������ݶ��������� */
		printf("\r\n %s ", buffer);						
  	if (res || br == 0) break;                          /* ������ߵ����ļ�β */        	    	
  }
	
	f_close(&fdst);	                                      /* �رմ򿪵��ļ� */	

 
}


#endif

	  
