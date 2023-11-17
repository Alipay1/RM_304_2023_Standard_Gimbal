
#ifndef __VISUALSCOPE__
#define __VISUALSCOPE__
		 
#include "main.h"

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void VisualScope_Output(float data1 ,float data2 ,float data3 ,float data4);
void display(void);
void VisualScope_Thread(void *arg);


#endif    

