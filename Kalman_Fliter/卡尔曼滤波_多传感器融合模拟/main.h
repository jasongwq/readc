#ifndef _MAIN_H
#define _MAIN_H


#include  "kalman.h"


extern float  Watch1[N];
extern float  Watch2[N];
extern float  Watch3[N];
extern float  Watch4[N];



extern void   UARTInit(void);
extern void   Long2Char(unsigned long longNum, unsigned char *pArray);
extern void   LongSend(unsigned long *LArray, unsigned int Num);
extern void   Short2Char(unsigned short ShortNum, unsigned char *pArray);
extern void   ShortSend(unsigned short *SArray, unsigned int Num);
extern void   Float2Char(float FloatNum, unsigned char *pArray);
extern void   FloatSend(float *FArray, unsigned int Num);


#endif