#ifndef  __SYSTEM_INIT_H__
#define  __SYSTEM_INIT_H__


//  ������Ҫ��ͷ�ļ�
#include  <hw_types.h>
#include  <hw_memmap.h>
#include  <hw_ints.h>
#include  <interrupt.h>
#include  <sysctl.h>
#include  <gpio.h>
#include  <uart.h>
#include  <adc.h>
#include  "main.h"


#include  "kalman.h"
#include  "matrix.h"
#include  "stdlib.h"
#include  "math.h"

//  ���ϳ��ı�ʶ������ɽ϶̵���ʽ
#define  SysCtlPeriEnable       SysCtlPeripheralEnable
#define  SysCtlPeriDisable      SysCtlPeripheralDisable
#define  GPIOPinTypeIn          GPIOPinTypeGPIOInput
#define  GPIOPinTypeOut         GPIOPinTypeGPIOOutput
#define  GPIOPinTypeOD          GPIOPinTypeGPIOOutputOD
#define  ADCSequEnable          ADCSequenceEnable
#define  ADCSequDisable         ADCSequenceDisable
#define  ADCSequConfig          ADCSequenceConfigure
#define  ADCSequStepConfig      ADCSequenceStepConfigure
#define  ADCSequDataGet         ADCSequenceDataGet


//  ����ȫ�ֵ�ϵͳʱ�ӱ���
extern unsigned long TheSysClock;


//  ��ֹJTAGʧЧ
extern void jtagWait(void);


//  ϵͳʱ�ӳ�ʼ��
extern void clockInit(void);


#endif  //  __SYSTEM_INIT_H__

