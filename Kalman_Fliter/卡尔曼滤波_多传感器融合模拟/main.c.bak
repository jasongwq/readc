#include  "systemInit.h"


#define   UART_PERIPH       SYSCTL_PERIPH_UART0
#define   UART_USE_BASE     UART0_BASE
#define   GPIO_PERIPH       SYSCTL_PERIPH_GPIOA
#define   GPIO_USE_BASE     GPIO_PORTA_BASE
#define   GPIO_PINS         GPIO_PIN_0|GPIO_PIN_1
#define   BAUDRATE          19200


//============================================================================//
//==                          UART初始化函数                                ==//
//============================================================================//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void UARTInit(void)
{
  SysCtlPeripheralEnable(GPIO_PERIPH);        //使能UART所在GPIO端口
  SysCtlPeripheralEnable(UART_PERIPH);        //使能UART外设
  GPIOPinTypeUART(GPIO_USE_BASE, GPIO_PINS);  //使能UART所在引脚
  
  UARTConfigSet(UART_USE_BASE, BAUDRATE, UART_CONFIG_WLEN_8|
                                         UART_CONFIG_STOP_ONE|
                                         UART_CONFIG_PAR_NONE);
  UARTEnable(UART_USE_BASE);                  //使能UART
}





//============================================================================//
//==                          长整型转换函数                                ==//
//============================================================================//
//==入口参数: LongNum		指定的长整型                                ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Long2Char(unsigned long longNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&longNum);
  for (i=0; i<sizeof(long); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                          长整型数发送函数                              ==//
//============================================================================//
//==入口参数: *LArray         指向需要发送的长整型数数组                    ==//
//==          Num             长整型数据个数                                ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void LongSend(unsigned long *LArray, unsigned int Num)
{
  unsigned int i;
  unsigned char a[4] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Long2Char(LArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);   //低字节放在低地址,先发送低字节
    UARTCharPut(UART_USE_BASE, a[1]);
    UARTCharPut(UART_USE_BASE, a[2]);
    UARTCharPut(UART_USE_BASE, a[3]);
  }
}





//============================================================================//
//==                            整型转换函数                                ==//
//============================================================================//
//==入口参数: ShortNum		指定的整型                                  ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Short2Char(unsigned short ShortNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&ShortNum);
  for (i=0; i<sizeof(short); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                            整型数发送函数                              ==//
//============================================================================//
//==入口参数: *SArray         指向需要发送的整型数数组                      ==//
//==          Num             整型数据个数                                  ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void ShortSend(unsigned short *SArray, unsigned int Num)
{
  unsigned int i;
  unsigned char a[2] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Short2Char(SArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);   //低字节放在低地址,先发送低字节
    UARTCharPut(UART_USE_BASE, a[1]);
  }
}





//============================================================================//
//==                           浮点数转换函数                               ==//
//============================================================================//
//==入口参数: FloatNum		指定的浮点数                                ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Float2Char(float FloatNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&FloatNum);
  for (i=0; i<sizeof(float); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                           浮点数发送函数                               ==//
//============================================================================//
//==入口参数: *Farray		指向需要发送的浮点数数组                    ==//
//==	      Num		浮点数个数				    ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void FloatSend(float *FArray, unsigned int Num)
{
  unsigned int i;
  unsigned char a[4] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Float2Char(FArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);
    UARTCharPut(UART_USE_BASE, a[1]);
    UARTCharPut(UART_USE_BASE, a[2]);
    UARTCharPut(UART_USE_BASE, a[3]);
  }
}


float Watch1[N] = {0};
float Watch2[N] = {0};
float Watch3[N] = {0};
float Watch4[N] = {0};


//  主函数（程序入口）
int main(void)
{
    unsigned long ulStart = 0;
    unsigned long ulStop  = 0;
    unsigned long ulInterval = 0;
    
    jtagWait();                                             //  防止JTAG失效，重要！
    clockInit();                                            //  时钟初始化：晶振，6MHz
    UARTInit();
    srand(SEED);                                            //  模拟噪声
    
    
    
    TimerLoadSet(TIMER0_BASE, TIMER_A, TheSysClock);        // 设置Timer初值，定时1s
    TimerEnable(TIMER0_BASE, TIMER_A);                      // 使能Timer计数
    ulStart = TimerValueGet(TIMER0_BASE, TIMER_A);
    KalMan();
    ulStop  = TimerValueGet(TIMER0_BASE, TIMER_A);
    ulInterval = (ulStart-ulStop) / 50;                     // 计算时间间隔(单位us)
    
    
    
    FloatSend(Watch1, N);
    FloatSend(Watch2, N);
    FloatSend(Watch3, N);
    FloatSend(Watch4, N);
    
    
    for (;;)
    {
    }
}
