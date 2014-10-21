#include  "systemInit.h"


/*==============================================================================
1.预估计
   X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)


2.计算预估计协方差矩阵
   P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k)
   Q(k) = U(k)×U(k)' 


3.计算卡尔曼增益矩阵
   K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k))
   R(k) = N(k)×N(k)' 


4.更新估计
   X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))


5.计算更新后估计协防差矩阵
   P(k|k) =（I-K(k)*H(k)）*P(k|k-1)


6. 更新最优值


A(k,k-1):     状态转移矩阵
B(k,k-1):     输入控制加权矩阵
H(k):         观测矩阵

X(k|k-1):     根据k-1时刻的最优值估计k时刻的值
X(k-1|k-1):   k-1时刻的最优值
P(k|k-1):     X(k|k-1)对应的covariance
P(k-1|k-1):   X(k-1|k-1)对应的covariance

Z(k):         k时刻的测量值
u(k):         k时刻的输入控制信号
K(k):         k时刻的卡尔曼增益

U(k):         k时刻的动态噪声
N(k):         k时刻的观测噪声
Q(k):         k时刻系统过程的covariance(对上一次测量估计值的信任程度)
R(k):         k时刻测量过程的covariance(对测量的信任程度)

例如:     系统状态X-----------实际温度
          系统矩阵A-----------温度变化转移
          状态的控制量B-------(通常没有)
          观测值Z-------------温度计读数
          观测矩阵H-----------摄氏度-〉华氏度
          过程噪声Q-----------温度变化偏差
          测量噪声R-----------读数误差

基本思路: 首先根据上一次(如果是第一次则根据预赋值计算)的数据计算出本次的估计值,
          同理,根据上一次的数据计算出本次估计值的协方差;  接着,由本次估计值的协
          方差计算出卡尔曼增益;  最后,根据估测值和测量值计算当前最优值及其协方差
          另外,卡尔曼滤波只是时域上的处理,可以认为频率影响很小
==============================================================================*/


//================================================//
//==               最优值结构体                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[X_LENGTH];
  float XPreOpt[X_LENGTH];
}tOptimal;


//================================================//
//==             最优值方差结构体               ==//
//================================================//
typedef struct  _tCovariance
{
  float PNowOpt[P_LENGTH];
  float PPreOpt[P_LENGTH];
}tCovariance;


tOptimal      tOpt;                                     //  在滤波函数中进行初始化
tCovariance   tCov;                                     //  在滤波函数中进行初始化
float         Z[Z_LENGTH]  = Z_VALUE;                   //  测量值(每次测量的数据需要存入该数组)
float         u[u_LENGTH]  = u_VALUE;                   //  输入控制信号量
float         I[I_LENGTH]  = I_VALUE;                   //  单位矩阵
float         X[X_LENGTH]  = X_VALUE;                   //  当前状态的预测值
float         P[P_LENGTH]  = P_VALUE;                   //  当前状态的预测值的协方差
float         K[K_LENGTH]  = K_VALUE;                   //  卡尔曼增益
//============================================================================//
//==                    卡尔曼滤波需要配置的变量                            ==//
//============================================================================//
float         A[A_LENGTH]       = A_VALUE;              //  状态转移矩阵
float         B[B_LENGTH]       = B_VALUE;              //  输入控制加权矩阵
float         Q[Q_LENGTH]       = Q_VALUE;              //  系统过程的协方差
float         R[R_LENGTH]       = R_VALUE;              //  测量过程的协方差
float         H[H_LENGTH]       = H_VALUE;              //  观测矩阵

float         Temp1[1]          = {0};                  //  辅助变量, 注意辅助变量的大小
float         Temp2[2]          = X_VALUE;              //  辅助变量, 同时保存tOpt.XPreOpt[]的初始化值
float         Temp3[2]          = X_VALUE;              //  辅助变量
float         Temp4[4]          = P_VALUE;              //  辅助变量, 同时保存tCov.PPreOpt[]的初始化值





//============================================================================//
//==                          卡尔曼滤波                                    ==//
//============================================================================//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void KalMan(void)
{
  unsigned char   i;
  unsigned short  j;
  
  
  for (i=0; i<X_LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp2[i];           //初始化
  }
  for (i=0; i<P_LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp4[i];           //初始化
  }
  
  
  for (j=0; j<N; j++)
  {
    
    //Watch1[j] = sin(2*3.14159265/500.0*j);
    Z[0] = sin(2*3.14159265/20.0*j) + Random1(0, 0.04);                      //  得到此时的测量值
    Watch1[j] = Z[0];
    
    u[0] = 2*3.14159265/20.0*cos(2*3.14159265/20.0*j) - 2 + Random1(0, 0.4);//  得到此时的控制信号量
    Watch2[j] = u[0];
      
    MatrixMul(A, tOpt.XPreOpt, X, A_ROW, X_ROW, X_COLUMN);       //  基于系统的上一状态而预测现在状态; X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    MatrixMul(B, u, Temp2, B_ROW, u_ROW, u_COLUMN);
    MatrixAdd(X, Temp2, X, X_ROW, X_COLUMN);
    
    MatrixCal(A, tCov.PPreOpt, Temp4, A_ROW, A_COLUMN);
    MatrixAdd(Temp4, Q, P, P_ROW, P_COLUMN);                     //  预测数据的协方差矩阵; P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q
    
    MatrixCal(H, P, Temp1, H_ROW, H_COLUMN);
    MatrixAdd(Temp1, R, Temp1, R_ROW, R_COLUMN);
    Gauss_Jordan(Temp1, H_ROW);
    MatrixTrans(H, Temp2, H_ROW, H_COLUMN);
    MatrixMul(P, Temp2, Temp3, P_ROW, H_COLUMN, H_ROW);
    MatrixMul(Temp3, Temp1, K, P_ROW, H_ROW, H_ROW);             //  计算卡尔曼增益; K(k) = P(k|k-1)*H' / (H(k)*P(k|k-1)*H(k)' + R)
    
    MatrixMul(H, X, Temp1, H_ROW, X_ROW, X_COLUMN);
    MatrixMinus(Z, Temp1, Temp1, Z_ROW, Z_COLUMN);
    MatrixMul(K, Temp1, Temp2, K_ROW, Z_ROW, Z_COLUMN);
    MatrixAdd(X, Temp2, tOpt.XNowOpt, X_ROW, X_COLUMN);          //  根据估测值和测量值计算当前最优值; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))
    
    MatrixMul(K, H, Temp4, K_ROW, H_ROW, H_COLUMN);
    MatrixMinus(I, Temp4, Temp4, I_ROW, I_COLUMN);
    MatrixMul(Temp4, P, tCov.PNowOpt, I_ROW, P_ROW, P_COLUMN);   //  计算更新后估计协防差矩阵; P(k|k) =（I-Kg(k)*H）*P(k|k-1)
    
    for (i=0; i<X_LENGTH; i++)
    {
      tOpt.XPreOpt[i] = tOpt.XNowOpt[i];                         //  更新最优值
    }
    for (i=0; i<P_LENGTH; i++)
    {
      tCov.PPreOpt[i] = tCov.PNowOpt[i];                         //  更新最优协方差
    }
    
    Watch3[j] = tOpt.XNowOpt[0];
    Watch4[j] = tOpt.XNowOpt[1];
    
  }//end of for
}//end of Kalman





//============================================================================//
//==                  产生服从正态分布的随机数序列                          ==//
//============================================================================//
//==正态分布的概率密度函数  均值为u   方差为σ2(或标准差σ)                 ==//
//==                  Z=(x-μ)/σ ～ N(0,1)                                 ==//
//==入口参数: Num               产生随机数的个数                            ==//
//==          *S                保存随机数的数组                            ==//
//==          mu                需要的均值                                  ==//
//==          sigma             需要的方差                                  ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Random(unsigned long Num, float *S, float mu, float sigma)
{
  unsigned long  j;
  unsigned int   Temp;
  float r;
  
  for(j=0; j<Num; j++)                //循环产生随机数
  {
    Temp = rand();
    r = (Temp+0.00)/1073741823.00;
    S[j] = sqrt(-2*log(r))*cos(2*3.14159265*rand())*sigma+mu;
  }
}





//============================================================================//
//==                   产生一个服从正态分布的随机数                         ==//
//============================================================================//
//==入口参数: mu                需要的均值                                  ==//
//==          sigma             需要的方差                                  ==//
//==出口参数: 无                                                            ==//
//==返回值:   随机数                                                        ==//
//============================================================================//
float Random1(float mu, float sigma)
{
  unsigned int Temp;
  float r;
  float Ret;
  
  Temp = rand();
  r = (Temp+0.00)/1073741823.00;
  Ret = sqrt(-2*log(r))*cos(2*3.14159265*rand())*sigma+mu;
  
  return Ret;
}