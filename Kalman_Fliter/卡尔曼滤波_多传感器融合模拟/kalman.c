#include  "systemInit.h"


/*==============================================================================
1.Ԥ����
   X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)


2.����Ԥ����Э�������
   P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k)
   Q(k) = U(k)��U(k)' 


3.���㿨�����������
   K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k))
   R(k) = N(k)��N(k)' 


4.���¹���
   X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))


5.������º����Э�������
   P(k|k) =��I-K(k)*H(k)��*P(k|k-1)


6. ��������ֵ


A(k,k-1):     ״̬ת�ƾ���
B(k,k-1):     ������Ƽ�Ȩ����
H(k):         �۲����

X(k|k-1):     ����k-1ʱ�̵�����ֵ����kʱ�̵�ֵ
X(k-1|k-1):   k-1ʱ�̵�����ֵ
P(k|k-1):     X(k|k-1)��Ӧ��covariance
P(k-1|k-1):   X(k-1|k-1)��Ӧ��covariance

Z(k):         kʱ�̵Ĳ���ֵ
u(k):         kʱ�̵���������ź�
K(k):         kʱ�̵Ŀ���������

U(k):         kʱ�̵Ķ�̬����
N(k):         kʱ�̵Ĺ۲�����
Q(k):         kʱ��ϵͳ���̵�covariance(����һ�β�������ֵ�����γ̶�)
R(k):         kʱ�̲������̵�covariance(�Բ��������γ̶�)

����:     ϵͳ״̬X-----------ʵ���¶�
          ϵͳ����A-----------�¶ȱ仯ת��
          ״̬�Ŀ�����B-------(ͨ��û��)
          �۲�ֵZ-------------�¶ȼƶ���
          �۲����H-----------���϶�-�����϶�
          ��������Q-----------�¶ȱ仯ƫ��
          ��������R-----------�������

����˼·: ���ȸ�����һ��(����ǵ�һ�������Ԥ��ֵ����)�����ݼ�������εĹ���ֵ,
          ͬ��,������һ�ε����ݼ�������ι���ֵ��Э����;  ����,�ɱ��ι���ֵ��Э
          ������������������;  ���,���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ����Э����
          ����,�������˲�ֻ��ʱ���ϵĴ���,������ΪƵ��Ӱ���С
==============================================================================*/


//================================================//
//==               ����ֵ�ṹ��                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[X_LENGTH];
  float XPreOpt[X_LENGTH];
}tOptimal;


//================================================//
//==             ����ֵ����ṹ��               ==//
//================================================//
typedef struct  _tCovariance
{
  float PNowOpt[P_LENGTH];
  float PPreOpt[P_LENGTH];
}tCovariance;


tOptimal      tOpt;                                     //  ���˲������н��г�ʼ��
tCovariance   tCov;                                     //  ���˲������н��г�ʼ��
float         Z[Z_LENGTH]  = Z_VALUE;                   //  ����ֵ(ÿ�β�����������Ҫ���������)
float         u[u_LENGTH]  = u_VALUE;                   //  ��������ź���
float         I[I_LENGTH]  = I_VALUE;                   //  ��λ����
float         X[X_LENGTH]  = X_VALUE;                   //  ��ǰ״̬��Ԥ��ֵ
float         P[P_LENGTH]  = P_VALUE;                   //  ��ǰ״̬��Ԥ��ֵ��Э����
float         K[K_LENGTH]  = K_VALUE;                   //  ����������
//============================================================================//
//==                    �������˲���Ҫ���õı���                            ==//
//============================================================================//
float         A[A_LENGTH]       = A_VALUE;              //  ״̬ת�ƾ���
float         B[B_LENGTH]       = B_VALUE;              //  ������Ƽ�Ȩ����
float         Q[Q_LENGTH]       = Q_VALUE;              //  ϵͳ���̵�Э����
float         R[R_LENGTH]       = R_VALUE;              //  �������̵�Э����
float         H[H_LENGTH]       = H_VALUE;              //  �۲����

float         Temp1[1]          = {0};                  //  ��������, ע�⸨�������Ĵ�С
float         Temp2[2]          = X_VALUE;              //  ��������, ͬʱ����tOpt.XPreOpt[]�ĳ�ʼ��ֵ
float         Temp3[2]          = X_VALUE;              //  ��������
float         Temp4[4]          = P_VALUE;              //  ��������, ͬʱ����tCov.PPreOpt[]�ĳ�ʼ��ֵ





//============================================================================//
//==                          �������˲�                                    ==//
//============================================================================//
//==��ڲ���: ��                                                            ==//
//==���ڲ���: ��                                                            ==//
//==����ֵ:   ��                                                            ==//
//============================================================================//
void KalMan(void)
{
  unsigned char   i;
  unsigned short  j;
  
  
  for (i=0; i<X_LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp2[i];           //��ʼ��
  }
  for (i=0; i<P_LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp4[i];           //��ʼ��
  }
  
  
  for (j=0; j<N; j++)
  {
    
    //Watch1[j] = sin(2*3.14159265/500.0*j);
    Z[0] = sin(2*3.14159265/20.0*j) + Random1(0, 0.04);                      //  �õ���ʱ�Ĳ���ֵ
    Watch1[j] = Z[0];
    
    u[0] = 2*3.14159265/20.0*cos(2*3.14159265/20.0*j) - 2 + Random1(0, 0.4);//  �õ���ʱ�Ŀ����ź���
    Watch2[j] = u[0];
      
    MatrixMul(A, tOpt.XPreOpt, X, A_ROW, X_ROW, X_COLUMN);       //  ����ϵͳ����һ״̬��Ԥ������״̬; X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    MatrixMul(B, u, Temp2, B_ROW, u_ROW, u_COLUMN);
    MatrixAdd(X, Temp2, X, X_ROW, X_COLUMN);
    
    MatrixCal(A, tCov.PPreOpt, Temp4, A_ROW, A_COLUMN);
    MatrixAdd(Temp4, Q, P, P_ROW, P_COLUMN);                     //  Ԥ�����ݵ�Э�������; P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q
    
    MatrixCal(H, P, Temp1, H_ROW, H_COLUMN);
    MatrixAdd(Temp1, R, Temp1, R_ROW, R_COLUMN);
    Gauss_Jordan(Temp1, H_ROW);
    MatrixTrans(H, Temp2, H_ROW, H_COLUMN);
    MatrixMul(P, Temp2, Temp3, P_ROW, H_COLUMN, H_ROW);
    MatrixMul(Temp3, Temp1, K, P_ROW, H_ROW, H_ROW);             //  ���㿨��������; K(k) = P(k|k-1)*H' / (H(k)*P(k|k-1)*H(k)' + R)
    
    MatrixMul(H, X, Temp1, H_ROW, X_ROW, X_COLUMN);
    MatrixMinus(Z, Temp1, Temp1, Z_ROW, Z_COLUMN);
    MatrixMul(K, Temp1, Temp2, K_ROW, Z_ROW, Z_COLUMN);
    MatrixAdd(X, Temp2, tOpt.XNowOpt, X_ROW, X_COLUMN);          //  ���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))
    
    MatrixMul(K, H, Temp4, K_ROW, H_ROW, H_COLUMN);
    MatrixMinus(I, Temp4, Temp4, I_ROW, I_COLUMN);
    MatrixMul(Temp4, P, tCov.PNowOpt, I_ROW, P_ROW, P_COLUMN);   //  ������º����Э�������; P(k|k) =��I-Kg(k)*H��*P(k|k-1)
    
    for (i=0; i<X_LENGTH; i++)
    {
      tOpt.XPreOpt[i] = tOpt.XNowOpt[i];                         //  ��������ֵ
    }
    for (i=0; i<P_LENGTH; i++)
    {
      tCov.PPreOpt[i] = tCov.PNowOpt[i];                         //  ��������Э����
    }
    
    Watch3[j] = tOpt.XNowOpt[0];
    Watch4[j] = tOpt.XNowOpt[1];
    
  }//end of for
}//end of Kalman





//============================================================================//
//==                  ����������̬�ֲ������������                          ==//
//============================================================================//
//==��̬�ֲ��ĸ����ܶȺ���  ��ֵΪu   ����Ϊ��2(���׼���)                 ==//
//==                  Z=(x-��)/�� �� N(0,1)                                 ==//
//==��ڲ���: Num               ����������ĸ���                            ==//
//==          *S                ���������������                            ==//
//==          mu                ��Ҫ�ľ�ֵ                                  ==//
//==          sigma             ��Ҫ�ķ���                                  ==//
//==���ڲ���: *c                ָ���������ָ��                          ==//
//==����ֵ:   ��                                                            ==//
//============================================================================//
void Random(unsigned long Num, float *S, float mu, float sigma)
{
  unsigned long  j;
  unsigned int   Temp;
  float r;
  
  for(j=0; j<Num; j++)                //ѭ�����������
  {
    Temp = rand();
    r = (Temp+0.00)/1073741823.00;
    S[j] = sqrt(-2*log(r))*cos(2*3.14159265*rand())*sigma+mu;
  }
}





//============================================================================//
//==                   ����һ��������̬�ֲ��������                         ==//
//============================================================================//
//==��ڲ���: mu                ��Ҫ�ľ�ֵ                                  ==//
//==          sigma             ��Ҫ�ķ���                                  ==//
//==���ڲ���: ��                                                            ==//
//==����ֵ:   �����                                                        ==//
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