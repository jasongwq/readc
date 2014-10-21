#include  "systemInit.h"


/*==============================================================================
1.Ԥ����
   X(k|k-1) = F(k,k-1)*X(k-1|k-1)        //������Ϊ0


2.����Ԥ����Э�������
   P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q(k)
   Q(k) = U(k)��U(k)' 


3.���㿨�����������
   Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R(k))
   R(k) = N(k)��N(k)' 


4.���¹���
   X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))


5.������º����Э�������
   P(k|k) =��I-Kg(k)*H��*P(k|k-1)


6. ��������ֵ


F(k,k-1):     ״̬ת�ƾ���
X(k|k-1):     ����k-1ʱ�̵�����ֵ����kʱ�̵�ֵ
X(k-1|k-1):   k-1ʱ�̵�����ֵ
P(k|k-1):     X(k|k-1)��Ӧ��covariance
P(k-1|k-1):   X(k-1|k-1)��Ӧ��covariance
Q(k):         ϵͳ���̵�covariance
R(k):         �������̵�Э����
H(k):         �۲����ת�ƾ���
Z(k):         kʱ�̵Ĳ���ֵ


����˼·: ���ȸ�����һ��(����ǵ�һ�������Ԥ��ֵ����)�����ݼ�������εĹ���ֵ,
          ͬ��,������һ�ε����ݼ�������ι���ֵ��Э����;  ����,�ɱ��ι���ֵ��Э
          ������������������;  ���,���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ����Э����
==============================================================================*/



//================================================//
//==             ����ֵ����ṹ��               ==//
//================================================//
typedef struct  _tCovariance
{
  float PNowOpt[LENGTH];
  float PPreOpt[LENGTH];
}tCovariance;



//================================================//
//==               ����ֵ�ṹ��                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
}tOptimal;



tOptimal      tOpt;
tCovariance   tCov;
float         Z[LENGTH]  = {4000};           //  ����ֵ(ÿ�β�����������Ҫ���������)
float         I[LENGTH]  = {1};              //  ��λ����
float         X[LENGTH]  = {0};              //  ��ǰ״̬��Ԥ��ֵ
float         P[LENGTH]  = {0};              //  ��ǰ״̬��Ԥ��ֵ��Э����
float         K[LENGTH]  = {0};              //  ����������
float         Temp3[LENGTH] = {0};           //  ��������
//============================================================================//
//==                    �������˲���Ҫ���õı���                            ==//
//============================================================================//
float         F[LENGTH]  = {1};              //  ״̬ת�ƾ���
float         Q[LENGTH]  = {0};              //  ϵͳ���̵�Э����
float         H[LENGTH]  = {1};              //  �۲����ת�ƾ���
float         R[LENGTH]  = {2};              //  �������̵�Э����
float         Temp1[LENGTH] = {1};           //  ��������, ͬʱ����tOpt.XPreOpt[]�ĳ�ʼ��ֵ
float         Temp2[LENGTH] = {10000};       //  ��������, ͬʱ����tCov.PPreOpt[]�ĳ�ʼ��ֵ





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
  unsigned short  k;
  
  for (i=0; i<LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp1[i];           //��ֵ��ʼ��
  }
  for (i=0; i<LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp2[i];           //��ֵ��ʼ��
  }
  
  
  for (k=0; k<N; k++)
  {
    //Z[0] = (float)ADCSampling();
    MatrixMul(F, tOpt.XPreOpt, X, ORDER, ORDER, ORDER);       //  ����ϵͳ����һ״̬��Ԥ������״̬; X(k|k-1) = F(k,k-1)*X(k-1|k-1)
    
    MatrixCal(F, tCov.PPreOpt, Temp1, ORDER);
    MatrixAdd(Temp1, Q, P, ORDER, ORDER);                     //  Ԥ�����ݵ�Э�������; P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q
    
    MatrixCal(H, P, Temp1, ORDER);
    MatrixAdd(Temp1, R, Temp1, ORDER, ORDER);
    Gauss_Jordan(Temp1, ORDER);
    MatrixTrans(H, Temp2, ORDER, ORDER);
    MatrixMul(P, Temp2, Temp3, ORDER, ORDER, ORDER);
    MatrixMul(Temp1, Temp3, K, ORDER, ORDER, ORDER);          //  ���㿨��������; Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)
    
    MatrixMul(H, X, Temp1, ORDER, ORDER, ORDER);
    MatrixMinus(Z, Temp1, Temp1, ORDER, ORDER);
    MatrixMul(K, Temp1, Temp2, ORDER, ORDER, ORDER);
    MatrixAdd(X, Temp2, tOpt.XNowOpt, ORDER, ORDER);          //  ���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))
    
    MatrixMul(K, H, Temp1, ORDER, ORDER, ORDER);
    MatrixMinus(I, Temp1, Temp1, ORDER, ORDER);
    MatrixMul(Temp1, P, tCov.PNowOpt, ORDER, ORDER, ORDER);   //  ������º����Э�������; P(k|k) =��I-Kg(k)*H��*P(k|k-1)
    
    for (i=0; i<LENGTH; i++)
    {
      tOpt.XPreOpt[i] = tOpt.XNowOpt[i];
      tCov.PPreOpt[i] = tCov.PNowOpt[i];
    }
  }
}





//============================================================================//
//==                    ����������̬�ֲ��������                            ==//
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
  
  srand(SEED);                        //��������
  
  for(j=0; j<Num; j++)                //ѭ�����������
  {
    Temp = rand();
    r = (Temp+0.00)/1073741823.00;
    S[j] = sqrt(-2*log(r))*cos(2*3.14159265*rand())*sigma+mu;
  }
}

