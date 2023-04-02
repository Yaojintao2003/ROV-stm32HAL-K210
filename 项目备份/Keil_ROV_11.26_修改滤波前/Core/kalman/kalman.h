#ifndef __KALMAN_H__
#define __KALMAN_H__

typedef struct
{
    /*���ö�*/
    double LastP;//�ϴι���Э����
    double Now_P;//��ǰ����Э����
    double out;//�������˲������
    double Kg;//����������
	  double Q;
	  double R;
}Kalman_Typedef;

void Kalman_Init(Kalman_Typedef *klm, const double klm_Q, const double klm_R);
double KalmanFilter(Kalman_Typedef *klm, double input);

#endif