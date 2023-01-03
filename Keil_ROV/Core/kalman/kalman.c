/*
�������˲���
����By ������ 2018211769
*/
#include "../kalman/kalman.h"

double KalmanFilter(Kalman_Typedef *klm, double input)
{
    //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    klm->Now_P = klm->LastP + klm->Q;
    //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    klm->Kg = klm->Now_P / (klm->Now_P + klm->R);
    //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    klm->out = klm->out + klm->Kg * (input -klm->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
    //����Э�����: ���ε�ϵͳЭ����� klm->LastP Ϊ��һ������׼����
    klm->LastP = (1-klm->Kg) * klm->Now_P;
	
	return (klm->out);
}

void Kalman_Init(Kalman_Typedef *klm, const double klm_Q, const double klm_R)//�¶�klm_Q=0.01 klm_R=0.25
{
	klm->LastP=0.02;		//�ϴι���Э����
	klm->Now_P=0;			//��ǰ����Э����
	klm->out=1;				//�������˲������
	klm->Kg=0;				//����������
	klm->Q=klm_Q;			//Q:��������Э���� Q�������˲��������ƽ���̶ȣ�QԽСԽƽ��;
	klm->R=klm_R;			//R:�۲�����Э���� R���������˲����������ʵ�����ߵ�����̶ȣ�RԽСԽ�ӽ�(����Խ��)
}