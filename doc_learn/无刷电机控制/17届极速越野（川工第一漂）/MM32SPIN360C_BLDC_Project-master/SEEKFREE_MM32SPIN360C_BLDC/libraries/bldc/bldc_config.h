#ifndef _bldc_config_h
#define _bldc_config_h

#define break_time  10
#define break_time_duty 2

//0��������ɲ������     1������
#define BLDC_BRAKE_ENABLE       1       

//0���������ٶȱջ�     1������
#define BLDC_CLOSE_LOOP_ENABLE  1       

//ֻ���ڱջ�������ʱ���ٶ����߲��ܱ�����
#if BLDC_CLOSE_LOOP_ENABLE==1      
    //�˺궨��������ʵ�ֵ����������
    //0���������ٶ��������� 1������
    #define BLDC_SPEED_CURVE_ENABLE 1
    
    //�������ת��
    #define BLDC_MAX_SPEED          55000   

    //BLDC_STEP_SPEED������ʹ�õ���������٣�������ֵ�������ģ�ֵԽС�������Խ��
    //�������Ҫ����ͨ����BLDC_SPEED_CURVE_ENABLE����Ϊ0�����ر��������
    #define BLDC_STEP_SPEED         20      

#endif



//�����ܵ�����ֵ����ĸ�ߵ����������õ���ֵ���Զ��ر�pwm��� �����÷�Χ0-15 ֵԽС����Խǿ
#define BLDC_FAULT_THRESHOLD    15
   










#endif

