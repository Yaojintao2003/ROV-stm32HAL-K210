#ifndef _move_filter_h
#define _move_filter_h

#include "common.h"



#define MOVE_AVERAGE_SIZE   8  //���建������С


typedef struct
{
	uint8 index;            //�±�
    uint8 buffer_size;      //buffer��С
	int32 data_buffer[MOVE_AVERAGE_SIZE];  //������
	int32 data_sum;         //���ݺ�
	int32 data_average;     //����ƽ��ֵ
}move_filter_struct;









extern move_filter_struct speed_filter;


void move_filter_init(move_filter_struct *move_average);
void move_filter_calc(move_filter_struct *move_average, int32 new_data);



#endif

