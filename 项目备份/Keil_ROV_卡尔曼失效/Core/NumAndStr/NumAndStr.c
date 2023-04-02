/*
 * NumAndStr.c
 *
 *  Created on: Mar 15, 2021
 *      Author: Royic
 */
#include "../NumAndStr/NumAndStr.h"
#include  <stdlib.h>

int32_t str2int(uint8_t * str, uint8_t flag, uint8_t no)
{
	uint8_t No = 1;
	uint8_t * Str = str;
	uint8_t NumTemp[TempIntLen];
	while(No!=no)
	{
		if(*Str == flag)
			No++;
		Str++;
	}
	No = 0;
	while(*Str != flag && *Str != '\r' && *Str != '\n' && *Str != '\0' && No < (TempIntLen - 1))
	{
		NumTemp[No] = *Str;
		Str++;
		No++;
	}
	NumTemp[No] = '\0';
	return atoi(NumTemp);
}

void str2double(uint8_t * str, uint8_t flag, uint8_t no, double * Output)
{
	uint8_t No = 1;
	uint8_t * Str = str;
	uint8_t NumTemp[TempDoubleLen];
	uint8_t NumTemp_int[TempDoubleLen];
	double OutputNum;
	while(No!=no)
	{
		if(*Str == flag)
			No++;
		Str++;
	}
	No = 0;
	while(*Str != flag && *Str != '\r' && *Str != '\n' && *Str != '\0' && No < (TempDoubleLen - 1))
	{
		NumTemp[No] = *Str;
		Str++;
		No++;
	}
	NumTemp[No] = '\0';
	NumTemp[(TempDoubleLen - 1)] = 0;
	No = 0;
	while(NumTemp[NumTemp[(TempDoubleLen - 1)]] != '\0' && NumTemp[(TempDoubleLen - 1)] < (TempDoubleLen - 1))
	{
		if(NumTemp[NumTemp[(TempDoubleLen - 1)]] == '.')
		{
			NumTemp[(TempDoubleLen - 1)]++;
			NumTemp_int[(TempDoubleLen - 1)] = NumTemp[(TempDoubleLen - 1)];
		}
		NumTemp_int[No] = NumTemp[NumTemp[(TempDoubleLen - 1)]];
		No++;
		NumTemp[(TempDoubleLen - 1)]++;
	}
	NumTemp_int[No]='\0';
	NumTemp[(TempDoubleLen - 1)] = NumTemp_int[(TempDoubleLen - 1)]++;
	OutputNum = (double)atoi(NumTemp_int);
	while(NumTemp[NumTemp[(TempDoubleLen - 1)]] != '\0')
	{
		OutputNum /= 10;
		NumTemp[(TempDoubleLen - 1)] ++;
	}
	*Output = OutputNum;
}
