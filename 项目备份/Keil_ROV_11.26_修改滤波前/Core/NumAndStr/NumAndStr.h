/*
 * NumAndStr.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Royic
 */

#ifndef NUMANDSTR_NUMANDSTR_H_
#define NUMANDSTR_NUMANDSTR_H_

#include "main.h"

#define TempDoubleLen 18
#define TempIntLen 11

/*
str:数字字符串首地址
flag:分隔符
no:第no个数字 从1开始计
Output: 小数存放地址
 */
extern int32_t str2int(uint8_t * str, uint8_t flag, uint8_t no);
extern void str2double(uint8_t * str, uint8_t flag, uint8_t no, double * Output);

#endif /* NUMANDSTR_NUMANDSTR_H_ */
