#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/

#define KEY PCin(15)

void KEY_Init(void);          //按键初始化
int KEY_Scan(void);          //按键扫描

#endif  
