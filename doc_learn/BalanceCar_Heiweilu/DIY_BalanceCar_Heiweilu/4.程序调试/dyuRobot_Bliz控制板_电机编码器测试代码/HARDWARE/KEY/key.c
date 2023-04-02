#include "key.h"
/**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PC端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC
} 
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
int KEY_Scan(void)
{
			static u8 flag_key=1;//按键按松开标志
			if(flag_key&&KEY==0)
			{
				flag_key=0;
				return 1;	// 按键按下
			}
			else if(1==KEY)			
			{
				flag_key=1;
				return 0;//无按键按下
			}
}

