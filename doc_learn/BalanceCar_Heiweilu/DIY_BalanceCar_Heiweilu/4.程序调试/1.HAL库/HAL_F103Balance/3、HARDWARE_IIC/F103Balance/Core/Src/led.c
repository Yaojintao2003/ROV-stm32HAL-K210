#include "led.h"

void LedFlash(uint16_t Ticks)
{
	static uint16_t Count = 0;
	if(Count++ >= Ticks)
	{
		Count = 0;
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	}
}
