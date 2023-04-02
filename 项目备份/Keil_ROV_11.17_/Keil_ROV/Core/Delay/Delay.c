#include "main.h"

#define USE_HAL_LEGACY
#include "stm32_hal_legacy.h"

//#define Timebase_Source_is_SysTick 1
#define Timebase_Source_is_SysTick 0

#if	(!Timebase_Source_is_SysTick)
	extern TIM_HandleTypeDef htim11;
	#define Timebase_htim htim11

	#define Delay_GetCounter()		__HAL_TIM_GetCounter(&Timebase_htim)
	#define Delay_GetAutoreload()	__HAL_TIM_GetAutoreload(&Timebase_htim)
#else
	#define Delay_GetCounter()		(SysTick->VAL)
	#define Delay_GetAutoreload()	(SysTick->LOAD)
#endif

static uint16_t fac_us = 0;
static uint32_t fac_ms = 0;

void delay_init(void)
{
#if	(!Timebase_Source_is_SysTick)
	fac_ms = 1000000;				
	fac_us = fac_ms / 1000;
#else
	fac_ms = SystemCoreClock / 1000;
	fac_us = fac_ms / 1000;
#endif
}

void delay_us(uint32_t nus)
{
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;

	reload = Delay_GetAutoreload();

	ticks = nus * fac_us;

	told = Delay_GetCounter();

	while (1)
	{
		tnow = Delay_GetCounter();

		if (tnow != told)
		{
			if (tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if (tcnt >= ticks)
			{
				break;
			}
		}
	}
}

/*姣寤舵椂寤鸿浣跨敤HAL_Delay锛岀簿搴︽洿楂�*/
void delay_ms(uint32_t nms)
{
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;

	reload = Delay_GetAutoreload();

	ticks = nms * fac_ms;

	told = Delay_GetCounter();

	while (1)
	{
		tnow = Delay_GetCounter();

		if (tnow != told)
		{
			if (tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if (tcnt >= ticks)
			{
				break;
			}
		}
	}
}

/*姣寤舵椂寤鸿浣跨敤HAL_Delay锛岀簿搴︽洿楂�*/
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /*涓嶅お鏄庣櫧瀹樻柟婧愮爜涓哄暐杩欎箞鍐欙紝浼氬寤舵椂锛屾敞閲婃帀鍚庢洿鍑�*/
//  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}
