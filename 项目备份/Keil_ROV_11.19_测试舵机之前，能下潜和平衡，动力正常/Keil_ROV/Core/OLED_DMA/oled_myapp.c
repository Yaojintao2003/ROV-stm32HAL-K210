#include <stdio.h>
#include "../OLED_DMA/draw_api.h"

void oled_print_float(int x, int y, float num_float)
{
	unsigned char str_temp[12] = {0};
	snprintf(str_temp, 11, "%3.7f", num_float);
	DrawString(x, y, str_temp);
}


