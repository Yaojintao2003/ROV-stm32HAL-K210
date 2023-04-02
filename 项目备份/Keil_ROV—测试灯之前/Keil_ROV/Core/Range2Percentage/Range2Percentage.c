int Range2Percentage(int First, int Last, float Percent)
{
	return ((Last-First)*Percent/100+First);
}