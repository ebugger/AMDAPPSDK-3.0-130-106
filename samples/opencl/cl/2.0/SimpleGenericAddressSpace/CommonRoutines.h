float addMul2D(float *src, float *filter, int2 filterDim, int cols)
{
	int i, j;
	float sum = 0.0f;
	
	for(i = 0; i < (filterDim.y); i++)
	{
		for(j = 0; j < (filterDim.x); j++)
		{
			sum += (src[(i*cols)+j])*(filter[(i*filterDim.x)+j]);
		}
	}

	return sum;
}