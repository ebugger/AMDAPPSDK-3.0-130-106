/**********************************************************************
Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

//#include <CommonRoutines.h>

/* 
** a genericAddressSpace routine, being called
*  by 2 different convolution kernels with 
*  different address space
**
*  @param src  a pointer to input data in generic address space
*  @param filter a pointer to filterDara in constant address space
*  @param filterDim Dimension of convolution filter
*  @param width width of given input image data
*  @return a convoultion sum of given data for the given filterDim size
*/

float4 addMul2D(uchar4 *src, __constant float *filter, int2 filterDim, int width)
{
	int i, j;
	float4 sum = (float4)(0);
	
	for(i = 0; i < (filterDim.y); i++)
	{
		for(j = 0; j < (filterDim.x); j++)
		{
			sum += (convert_float4(src[(i*width)+j]))*((float4)(filter[(i*filterDim.x)+j]));
		}
	}

	return sum;
}

/************************* Convolution Operations ************************/
/*Kernel I*/
/**
 *  @brief    Perform Convolution operation on a 2D array using global address space
 *  @param    inputImageData input buffer stores input image data
 *  @param    outputImageData output buffer stores output image data
 *  @param	  height number of rows
 *  @param	  width number of columns
 *  @param    filterData an array of float
 *  @param    filterHeight height of filterdata
 *  @param    filterWidth width of filterData
*/


__kernel 
void convolution2DUsingGlobal(
		__global uchar4* inputImageData,
		__global uchar4* outputImageData,
		int height, int width,
		__constant float* filterData,
		int filterHeight,
		int filterWidth
		)
{
	int globalCol = get_global_id(0);
	int globalRow = get_global_id(1);

	int2 filterDim = {filterWidth, filterHeight};
	int2 filterRadius = filterDim/2;
	int2 padding = filterRadius*2;

	// Perform the convolution                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
	if((globalCol < (width - padding.x)) && (globalRow < (height - padding.y)))
	{
		float4 sum = (float4)(0);
		
		// call generic function with global address space
		sum = addMul2D(&inputImageData[(globalRow*width) + (globalCol)], filterData, filterDim, width);

		// write output data
		outputImageData[(globalRow+filterRadius.y)*width + (globalCol+filterRadius.x)] = (convert_uchar4_sat)(sum);
	}
}



/************************* (Convolution+sepiaToning) Operations ************************/
/*Kernel II*/
/**
 *  @brief    Perform sepiaToning filter operation 
 *            on a convoluted image which has been computed
 *            using local address space
 *  @param    inputImageData input buffer stores input image data
 *  @param    outputImageData output buffer stores output image data
 *  @param	  height  number of rows
 *  @param	  width number of columns
 *  @param    filterData an array of float
 *  @param    filterHeight height of filterdata
 *  @param    filterWidth width of filterData
 *  @param    localImage a shared array to store input image data
 *  @param    localHeight height of filterData
 *  @param    localWidth width of filterData
*/


__kernel 
void sepiaToning2DUsingLocal(
		__global uchar4* inputImageData,
		__global uchar4* outputImageData,
		int height, int width,
		__constant float* filterData,
		int filterHeight,
		int filterWidth,
		__local uchar4* localImage,
		int localHeight,
		int localWidth
		)
{

	int globalCol = get_global_id(0);
	int globalRow = get_global_id(1);
	int localCol = get_local_id(0);
	int localRow = get_local_id(1);
	int groupStartCol = get_group_id(0)*get_local_size(0);
	int groupStartRow = get_group_id(1)*get_local_size(1);

	int2 filterDim = {filterWidth, filterHeight};
	int2 filterRadius = filterDim/2;
	int2 padding = filterRadius*2;

	// Copy image into local address space
	for(int i = localRow; i < localHeight; i += get_local_size(1))
	{
		int curRow = groupStartRow+i;
		for(int j = localCol; j < localWidth; j+=get_local_size(0))
		{
			int curCol = groupStartCol+j;

			// perform the read if it is in bounds
			if(curRow < height && curCol < width)
				localImage[i*localWidth+j] = inputImageData[curRow*width+curCol];
		} 
	}
	work_group_barrier(CLK_LOCAL_MEM_FENCE);

	// Perform the convolution                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
	if((globalCol < (width - padding.x)) && (globalRow < (height - padding.y)))
	{
		float4 sum = (float4)(0);
		uchar4 temp;
		int id = (globalRow+filterRadius.y)*width + (globalCol+filterRadius.x);

		// call generic function with local address space
		sum = addMul2D(&localImage[(localRow*localWidth) + (localCol)], filterData, filterDim, localWidth);
		temp = (convert_uchar4_sat)(sum);

		// Now Performing the sepiaToning on convoluted image
		int red, green, blue;

		red = temp.x;
		green = temp.y;
		blue = temp.z;

		double grayscale = (0.3 * red + 0.59 * green + 0.11 * blue);
		double depth = 1.8;

		red = (int)(grayscale + depth * 56.6 );
		if (red > 255)
		{
		   red = 255;
		}

		green = (int)(grayscale + depth * 33.3 );
		if (green > 255)
		{
			green = 255;
		}

		blue = (int)(grayscale + depth * 10.1);
		if (blue > 255)
		{
			blue = 255;
		}

		// write output data
		outputImageData[id].x = red;
		outputImageData[id].y = green;
		outputImageData[id].z = blue;
	}
}
