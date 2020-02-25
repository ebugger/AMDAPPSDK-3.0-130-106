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
#define NBINS 256
#define NBANKS 16

/************************* Generate Histogram ************************/
/*Kernel I*/
/**
 *  @brief    Generate block-histogram bin whose bin size is 256
 *  @param    inputImageData input buffer stores image data
 *  @param    sharedArray shared array for thread-histogram bins
 *  @param    binResult block-histogram array
 *  @param    nPixelsPerThread an integer variable used to make sure that each thread accesses nPixelsPerThread number of pixels
*/

__kernel 
void imageHistogram256(
		__global uchar* inputImageData,
		__local uint* sharedArray,
        __global uint* binResult,
		int nPixelsPerThread)                      
{
	size_t ltd = get_local_id(0);
    size_t gid = get_global_id(0);
    size_t groupId = get_group_id(0);
    size_t groupSize = get_local_size(0);
	size_t globalSize = get_global_size(0);

	// initialize shared array to zero
	uint lmem_items = NBANKS * NBINS;
	uint lmem_max_threads = groupSize;
    uint lmem_items_per_thread = lmem_items/lmem_max_threads;
	uint idx;
	int i;

	__local uint * input = (__local uint*)sharedArray;
	if( ltd < lmem_max_threads )
    {
       for(i=0, idx=ltd; i<lmem_items_per_thread; i++, idx+=lmem_max_threads)
       {
          input[idx] = 0;
       }
    }
    barrier(CLK_LOCAL_MEM_FENCE);

	int temp1, temp2;
	int offset = ltd % NBANKS;
	for(i = 0, idx = gid; i < nPixelsPerThread; i++, idx+=globalSize)
	{
		temp1 = (int)(inputImageData[idx]);
		temp2 = (temp1 * NBANKS) + offset;
		(void) atom_inc(sharedArray + temp2);
	}

    work_group_barrier(CLK_LOCAL_MEM_FENCE); 

	// reduce __local banks to single histogram per work-group
    if( ltd < NBINS )
    {
       uint bin = 0;

       for( i=0; i<NBANKS; i++ )
       {
          bin += sharedArray[ (ltd * NBANKS) + i ];
       }

       binResult[(groupId * NBINS) + ltd] = bin;
    }
}


/************************* Image Binarization ************************/
/*Kernel III*/
/**
 *  @brief    Binarization of a graylevel Image
 *  @param    image  a 2d image, used as input as well as output
 *  @param    threshold  a Otsu's threshold input value
*/

__constant sampler_t imageSampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST; 

__kernel 
void imageBinarization(
		__read_write image2d_t image,
		int threshold)                          
{
	// Store each work-item's unique row and column
	int2 coord = (int2)(get_global_id(0), get_global_id(1));

	// Read pixel from image
	uint4 temp = read_imageui(image, imageSampler, coord);

	// Perform Image Thresholding on each pixel
	temp.x = select(255, 0, (uint)(temp.x < threshold));

	// Write modified pixel back to original image
	write_imageui(image, coord, temp);
}

