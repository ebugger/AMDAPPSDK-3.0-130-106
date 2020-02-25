/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
/**
* @brief Filter the input data based on the Filter size. 
* @param input   Input data
* @param output  Filtered data
* @param N       Filter size
*/
__kernel void boxFilter(__global uint4* input,
                        __global uchar4* output,
                        uint N)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int width = get_global_size(0);
    int height = get_global_size(1);
    int k = (N - 1) / 2;
    if((x < k) || (y < k) || (x > (width - k - 1)) ||( y > (height - k - 1)))
    {
        output[(x + y * width)] = (uchar4)(0);
        return;
    }

    /* N should be an odd number */
    int filterSize = (N * N);
    int2 posA = (int2)(x - k, y - k);
    int2 posB = (int2)(x + k, y - k);
    int2 posC = (int2)(x + k, y + k);
    int2 posD = (int2)(x - k, y + k);

    int4 sumA = (int4)0;
    int4 sumB = (int4)0;
    int4 sumC = (int4)0;
    int4 sumD = (int4)0;

    /* Shift coordinates to get corresponding values */
    posA.x -= 1;
    posA.y -= 1;
    posB.y -= 1;
    posD.x -= 1;

    if(posA.x >= 0 && posA.y >= 0)
    {
        sumA = convert_int4(input[posA.x + posA.y * width]);
    }
    if(posB.x >= 0 && posB.y >= 0)
    {
        sumB = convert_int4(input[posB.x + posB.y * width]);
    }
    if(posD.x >= 0 && posD.y >= 0)
    {
        sumD = convert_int4(input[posD.x + posD.y * width]);
    }
    sumC = convert_int4(input[posC.x + posC.y * width]);

	/* Using (x*height + y) for output index calculation instead (x + y*width) since input is transposed data from the previous vertical scan step*/
    output[x * height + y] = (convert_uchar4_sat)((sumA + sumC - sumB - sumD) / filterSize);
}
