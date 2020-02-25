/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

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
#define STRIDE 512
#define NUM_READS 32

__kernel void read_linear(__global DATATYPE *input,__global DATATYPE *output)
{
	DATATYPE val = (DATATYPE)(0.0f);
	IDXTYPE gid = get_global_id(0);

	val = val + input[gid + 0];
	val = val + input[gid + 1];
	val = val + input[gid + 2];
	val = val + input[gid + 3];
	val = val + input[gid + 4];
	val = val + input[gid + 5];
	val = val + input[gid + 6];
	val = val + input[gid + 7];
	val = val + input[gid + 8];
	val = val + input[gid + 9];
	val = val + input[gid + 10];
	val = val + input[gid + 11];
	val = val + input[gid + 12];
	val = val + input[gid + 13];
	val = val + input[gid + 14];
	val = val + input[gid + 15];
	val = val + input[gid + 16];
	val = val + input[gid + 17];
	val = val + input[gid + 18];
	val = val + input[gid + 19];
	val = val + input[gid + 20];
	val = val + input[gid + 21];
	val = val + input[gid + 22];
	val = val + input[gid + 23];
	val = val + input[gid + 24];
	val = val + input[gid + 25];
	val = val + input[gid + 26];
	val = val + input[gid + 27];
	val = val + input[gid + 28];
	val = val + input[gid + 29];
	val = val + input[gid + 30];
	val = val + input[gid + 31];

	output[gid] = val;
}

__kernel void read_linear_uncached(__global DATATYPE *input,__global DATATYPE *output)
{
	DATATYPE val = (DATATYPE)(0.0f);
	IDXTYPE gid = get_global_id(0);
	IDXTYPE index = gid;

	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];
	val = val + input[index += OFFSET];

	output[gid] = val;
}

__kernel void read_single(__global DATATYPE *input,__global DATATYPE *output)
{
	DATATYPE val = (DATATYPE)(0.0f);
	IDXTYPE gid = get_global_id(0);

	val = val + input[0];
	val = val + input[1];
	val = val + input[2];
	val = val + input[3];
	val = val + input[4];
	val = val + input[5];
	val = val + input[6];
	val = val + input[7];
	val = val + input[8];
	val = val + input[9];
	val = val + input[10];
	val = val + input[11];
	val = val + input[12];
	val = val + input[13];
	val = val + input[14];
	val = val + input[15];
	val = val + input[16];
	val = val + input[17];
	val = val + input[18];
	val = val + input[19];
	val = val + input[20];
	val = val + input[21];
	val = val + input[22];
	val = val + input[23];
	val = val + input[24];
	val = val + input[25];
	val = val + input[26];
	val = val + input[27];
	val = val + input[28];
	val = val + input[29];
	val = val + input[30];
	val = val + input[31];

	output[gid] = val;
}

__kernel void write_linear(__constant DATATYPE *c0, __global DATATYPE *output)
{
	IDXTYPE gid = get_global_id(0);

	output[gid + 0 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 1 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 2 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 3 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 4 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 5 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 6 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 7 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 8 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 9 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 10 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 11 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 12 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 13 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 14 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 15 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 16 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 17 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 18 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 19 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 20 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 21 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 22 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 23 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 24 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 25 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 26 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 27 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 28 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 29 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 30 * (STRIDE / sizeof(DATATYPE))] =  *c0;
	output[gid + 31 * (STRIDE / sizeof(DATATYPE))] =  *c0;
}

__kernel void read_random(__global DATATYPE2 *input, __global DATATYPE2 *output)
{
	IDXTYPE gid = get_global_id(0);
	IDXTYPE Index = get_local_id(0);
	
    DATATYPE2 val = (DATATYPE2)(0.0f);
    DATATYPE2 midval = (DATATYPE2)(0.0f);
   
    for(uint i = 0; i<32;i++)
    {
		midval = input[Index];
		val += midval;
		Index = (uint)(midval.S0);
    }
    
	output[gid] = val;	
}

__kernel void read_random1(__global float *input, __global float *output)
{
	IDXTYPE gid = get_global_id(0);
	IDXTYPE Index = get_local_id(0);
	
    float val = (float)(0.0f);
    float midval = (float)(0.0f);
    
    for(uint i = 0; i<32;i++)
    {
		midval = input[Index];
		val += midval;
		Index = (uint)(midval);
    }
    
	output[gid] = val;
}


__kernel void read_uncoalescing(__global DATATYPE *input,__global DATATYPE *output)
{
    DATATYPE val = (DATATYPE)(0.0f);
    IDXTYPE gid = get_global_id(0);

    val = val + input[gid * NUM_READS + 0];
    val = val + input[gid * NUM_READS + 1];
    val = val + input[gid * NUM_READS + 2];
    val = val + input[gid * NUM_READS + 3];
    val = val + input[gid * NUM_READS + 4];
    val = val + input[gid * NUM_READS + 5];
    val = val + input[gid * NUM_READS + 6];
    val = val + input[gid * NUM_READS + 7];
    val = val + input[gid * NUM_READS + 8];
    val = val + input[gid * NUM_READS + 9];
    val = val + input[gid * NUM_READS + 10];
    val = val + input[gid * NUM_READS + 11];
    val = val + input[gid * NUM_READS + 12];
    val = val + input[gid * NUM_READS + 13];
    val = val + input[gid * NUM_READS + 14];
    val = val + input[gid * NUM_READS + 15];
    val = val + input[gid * NUM_READS + 16];
    val = val + input[gid * NUM_READS + 17];
    val = val + input[gid * NUM_READS + 18];
    val = val + input[gid * NUM_READS + 19];
    val = val + input[gid * NUM_READS + 20];
    val = val + input[gid * NUM_READS + 21];
    val = val + input[gid * NUM_READS + 22];
    val = val + input[gid * NUM_READS + 23];
    val = val + input[gid * NUM_READS + 24];
    val = val + input[gid * NUM_READS + 25];
    val = val + input[gid * NUM_READS + 26];
    val = val + input[gid * NUM_READS + 27];
    val = val + input[gid * NUM_READS + 28];
    val = val + input[gid * NUM_READS + 29];
    val = val + input[gid * NUM_READS + 30];
    val = val + input[gid * NUM_READS + 31];

    output[gid] = val;
}