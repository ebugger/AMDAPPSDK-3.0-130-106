/*****************************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this l
ist of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#define  MAX_RGB         255
#define  MIN_RGB         0

#define  GPU_UPDATE      0  
#define  CPU_UPDATE      1  

/***
 * laplacian:
 ***/
__kernel void pdeKernel(unsigned int          sizex,
			unsigned int          sizey,
			__global float*       cField,
			__global void*        ping,
			__global void*        pong,
			volatile __global atomic_int* controlField)
{
  int x     = get_global_id(0) + 1;
  int y     = get_global_id(1) + 1;
  int sizeX = get_global_size(0);
  int sizeY = get_global_size(1);
  
  int exSizeX = sizeX + 2;
  int exSizeY = sizeY + 2;

  int c = y*exSizeX + x;
  int t = (y-1)*exSizeX + x;
  int l = c -1;
  int b = (y+1)*exSizeX + x;
  int r = c +1;

  float *prevField     = (float *)ping;
  float *nextField     = (float *)pong;
  
  //heat equation
  float laplacian = prevField[t] + prevField[l] +  
                    prevField[b] + prevField[r];  

  laplacian = laplacian - (float)4.0*prevField[c];

  int val = atomic_load_explicit(&controlField[c], memory_order_seq_cst, memory_scope_all_svm_devices);

  if (val == GPU_UPDATE)
	nextField[c] = cField[c]*laplacian + prevField[c];
  else
	nextField[c] = prevField[c]; 
}           


__kernel void tempToRgbKernel(unsigned int           sizex,
			      unsigned int           sizey,
			      __global void*         heatField,
			      __global unsigned int* rgbValue)
{
  int x = get_global_id(0);
  int y = get_global_id(1);

  int c = y*(sizex +2) + x;

  float*        field         = (float *)heatField;

  float         t1 = field[c]/1000.0;

  float         fr, fg, fb;

  /* red */
  if(t1 < 6.000)
    fr = 0.0f;
  else
    fr = (t1 - 6.0)/1.5;

  /* green */
  if(t1 < 7.500)
    fg = 0.0f;
  else
    fg = (t1 - 7.5)/2.0;

  /* green */
  if(t1 < 9.500)
    fb = 0.0f;
  else
    fb = (t1 - 9.5)/0.5;


  /* f to ub */
  unsigned int cr,cg,cb;

  if (fr < 0)
    cr = (unsigned int)MIN_RGB;
  if (fr >= 1.0f)
    cr = (unsigned int)MAX_RGB;
  else
    cr = (unsigned int)(fr*MAX_RGB);

  if (fg < 0)
    cg = (unsigned int)MIN_RGB;
  if (fg >= 1.0f)
    cg = (unsigned int)MAX_RGB;
  else
    cg = (unsigned int)(fg*MAX_RGB);

  if (fb < 0)
    cb = (unsigned int)MIN_RGB;
  if (fb >= 1.0f)
    cb = (unsigned int)MAX_RGB;
  else
    cb = (unsigned int)(fb*MAX_RGB);

  /* pack */
  rgbValue[c] = (cr << 16)|(cg << 8)|cb;
}
