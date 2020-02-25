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

/***
 * group_scan_kernel:
 * this kernel takes an input array [in] and produces a work-group level scan in [out].
 ***/
__kernel void group_scan_kernel(__global float *in, __global float *out)
{
  float  in_data;

  int i = get_global_id(0);
	 
  in_data = in[i];
  out[i]  = work_group_scan_inclusive_add(in_data);
}

/***
 * global_scan_kernel:
 * takes a work-group scanned array [out] from group_scan_kernel and gives globally scanned array. 
 ***/
__kernel void global_scan_kernel(__global float *out, unsigned int stage)
{
  unsigned int l;
  unsigned int vlen;

  unsigned int prev_gr,prev_el;
  unsigned int curr_gr, curr_el;
  
  float    add_elem;
  
  int lid  = get_local_id(0);
  int grid = get_group_id(0);
  int szgr = get_local_size(0);
  
  /* array size to be processed */
  vlen = 1 << stage;
  
  /* find the element to be added */
  l       = (grid >> stage);
  prev_gr = l*(vlen << 1) + vlen - 1;
  prev_el = prev_gr*szgr + szgr - 1;
  
  if (lid == 0)
    add_elem    = out[prev_el];
  
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);
  
  add_elem = work_group_broadcast(add_elem,0);
  
  /* find the array to which the element to be added */
  curr_gr = prev_gr + 1 + (grid % vlen);
  curr_el = curr_gr*szgr + lid;
  out[curr_el] += add_elem;
}


