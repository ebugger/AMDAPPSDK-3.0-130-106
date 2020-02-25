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

#define ENQUEUE_KERNEL_FAILURE_LEVEL1 (-5)
#define ENQUEUE_KERNEL_FAILURE_LEVEL2 (-10)
#define ENQUEUE_KERNEL_FAILURE_LEVEL3 (-15)


unsigned int findStages(unsigned int data_size, unsigned int wg_size)
{
  if (data_size <= wg_size)
    return 0;
  
  //assumption is that work group size is a power of two
  unsigned int log2wg = 0;

  unsigned int n = 1;
  while (n < wg_size)
    {
      log2wg += 1;
      n <<= 1;
    }
  
  unsigned int log2data = 0;

  n = 1;
  while (n < data_size)
    {
      log2data += 1;
      n <<= 1;
    }
  
  return (unsigned int)(log2data - log2wg); 
}

int isPrime(int number) {
    int i;
    if (number == 0 || number == 1) return 0;
    for (i=2; i*i<=number; i++) {
        if (number % i == 0) return 0;
    }
    return 1;
}

/***
 * get_primes_kernel_enqueue:
 * this kernel fills output array to reflect the entry in
 * input array is prime
***/
void get_primes_kernel_enqueue(global int *in,
global int *primes,
global int *output,
global int *out)
{
  int id = get_global_id(0);
  int k = output[id] - 1;

  if (id == 0) {
	if (output[id] == 1) 
		out[k] = in[id];
	return;
	}

  if (output[id-1] == k)
       out[k] = in[id];

}


/***
 * global_scan_kernel_enqueue:
 * takes a work-group scanned array [out] from group_scan_kernel and gives globally scanned array. 
 ***/
void global_scan_kernel_enqueue(global int *in,
global int *primes,
global int *output,
global int *out,
unsigned int globalThreads,
unsigned int stage,
unsigned int stages,
size_t localThreads,
__global uint4 *errorBuffer )
{
  unsigned int l;
  unsigned int vlen;

  unsigned int prev_gr,prev_el;
  unsigned int curr_gr, curr_el;

  
  int    add_elem;
  
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
    add_elem    = output[prev_el];

    
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);
  
  add_elem = work_group_broadcast(add_elem,0);
  
  /* find the array to which the element to be added */
  curr_gr = prev_gr + 1 + (grid % vlen);
  curr_el = curr_gr*szgr + lid;
  output[curr_el] += add_elem;

   if((stage == (stages-1)) && ((lid == 0) && (grid==0)))
   {
  		queue_t defQ = get_default_queue();
			  
		ndrange_t ndrange1 = ndrange_1D(globalThreads,localThreads);

		/**** Kernel Block *****/
		void (^blk)(void) = ^{get_primes_kernel_enqueue(in,
			primes, 
			output,
			out);};
					
		int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,blk);

		if(err_ret != 0)
		{
			errorBuffer[0].w = ENQUEUE_KERNEL_FAILURE_LEVEL3;
			errorBuffer[0].z = err_ret;
			return;
		}
		
  }

}

/***
 * group_scan_kernel_enqueue:
 * this kernel takes an input array [in] and produces a work-group level scan in [out].
 ***/
void group_scan_kernel_enqueue(global int *in,
global int *primes,
global int *output,
global int *out,
size_t globalThreads,
unsigned int length,
size_t localThreads,
__global uint4 *errorBuffer )
{
  int  in_data;

  int i = get_global_id(0);
	 
  in_data = primes[i];
  output[i]  = work_group_scan_inclusive_add(in_data);

  if(i == 0)
  {
		queue_t defQ = get_default_queue();
			  
		unsigned int  stages = (unsigned int)findStages(length,localThreads);

	if(stages >= 1)
	{
		unsigned int stage = 0;
		for(unsigned int stage = 0 ; stage < stages ; stage++)
		{

			ndrange_t ndrange1 = ndrange_1D(globalThreads/2,localThreads);
			/**** Kernel Block *****/
			void (^blk)(void) = ^{global_scan_kernel_enqueue(in,
				primes, 
				output,
				out,
				globalThreads,
				stage,
				stages,
				localThreads,
				errorBuffer);};
					
			int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,blk);

			if(err_ret != 0)
			{
				errorBuffer[0].w = ENQUEUE_KERNEL_FAILURE_LEVEL2;
				errorBuffer[0].z = err_ret;
				return;
			}
		}
	}
	else
	{
		queue_t defQ = get_default_queue();
			  
		ndrange_t ndrange1 = ndrange_1D(globalThreads,localThreads);

		/**** Kernel Block *****/
		void (^blk)(void) = ^{get_primes_kernel_enqueue(in,
			primes, 
			output,
			out);};
					
		int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,blk);

		if(err_ret != 0)
		{
			errorBuffer[0].w = ENQUEUE_KERNEL_FAILURE_LEVEL2;
			errorBuffer[0].z = err_ret;
			return;
		}
		
	}
		
  }

}

/***
 * extractPrime Kernle **
 * this kernel fills the boolean primes array to reflect the entry in
 * input array is prime
***/
kernel void extract_primes_kernel(global int *in,
global int *primes,
global int *output,
global int *out,
size_t globalThreads,
unsigned int length,
size_t localThreads,
__global uint4 *errorBuffer )
{
  int id = get_global_id(0);

  primes[id] = 0;
  if (isPrime(in[id]))
        primes[id] = 1;
		
  if(id == 0)
  {
		queue_t defQ = get_default_queue();
			  
		ndrange_t ndrange1 = ndrange_1D(globalThreads,localThreads);

		/**** Kernel Block *****/
		void (^blk)(void) = ^{group_scan_kernel_enqueue(in,
			primes, 
			output,
			out,
			globalThreads,
			length,
			localThreads,
			errorBuffer);};
					
		int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,blk);

		if(err_ret != 0)
		{
			errorBuffer[0].w = ENQUEUE_KERNEL_FAILURE_LEVEL1;
			errorBuffer[0].z = err_ret;
			return;
		}
		
  }
 
}


/***
 * set_primes_kernel:
 * this kernel fills the boolean primes array to reflect the entry in
 * input array is prime
***/
kernel void set_primes_kernel(global int *in, global int *primes)
{
  int id = get_global_id(0);

  primes[id] = 0;
  if (isPrime(in[id]))
        primes[id] = 1;

}

/***
 * get_primes_kernel:
 * this kernel fills output array to reflect the entry in
 * input array is prime
***/
kernel void get_primes_kernel(global int *in, global int *output, global int *outPrimes)
{
  int id = get_global_id(0);
  int k = output[id] - 1;

  if (id == 0) {
	if (output[id] == 1) 
		outPrimes[k] = in[id];
	return;
	}

  if (output[id-1] == k)
       outPrimes[k] = in[id];

}

/***
 * group_scan_kernel:
 * this kernel takes an input array [in] and produces a work-group level scan in [out].
 ***/
kernel void group_scan_kernel(global int *in, global int *out)
{
  int  in_data;

  int i = get_global_id(0);
	 
  in_data = in[i];
  out[i]  = work_group_scan_inclusive_add(in_data);
}

/***
 * global_scan_kernel:
 * takes a work-group scanned array [out] from group_scan_kernel and gives globally scanned array. 
 ***/
kernel void global_scan_kernel(global int *out, unsigned int stage)
{
  unsigned int l;
  unsigned int vlen;

  unsigned int prev_gr,prev_el;
  unsigned int curr_gr, curr_el;
  
  int    add_elem;
  
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


