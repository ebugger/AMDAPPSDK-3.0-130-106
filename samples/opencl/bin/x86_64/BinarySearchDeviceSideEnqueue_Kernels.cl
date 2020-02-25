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

/**
 * One instance of this kernel call is a thread.
 * Each thread finds out the segment in which it should look for the element.
 * After that, it checks if the element is between the lower bound and upper bound
 * of its segment. If yes, then this segment becomes the total searchspace for the next pass.
 *
 * To achieve this, it writes the lower bound and upper bound to the output array.
 * In case the element at the left end (lower bound) matches the element we are looking for,
 * That is marked in the output and enqueue a child kernel if the subdivSize is equal or more than global threads .
 */

#ifdef CL_VERSION_2_0
 #define ENQUEUE_KERNEL_FAILURE (5)

 /** Flag  used to launch the child kernel if at-least one key is found in the parent kernel execution
 */
 __global atomic_int keysHit = ATOMIC_VAR_INIT(0);
 

 /**  binarySearch_device_enqueue_Multikeys_Level2 => This function is used as child kernel, slightly varied 
	  from the parent kernel to avoid certain issues like global memory synchronization	.
 */  

void
binarySearch_device_enqueue_multiKeys_child(__global uint4 * outputArray,
             __global uint  * sortedArray, 
			 unsigned int subdivSize,
			 __global uint *globalLowerIndex,
             __global uint  *keys,
			 unsigned int nKeys,
			 __global uint * parentGlobalids,
			  unsigned int globalThreads
			 );
 
/** Implementation of the Binary Search algorithm Using Device-Side Kernel enqueue Feature of
  * OpenCL 2.0 Standard .
 */  
__kernel void
binarySearch_device_enqueue_multiKeys(__global uint4 * outputArray,
             __global uint  * sortedArray, 
			 unsigned int subdivSize,
			 __global uint *globalLowerIndex,
             __global uint  *keys,
			 unsigned int nKeys,
			 __global uint * parentGlobalids,
			  unsigned int globalThreads
			 )			
{
    unsigned int tid = get_global_id(0);
	int keyCount =0 ;
	   

	for(keyCount=0;keyCount<nKeys;keyCount++)
	{
		 /* Then we find the elements  for this thread */
		int elementLower = sortedArray[globalLowerIndex[keyCount] + tid * subdivSize];
		int elementUpper = sortedArray[globalLowerIndex[keyCount] + (tid + 1) * subdivSize - 1];

		/* If the element to be found does not lie between them, then nothing left to do in this thread */
		if( (elementLower > keys[keyCount]) || (elementUpper < keys[keyCount])) 
		{
			continue;
		}
		else
		{
			/* However, if the element does lie between the lower and upper bounds of this thread's searchspace
			 * we need to narrow down the search further in this search space 
			 */
			unsigned int globalLowerTemp = tid * subdivSize;
						
			/*** Calculating the Lower bound of the Keys ****/	
		   	  outputArray[keyCount].x = parentGlobalids[keyCount] + globalLowerTemp;
			  parentGlobalids[keyCount]  += globalLowerTemp ;

			/****** Used for Error Checks *******/
			  outputArray[keyCount].w = 1;
			
			/****** Pass the new lower-index for each key for next kernel launch *******/
			  globalLowerIndex[keyCount]  = globalLowerTemp;

		    /******  Pass the final subdivSize to the Host ***/
			  outputArray[keyCount].y = subdivSize;

            /*** Used atomics to ensure memory consistency ***/ 
			 atomic_store_explicit(&keysHit,1,memory_order_seq_cst);
		
		}
	}

	/*** child kernel is launched for one time ***/
	if(tid == 0)
	{
		
			
		queue_t defQ = get_default_queue();
			  
		/*** If the Search space is too small return to the host since it is not suitable to use GPU ***/ 
		if(subdivSize < globalThreads)
		{
		return;
		}

		/**** Narrow-down the search space ******/
		subdivSize = subdivSize/globalThreads ; 

			
		ndrange_t ndrange1 = ndrange_1D(globalThreads);

		/**** Kernel Block *****/
		void (^binarySearch_device_enqueue_wrapper_blk)(void) = ^{binarySearch_device_enqueue_multiKeys_child(outputArray,
			sortedArray, 
			subdivSize,
			globalLowerIndex,
			keys
			,nKeys
			,parentGlobalids,globalThreads);};
					
		int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,binarySearch_device_enqueue_wrapper_blk);

		if(err_ret != 0)
		{
			outputArray[keyCount].w = ENQUEUE_KERNEL_FAILURE;
			outputArray[keyCount].z = err_ret;
			return;
		}

	
	}
}

void
binarySearch_device_enqueue_multiKeys_child(__global uint4 * outputArray,
             __global uint  * sortedArray, 
			 unsigned int subdivSize,
			 __global uint *globalLowerIndex,
             __global uint  *keys,
			 unsigned int nKeys,
			 __global uint * parentGlobalids,
			  unsigned int globalThreads
			 )			
{
    unsigned int tid = get_global_id(0);
	int keyCount =0 ;

	

	/**** Further Search happens only when at-least one key is found in previous search ****/
	int keysHitFlag = atomic_load_explicit(&keysHit,memory_order_seq_cst);

	if(keysHitFlag == 0)
		return;	
	
	

	for(keyCount=0;keyCount<nKeys;keyCount++)
	{
		 /* Then we find the elements  for this thread */
		int elementLower = sortedArray[globalLowerIndex[keyCount] + tid * subdivSize];
		int elementUpper = sortedArray[globalLowerIndex[keyCount] + (tid + 1) * subdivSize - 1];

		/* If the element to be found does not lie between them, then nothing left to do in this thread */
		if( (elementLower > keys[keyCount]) || (elementUpper < keys[keyCount])) 
		{
			continue;
		}
		else
		{
			/* However, if the element does lie between the lower and upper bounds of this thread's searchspace
			 * we need to narrow down the search further in this search space 
			 */
 
			unsigned int globalLowerTemp = tid * subdivSize;
						
			/*** Calculating the Lower bound of the Keys ****/	
		   	  outputArray[keyCount].x = parentGlobalids[keyCount] + globalLowerTemp;
			  parentGlobalids[keyCount]  += globalLowerTemp ;

			/****** Used for Error Checks *******/
			  outputArray[keyCount].w = 1;
			
			/****** Pass the new lower-index for each key for next kernel launch *******/
			  globalLowerIndex[keyCount]  = globalLowerTemp;

		    /******  Pass the final subdivSize to the Host ***/
			  outputArray[keyCount].y = subdivSize;

			 /*** Used atomics to ensure memory consistency ***/ 
			 atomic_store_explicit(&keysHit,1,memory_order_seq_cst);
		
		}
	}

	/*** child kernel is launched for one time ***/
	if(tid == 0)
	{
			
		queue_t defQ = get_default_queue();
			  
		/*** If the Search space is too small return to the host since it is not suitable to use GPU ***/ 
		if(subdivSize < globalThreads)
		{
		return;
		}

		/**** Narrow-down the search space ******/
		subdivSize = subdivSize/globalThreads ; 

			
		ndrange_t ndrange1 = ndrange_1D(globalThreads);

		/**** Kernel Block *****/
		void (^binarySearch_device_enqueue_wrapper_blk)(void) = ^{binarySearch_device_enqueue_multiKeys_child(outputArray,
			sortedArray, 
			subdivSize,
			globalLowerIndex,
			keys
			,nKeys
			,parentGlobalids,globalThreads);};
					
		int err_ret = enqueue_kernel(defQ,CLK_ENQUEUE_FLAGS_WAIT_KERNEL,ndrange1,binarySearch_device_enqueue_wrapper_blk);

		if(err_ret != 0)
		{
			outputArray[keyCount].w = ENQUEUE_KERNEL_FAILURE;
			outputArray[keyCount].z = err_ret;
			return;
		}



		
	}
}

#endif
/** Implementation of the Binary Search algorithm Using OpenCL 1.2 Standard .
 */  

__kernel void
binarySearch(__global uint4 * outputArray,
             __global uint  * sortedArray, 
			 unsigned int subdivSize,
			 __global uint *globalLowerIndex,
             __global uint  *keys,
			 unsigned int nKeys
			 )			
{
    unsigned int tid = get_global_id(0);
	int keyCount =0 ;
   

	for(keyCount=0;keyCount<nKeys;keyCount++)
	{
		 /* Then we find the elements  for this thread */
		int elementLower = sortedArray[globalLowerIndex[keyCount] + tid * subdivSize];
		int elementUpper = sortedArray[globalLowerIndex[keyCount] + (tid + 1) * subdivSize - 1];

		/* If the element to be found does not lie between them, then nothing left to do in this thread */
		if( (elementLower > keys[keyCount]) || (elementUpper < keys[keyCount])) 
		{
			continue;
		}
		else
		{
			/* However, if the element does lie between the lower and upper bounds of this thread's searchspace
			 * we need to narrow down the search further in this search space 
			 */
 
			/* The search space for this thread is marked in the output as being the total search space for the next pass */
			outputArray[keyCount].x = tid;
			outputArray[keyCount].w = 1;
		
		}
	}
}