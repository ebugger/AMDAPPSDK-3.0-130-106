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


#ifndef _EXTRACT_PRIMES_H_
#define _EXTRACT_PRIMES_H_


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"
#define OCL_COMPILER_FLAGS  "ExtractPrimes_OclFlags.txt"

#define ENQUEUE_KERNEL_FAILURE_LEVEL1 (-5)
#define ENQUEUE_KERNEL_FAILURE_LEVEL2 (-10)
#define ENQUEUE_KERNEL_FAILURE_LEVEL3 (-15)

/**
 * ExtractPrimes
 * Class implements OpenCL Prefix Sum sample
 */

class ExtractPrimes
{
        cl_uint
        seed;      /**< Seed value for random number generation */
        cl_double           setupTime;      /**< Time for setting up OpenCL */
        cl_double          kernelTime;      /**< Time for kernel execution */
        cl_uint                 length;     /**< length of the input array */
        cl_int               *input;      /**< Input array */
        cl_int               *outPrimes;      /**< Input array */
        cl_int               *primes;      /**< Input array */
        cl_int  *verificationOutput;      /**< Output array for reference implementation */
        cl_context            context;      /**< CL context */
        cl_device_id         *devices;      /**< CL device list */
        cl_mem            inputBuffer;      /**< CL memory buffer */
        cl_mem           primesBuffer;      /**< CL memory output Buffer */
        cl_mem           outputBuffer;      /**< CL memory output Buffer */
        cl_mem           outPrimesBuffer;   /**< CL memory outPrimes Buffer */
		cl_mem			 errorBuffer;       /** Used for error handling ***/
        cl_command_queue commandQueue;      /**< CL command queue */
		cl_command_queue devcommandQueue;      /**< CL command queue */
        cl_program            program;      /**< CL program  */
        cl_kernel        group_kernel;      /**< CL kernel */
        cl_kernel        set_primes_kernel;      /**< CL kernel */
        cl_kernel        get_primes_kernel;      /**< CL kernel */
		 cl_kernel		 extract_primes_kernel;  /**< CL kernel */
        cl_kernel       global_kernel;      /**< CL kernel */
		cl_uint         *errorBuffer_cpu;      /**Used for error handling */
        int
        iterations;      /**< Number of iterations for kernel execution */
        SDKDeviceInfo deviceInfo;/**< Structure to store device information*/
        KernelWorkGroupInfo kernelInfo;/**< Structure to store kernel related info */

        SDKTimer *sampleTimer;      /**< SDKTimer object */
		bool		devEnqueue;		/** Device-Side Enqueue flag **/

cl_uint         stages;
    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        /**
        *******************************************************************************
        * @fn Constructor
        * @brief Initialize member variables
        *
        *******************************************************************************
        */
        ExtractPrimes()
            : seed(123),
              setupTime(0),
              kernelTime(0),
              length(512),
              input(NULL),
              verificationOutput(NULL),
              devices(NULL),
              iterations(1)
        {
            sampleArgs =  new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            sampleArgs->flags        = OCL_COMPILER_FLAGS;
			devEnqueue				 = true ;
        }

        /**
        *******************************************************************************
        * @fn Destructor
        * @brief Cleanup the member objects.
        *******************************************************************************
        */
        ~ExtractPrimes()
        {
            // release program resources
            FREE(input);
            FREE(verificationOutput);
            FREE(devices);
        }

        /**
        *******************************************************************************
        * @fn setupExtractPrimes
        * @brief Allocate and initialize host memory array with random values
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int setupExtractPrimes();

        /**
        *******************************************************************************
        * @fn setupCL
        * @brief OpenCL related initialisations. Set up Context, Device list,
        *        Command Queue, Memory buffers Build CL kernel program executable.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int setupCL();

        /**
        *******************************************************************************
        * @fn runCLKernels
        * @brief Set values for kernels' arguments, enqueue calls to the kernels
        *        on to the command queue, wait till end of kernel execution.
        *        Get kernel start and end time if timing is enabled.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int runCLKernels();

        /**
        *******************************************************************************
        * @fn ExtractPrimesCPUReference
        * @brief Reference CPU implementation of Prefix Sum.
        *
        * @param output the array that stores the prefix sum
        * @param input the input array
        * @param length length of the input array
        *******************************************************************************
        */
        void extractPrimesCPUReference(cl_int * output,
				     cl_int * input,
				     const cl_uint length,
				     cl_int *outLength);

        /**
        *******************************************************************************
        * @fn printStats
        * @brief Override from SDKSample. Print sample stats.
        *******************************************************************************
        */
        void printStats();

        /**
        *******************************************************************************
        * @fn initialize
        * @brief Override from SDKSample. Initialize command line parser, add custom options.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int initialize();

        /**
        *******************************************************************************
        * @fn genBinaryImage
        * @brief Override from SDKSample, Generate binary image of given kernel
        *        and exit application.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int genBinaryImage();



        /**
        *******************************************************************************
        * @fn setup
        * @brief Override from SDKSample, adjust width and height
        *        of execution domain, perform all sample setup
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int setup();

        /**
        *******************************************************************************
        * @fn run
        * @brief Run OpenCL FastWalsh Transform. Override from SDKSample.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int run();

        /**
        *******************************************************************************
        * @fn cleanup
        * @brief Cleanup memory allocations. Override from SDKSample.
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int cleanup();

        /**
        *******************************************************************************
        * @fn verifyResults
        * @brief Verify against reference implementation
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int verifyResults();

        /**
        *******************************************************************************
        * @fn mapBuffer
        * @brief A common function to map cl_mem object to host
        *
        * @param[in] deviceBuffer : Device buffer
        * @param[out] hostPointer : Host pointer
        * @param[in] sizeInBytes : Number of bytes to map
        * @param[in] flags : map flags
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        template<typename T>
        int mapBuffer(cl_mem deviceBuffer, T* &hostPointer, size_t sizeInBytes,
                      cl_map_flags flags);

        /**
        *******************************************************************************
        * @fn unmapBuffer
        * @brief A common function to unmap cl_mem object from host
        *
        * @param[in] deviceBuffer : Device buffer
        * @param[in] hostPointer : Host pointer
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int unmapBuffer(cl_mem deviceBuffer, void* hostPointer);

        /**
        *******************************************************************************
        * @fn runGroupKernel
        * @brief Run group prefixsum CL kernel. The kernel make prefix sum on individual work groups.
        *
        * @param[in] offset : Distance between two consecutive index.
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int runGroupKernel();

        /**
        *******************************************************************************
        * @fn runGlobalKernel
        * @brief Run global prefixsum CL kernel. The kernel updates all elements.
        *
        * @param[in] offset : Distance between two consecutive index.
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int runGlobalKernel();
		
        /**
        *******************************************************************************
        * @fn runSetPrimesKernel
        * @brief Set the primes array for the index (in input array) which is prime.
        *
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */		
		int runSetPrimesKernel();
		
		/**
        *******************************************************************************
        * @fn runGetPrimesKernel
        * @brief This kernel gets the primes into the outPrimes array for those index which
		* have primes in input array
        *
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int runGetPrimesKernel();
		
		/**
        *******************************************************************************
        * @fn runGlobalKernel
        * @brief Utility function to check if the given number is prime.
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
    	int isPrime(int num);

        /***
	     *findStages
	     *finds number of stages required for global scan
	     ***/
        cl_uint findStages(cl_uint data_size, cl_uint wg_size);


		/**
        *******************************************************************************
        * @fn extractPrimesEnqueueKernel
        * @brief This kernel does the job of finding Primes by enqueueing the supported kernels
        * like set_primes_kernel,get_primes_kernel,group_scan_kernel & global_scan_kernel in the
        * device side
		*
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
		int extractPrimesEnqueueKernel();
};
#endif
