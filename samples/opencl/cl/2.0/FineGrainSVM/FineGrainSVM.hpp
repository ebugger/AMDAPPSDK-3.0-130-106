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


#ifndef _FINEGRAIN_SVM_H_
#define _FINEGRAIN_SVM_H_


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <atomic>
#include "CLUtil.hpp"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"
#define OCL_COMPILER_FLAGS  "FineGrainSVM_OclFlags.txt"

/**
 * FineGrainSVM
 * Class implements OpenCL Prefix Sum sample
 */

class FineGrainSVM
{
        cl_uint
        seed;      /**< Seed value for random number generation */
        cl_double           setupTime;      /**< Time for setting up OpenCL */
        cl_double          kernelTime;      /**< Time for kernel execution */
        cl_uint                 length;     /**< length of the input array */
        cl_int               *buffer;      /**< Input array */
        cl_int               *atomicBuffer;      /**< Input array */
        cl_context            context;      /**< CL context */
        cl_device_id         *devices;      /**< CL device list */
        cl_command_queue commandQueue;      /**< CL command queue */
        cl_program            program;      /**< CL program  */
        cl_kernel        fine_grain_ldstore;      /**< CL kernel */
        int
        iterations;      /**< Number of iterations for kernel execution */
        SDKDeviceInfo deviceInfo;/**< Structure to store device information*/
        KernelWorkGroupInfo kernelInfo;/**< Structure to store kernel related info */

        SDKTimer *sampleTimer;      /**< SDKTimer object */

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
        FineGrainSVM()
            : seed(123),
              setupTime(0),
              kernelTime(0),
              length(1024),
              devices(NULL),
              iterations(1)
        {
            sampleArgs =  new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            sampleArgs->flags        = OCL_COMPILER_FLAGS;
        }

        /**
        *******************************************************************************
        * @fn Destructor
        * @brief Cleanup the member objects.
        *******************************************************************************
        */
        ~FineGrainSVM()
        {
            // release program resources
            FREE(devices);
        }

        /**
        *******************************************************************************
        * @fn setupFineGrainSVM
        * @brief Allocate and initialize host memory array with random values
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int setupFineGrainSVM();

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
        * @fn fineGrainSVMCPUReference
        * @brief Reference CPU implementation of Prefix Sum.
        *
        * @param output the array that stores the prefix sum
        * @param input the input array
        * @param length length of the input array
        *******************************************************************************
        */
        void fineGrainSVMCPUReference(cl_float * output);

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
        * @fn runGroupKernel
        * @brief Run group prefixsum CL kernel. The kernel make prefix sum on individual work groups.
        *
        * @param[in] offset : Distance between two consecutive index.
        *
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        *******************************************************************************
        */
        int runFineGrainSVMKernel();

};
#endif
