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

#ifndef GLOBAL_MEMORY_BANDWIDTH_H_
#define GLOBAL_MEMORY_BANDWIDTH_H_


#define GROUP_SIZE 256
#define NUM_READS 32
#define NUM_KERNELS 6
#define OFFSET 16384
#define EXTRA_ELEMENTS (NUM_READS * OFFSET)
#define NUM_INPUT 1024 * 256
#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.3"

//Header Files
#include <CL/cl.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"

using namespace appsdk;

/**
 * GlobalMemoryBandwidth
 * Class implements OpenCL Global Buffer Bandwidth sample
 */

class GlobalMemoryBandwidth
{
        cl_double            setupTime;      /**< Time for setting up OpenCL */
        cl_uint                 length;      /**< Length of the input data */
        cl_uint                 readRange;
        cl_float                *input;       /**< Input array */
        cl_float     *outputReadSingle;       /**< Output Array for Single kernel*/
        cl_float     *outputReadLinear;       /**< Output Array for Linear kernel*/
        cl_float         *outputReadLU;       /**< Output Array for Linear kernel*/
        cl_float    *outputWriteLinear;       /**< Output Array for Linear kernel*/

        cl_float    *outputReadRandom;
        cl_float    *outputReadunCombine;
        cl_float    *inputExtra;

        cl_float   *verificationOutput;       /**< Output Array for Verification */
        cl_context             context;      /**< CL context */
        cl_device_id          *devices;      /**< CL device list */

        cl_mem             inputBuffer;      /**< input buffer */
        cl_mem  outputBufferReadSingle;      /**< CL memory buffer */
        cl_mem  outputBufferReadLinear;      /**< CL memory buffer */
        cl_mem      outputBufferReadLU;      /**< CL memory buffer */
        cl_mem outputBufferWriteLinear;

        cl_mem constValue;

        cl_mem outputBufferReadRandom;
        cl_mem outputBufferReadunCombine;
        cl_mem inputBufferExtra;

		/* SVM buffers */
		cl_float         *inputSVMBuffer;      
		cl_float	*inputSVMBufferExtra;		
		cl_float *outputSVMBufferWriteLinear;

        cl_command_queue  commandQueue;      /**< CL command queue */
        cl_program             program;      /**< CL program */
        cl_kernel  kernel[NUM_KERNELS];      /**< CL kernel */
        size_t           globalThreads;
        size_t            localThreads;


        size_t    kernelWorkGroupSize;      /**< Group Size returned by kernel */
        cl_ulong availableLocalMemory;
        cl_ulong    neededLocalMemory;
        int
        iterations;      /**< Number of iterations for kernel execution */
        int                vectorSize;      /**< float, float2, float4 */
        bool                writeFlag;
        bool                extraFlag;
        bool             uncachedRead;
        bool                  vec3;
		bool			   svmSupport;		/*Flag to indicate whether SVM support exists in the target platform*/

        double readLinearUncachedGbps;          /**< Record GBPS for every type of bandwidth test */
        double readLinearGbps;
        double writeLinearGbps;
        double readSingleGbps;
        double readRandomGbps;
        double readUncombineGbps;

        double readLinearUncachedTime;          /**< Record time for every type of bandwidth test */
        double readLinearTime;
        double writeLinearTime;
        double readSingleTime;
        double readRandomTime;
        double readUncombineTime;

        SDKDeviceInfo deviceInfo;  /**< Structure to store device information*/

        SDKTimer *sampleTimer;      /**< SDKTimer object */

    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        /**
         * Constructor
         * Initialize member variables
         */
        GlobalMemoryBandwidth()
        {
            sampleArgs = new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            input = NULL;
            outputReadSingle = NULL;
            outputReadLinear = NULL;
            outputWriteLinear = NULL;
            verificationOutput = NULL;

            outputReadRandom = NULL;
            outputReadunCombine = NULL;
            inputExtra = NULL;

            setupTime = 0;
            iterations = 100;
            length = NUM_INPUT;
            vectorSize =
                0;     // Query the device later and select the preferred vector-size
            globalThreads = length;
            localThreads = GROUP_SIZE;
            writeFlag = false;
            extraFlag = false;
            uncachedRead = false;
            vec3 = false;
			svmSupport = false;
        }

        /**
         * Allocate and initialize host memory array with random values
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int setupGlobalMemoryBandwidth();

        /**
         * Override from SDKSample, Generate binary image of given kernel
         * and exit application
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int genBinaryImage();

        /**
         * OpenCL related initialisations.
         * Set up Context, Device list, Command Queue, Memory buffers
         * Build CL kernel program executable
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int setupCL();

        /**
         * Set values for kernels' arguments, enqueue calls to the kernels
         * on to the command queue, wait till end of kernel execution.
         * Get kernel start and end time if timing is enabled
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int runCLKernels();

		int runCLKernels_SVM();

        int bandwidth(cl_kernel &kernel,
                      cl_mem outputBuffer,
					  cl_float *outputSVMBuffer,
                      double *timeTaken,
                      double *gbps,
					  bool useSVM);

        /**
        * Override from SDKSample. Print sample stats.
        */
        void printStats();

        /**
         * Override from SDKSample. Initialize
         * command line parser, add custom options
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int initialize();

        /**
         * Override from SDKSample, adjust width and height
         * of execution domain, perform all sample setup
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int setup();

        /**
         * Override from SDKSample
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int run();

        /**
         * Override from SDKSample
         * Cleanup memory allocations
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int cleanup();

        /**
         * Override from SDKSample
         * Verify against reference implementation
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int verifyResults(bool useSVM);

    private:

        /**
        * A common function to map cl_mem object to host
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        template<typename T>
        int mapBuffer(cl_mem deviceBuffer, T* &hostPointer, size_t sizeInBytes,
                      cl_map_flags flags=CL_MAP_READ);

        /**
        * A common function to unmap cl_mem object from host
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int unmapBuffer(cl_mem deviceBuffer, void* hostPointer);

		int createSVMBuffers(cl_uint bufferSize, cl_uint inputExtraBufferSize);

		bool is64Bit();
};
#endif
