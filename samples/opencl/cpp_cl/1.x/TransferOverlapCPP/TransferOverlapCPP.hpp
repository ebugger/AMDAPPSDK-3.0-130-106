/**********************************************************************
Copyright �2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

�   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
�   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef BUFFER_BANDWIDTH_H_
#define BUFFER_BANDWIDTH_H_

#define  MAX_WAVEFRONT_SIZE 64     // Work group size

#define CL_USE_DEPRECATED_OPENCL_1_1_APIS

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

#include "Log.h"
#include "Timer.h"
#include <CL/cl.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#if defined(__MINGW32__)
#undef max
#undef min
#endif
#include "CLUtil.hpp"

using namespace appsdk;

/**
 * TransferOverlap
 * Class implements OpenCL TransferOverlap benchmark sample
 */

class TransferOverlap
{
        bool correctness;     // Correctness status variable
        int nLoops;           // Overall number of timing loops
        int nSkip;            // To discount lazy allocation effects, etc.
        int nKLoops;          // Repeat inside kernel to show peak mem B/W,

        int nBytes;           // Input and output buffer size
        int nThreads;         // Number of GPU work items
        int nItems;           // Number of 32-bit 4-vectors for GPU kernel
        int nAlign;           // Safe bet for most PCs
        int nItemsPerThread;  // Number of 32-bit 4-vectors per GPU thread
        int nBytesResult;

        size_t globalWorkSize; // Global work items
        size_t localWorkSize;  // Local work items
        double testTime;         // Total time to complete

        bool printLog;       // Enable/Disable print log
        bool noOverlap;      // Disallow memset/kernel overlap
        int  numWavefronts;

        TestLog *timeLog;


        cl::CommandQueue commandQueue;

        cl::Context context;

        cl::Program program;

        cl::Kernel readKernel;

        cl::Kernel writeKernel;
        std::vector<cl::Platform> platforms;    /**< vector of platforms */

        std::vector<cl::Device> devices;        /**< vector of devices */
        std::vector<cl::Device> device;         /**< device to be used */

        std::vector< ::size_t> maxWorkItemSizes; /**<device info vector>*/
        std::string deviceVersion;
        std::string extensions;
        std::string openclCVersion;


        CPerfCounter t;


        cl::Buffer inputBuffer1;

        cl::Buffer inputBuffer2;

        cl::Buffer resultBuffer1;

        cl::Buffer resultBuffer2;

        cl_mem_flags inFlags;
        int inFlagsValue;
        SDKDeviceInfo deviceInfo;

        SDKTimer    *sampleTimer;      /**< SDKTimer object */

    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        /**
         * Constructor
         * Initialize member variables
         */
        TransferOverlap()
            :nLoops(50),
             nSkip(3),
             nKLoops(45),
             nBytes(16 * 1024 * 1024),
             nThreads(MAX_WAVEFRONT_SIZE),
             nItems(2),
             nAlign(4096),
             nBytesResult(32 * 1024 * 1024),
             printLog(false),
             numWavefronts(7),
             timeLog(NULL),
             inFlags(0),
             inFlagsValue(0),
             noOverlap(false),
             correctness(true)
        {
            sampleArgs = new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
        }

        /**
         * Allocate and initialize host memory array with random values
         * @return 1 on success and 0 on failure
         */
        int setupTransferOverlap();

        /**
         * OpenCL related initialisations.
         * Set up Context, Device list, Command Queue, Memory buffers
         * Build CL kernel program executable
         * @return 1 on success and 0 on failure
         */
        int setupCL();

        /**
         * Override from SDKSample. Initialize
         * command line parser, add custom options
         */
        int initialize();

        /**
         * Override from SDKSample, Generate binary image of given kernel
         * and exit application
         */
        int genBinaryImage();

        /**
         * Override from SDKSample, adjust width and height
         * of execution domain, perform all sample setup
         */
        int setup();

        /**
         * Override from SDKSample
         */
        int run();

        /**
         * Override from SDKSample
         * Cleanup memory allocations
         */
        int cleanup();

        /**
         * Override from SDKSample
         * Verify against reference implementation
         */
        int verifyResults()
        {
            return SDK_SUCCESS;
        };

        void printStats();

        /**
         * Parses Extra command line options and
         * calls SDKSample::parseCommandLine()
         */
        int parseExtraCommandLineOptions(int argc, char**argv);
        int verifyResultBuffer(cl::Buffer resultBuffer, bool firstLoop);
        int launchKernel(cl::Buffer inputBuffer, cl::Buffer resultBuffer,
                         unsigned char v);
        void* launchMapBuffer(cl::Buffer buffer, cl::Event *mapEvent);
        int fillBuffer(cl::Buffer buffer, cl::Event *mapEvent, void *ptr,
                       unsigned char v);
        int runOverlapTest();
        int setDeviceInfo(cl::Device device);
};


#endif
