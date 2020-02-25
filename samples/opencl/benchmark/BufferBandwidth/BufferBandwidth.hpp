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

#ifndef BUFFER_BANDWIDTH_H_
#define BUFFER_BANDWIDTH_H_

#undef   USE_CL_PROFILER
#define  MAX_WAVEFRONT_SIZE 64               // work group size

#include "CLUtil.hpp"
#include "Host.h"
#include "Log.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

extern void assessHostMemPerf( void *, void *, size_t );


using namespace appsdk;

struct _flags { cl_mem_flags f;
                       const char  *s; };

struct _flags flags[] = {
             { CL_MEM_READ_ONLY,              "CL_MEM_READ_ONLY" },
             { CL_MEM_WRITE_ONLY,             "CL_MEM_WRITE_ONLY" },
             { CL_MEM_READ_WRITE,             "CL_MEM_READ_WRITE" },
             { CL_MEM_USE_HOST_PTR,           "CL_MEM_USE_HOST_PTR" },
             { CL_MEM_COPY_HOST_PTR,          "CL_MEM_COPY_HOST_PTR" },
             { CL_MEM_ALLOC_HOST_PTR,         "CL_MEM_ALLOC_HOST_PTR" },
             { CL_MEM_USE_PERSISTENT_MEM_AMD, "CL_MEM_USE_PERSISTENT_MEM_AMD"},
             { CL_MEM_HOST_WRITE_ONLY,        "CL_MEM_HOST_WRITE_ONLY"},
             { CL_MEM_HOST_READ_ONLY,         "CL_MEM_HOST_READ_ONLY"},
             { CL_MEM_HOST_NO_ACCESS,         "CL_MEM_HOST_NO_ACCESS"} };

/**
 * BufferBandwidth 
 * Class implements OpenCL Constant Buffer Bandwidth sample
 * Derived from SDKSample base class
 */

class BufferBandwidth 
{
    cl_command_queue queue;
	cl_context       context;
	cl_kernel        read_kernel;
	cl_kernel        write_kernel;
	cl_device_id *devices;          /**< CL device list */
	cl_program program;                 /**< CL program  */

	cl_mem_flags inFlags;
	cl_mem_flags outFlags;
	cl_mem_flags copyFlags;

	int              devnum;
	char             devname[256];

	#define  WS 128       // work group size

	int nLoops;           // overall number of timing loops
	int loop;             // number of read/write buffer loops
	int nRepeats;         // # of repeats for each transfer step
	int nSkip;            // to discount lazy allocation effects, etc.
	int nKLoops;          // repeat inside kernel to show peak mem B/W
	unsigned int nBytes;           // input and output buffer size
	int nThreads;         // number of GPU work items
	int nItems;           // number of 32-bit 4-vectors for GPU kernel
	int nAlign;           // safe bet for most PCs

	unsigned int nBytesResult;
	
	bool printLog;
	bool doHost;
	bool useSVM;

	int  whichTest;
	int inFlagNum;
	int outFlagNum;
	int copyFlagNum;

	bool mapRW;
	bool timings;
	int  nWF;
	double setupTime;

	TestLog *tlog;
	bool vFailure;
	bool signalA ;
	bool signalB ;
	void *memIn,
		 *memOut,
		 *memResult,
		 *memScratch;

	cl_mem inputBuffer,
		   outputBuffer,
		   resultBuffer,
		   resultBuffer2,
		   copyBuffer;

	//For SVM
	bool isOCL2_x;
	cl_int* inputSVMBuffer;
	cl_int* resultSVMBuffer;
	cl_int* outputSVMBuffer;
	cl_mem_flags inSVMFlags;
	cl_mem_flags outSVMFlags;

	int nFlags;

	SDKDeviceInfo deviceInfo;
	SDKTimer    *sampleTimer;      /**< SDKTimer object */

    public:

    CLCommandArgs *sampleArgs;

    /** 
     * Constructor 
     * Initialize member variables
     */
    BufferBandwidth()
    {
		sampleArgs = new CLCommandArgs() ;
        sampleTimer = new SDKTimer();
        sampleArgs->sampleVerStr = SAMPLE_VERSION;

		vFailure = false;
		signalA = false;
		signalB = false;
		nWorkers = 1;
		nLoops = 20;
		loop = 20;
		nRepeats = 1;
		nSkip = 2;
		nKLoops = 20;

		nBytes = 32*1024*1024;
		nAlign = 4096;

		printLog = false;
		doHost = true;
		whichTest = 0;
		mapRW = false;
		timings = false;
		nWF = 7;
		devnum = 0;

		inFlags = 0;
		outFlags = 0;
		copyFlags = 0;

		inSVMFlags = 0;
		outSVMFlags = 0;

		inFlagNum = -1;
		outFlagNum = -1;
		copyFlagNum = -1;

		nFlags = sizeof(flags) / sizeof(flags[0]);

		isOCL2_x = false;
		useSVM = false;
    }

   
    /**
     * OpenCL related initialisations. 
     * Set up Context, Device list, Command Queue, Memory buffers
     * Build CL kernel program executable
     * @return 1 on success and 0 on failure
     */
    int setupCL();

	/**
        * Set values for kernels' arguments, enqueue calls to the kernels
        * on to the command queue, wait till end of kernel execution.
        * Get kernel start and end time if timing is enabled
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
    int runCLKernels();

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
	
    void printStats();

	void runPCIeTestNoblock();
	void runPCIeTest();

    void runMapTest();
    void runReadWriteTest();
    void runCopyTest();
	void runMappedReadWriteTest();

	void timedBufMap(cl_command_queue queue, cl_mem buf, void **ptr, bool quiet );
	void timedBufUnmap( cl_command_queue queue, cl_mem buf, void **ptr, bool quiet );
    
	void timedBufMappedRead(cl_command_queue queue, cl_mem buf, unsigned char v, bool pcie);
    void timedBufMappedWrite(cl_command_queue queue, cl_mem buf, unsigned char v, bool pcie);
    
	void timedBufCLRead(cl_command_queue queue, cl_mem buf, void *ptr, unsigned char v, bool pcie);
    void timedBufCLWrite(cl_command_queue queue, cl_mem buf, void *ptr, unsigned char v, bool pcie);
    
	void timedBufCLCopy(cl_command_queue queue, cl_mem srcBuf, cl_mem dstBuf);
    void timedKernel(cl_command_queue queue, cl_kernel kernel, void * bufSrc, 
					void * bufDst, unsigned char v, bool quiet, bool svm = false);
    void timedReadKernelVerify(cl_command_queue queue, cl_kernel kernel, cl_mem bufSrc, cl_mem bufRes, unsigned char v, bool quiet);
    
	//svm functions
	int createSVMBuffers();
	void timedSVMMappedWrite(cl_command_queue queue, cl_int* svmptr, unsigned char v, bool pcie);
	void timedSVMMappedRead(cl_command_queue queue, cl_int* svmptr, unsigned char v, bool pcie);
	void timedSVMBufferKernelVerify( cl_command_queue queue, cl_kernel kernel, cl_int* bufSrc, cl_int* bufRes, unsigned char v, bool quiet);

	void computeGlobals();
	void printHeader();
	int createBuffers();
};


#endif
