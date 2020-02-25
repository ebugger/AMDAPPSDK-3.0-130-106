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
#ifndef DEVICE_ENQUEUE_BFS_H
#define DEVICE_ENQUEUE_BFS_H

#define NUM_OF_NODES 1024
#define INIFINITY 100000000

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

//Header Files
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <list>
#include <string>
#include <sstream>
#include <vector>


#include "CLUtil.hpp"
#include <CL/cl.h>

typedef struct linearQueue
{
	int info;
	struct linearQueue *next;
} queue;

queue *front;				   /**< front node of the queue */	
queue *rear;				   /**< rear node of the queue */	
queue *newNode;

using namespace appsdk;

/**
 * DeviceEnqueueBFS
 * Class implements OpenCL Device-Enqueue BFS sample
 */

class DeviceEnqueueBFS
{
		double  setupTime;			   /**< Time for setting up OpenCL */
        double  seqTime;			   /**< Sequential kernel run time */
        double  conTime;		       /**< Concurrent kernel run time */

		cl_uint **adjMat;			   /**< Adjacency Matrix */
		cl_uint *colIndex;			   /**< Store column indexes of each non-zero element */
		cl_uint *rowPtr;			   /**< Store the location in colIndex array where each row starts */
		cl_uint *refDist;			   /**< Reference Array stores distance of each node from root node */

        cl_context context;            /**< CL context */
        cl_device_id *devices;         /**< CL device list */
		cl_command_queue commandQueue; /**< CL host command queue */
		cl_command_queue deviceCommandQueue; /**< CL device command queue */
        cl_program program;            /**< CL program  */
		cl_kernel writePipeKernel, deviceEnqueueKernel;     /**< CL kernel */
	
		size_t localWorkItems;		   /**< Work group size */
		size_t globalWorkItems;		   /**< Global threads  */

        cl_mem inputRowPtrBuffer;      /**< CL input RowPtr memory buffer */
		cl_mem inputColIndexBuffer;    /**< CL input ColIndex memory buffer */
		cl_uint *outputDistSVMBuffer;  /**< CL output Distance SVM buffer */
		cl_mem vertexQueueReadPipe, edgeQueueWritePipe;

		int numNodes;				   /**< Number of nodes in the given graph */	 
		int nZRCount;				   /**< Number of non-zero elements in given adjancey matrix */
		cl_uint rootNode;			   /**< Root node of the given graph */
		cl_uint offset;				   
		int  numComputeUnits;          /**< Number of compute units in the GPU */
		int  iterations;               /**< Number of iterations for kernel execution*/
		int  numWGsPerKernel;          /**< Number of WorkGroups per kernel run */
		cl_uint localSize;


        SDKDeviceInfo deviceInfo;      /**< Structure to store device information*/
        SDKTimer    *sampleTimer;      /**< SDKTimer object */
		int               Timer;	   /**< Timer */

    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        /**
         * Constructor
         * Initialize member variables
         */
        DeviceEnqueueBFS()
			:adjMat(NULL),
			 colIndex(NULL),
			 rowPtr(NULL),
			 refDist(NULL),
			 iterations(1),
			 inputRowPtrBuffer(NULL),
			 inputColIndexBuffer(NULL),
			 outputDistSVMBuffer(NULL),
			 vertexQueueReadPipe(NULL),
			 edgeQueueWritePipe(NULL),
             devices(NULL),
			 commandQueue(NULL),
			 deviceCommandQueue(NULL),
			 numNodes(NUM_OF_NODES),
			 nZRCount(0),
			 rootNode(0),
			 offset(0),
             numWGsPerKernel(10),
             localSize(64),
			 setupTime(0),
			 seqTime(0),
			 conTime(0)
        {
            sampleArgs = new CLCommandArgs() ;
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
        }

        /**
         * Allocate and initialize host memory array with random values
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int setupBFS();

        /**
         * OpenCL related initialisations.
         * Set up Context, Device list, Command Queue, Memory buffers
         * Build CL kernel program executable
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int setupCL();

        /**
         * Override from SDKSample. Initialize
         * command line parser, add custom options
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int initialize();

        /**
         * Override from SDKSample, Generate binary image of given kernel
         * and exit application
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int genBinaryImage();

        /**
         * Override from SDKSample, adjust width and height
         * of execution domain, perform all sample set-up
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
         * Clean-up memory allocations
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int cleanup();

        /**
         * Override from SDKSample
         * Verify against reference implementation
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int verifyResults();


        /**
         * Prints data and performance results
         */
        void printStats();

		/**
         * This function is indeed needed for more than iteration
		 * This function is used for clean-up the pipe memory objects 
		 * in-order to get correct value with-in pipes for next iteration
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
		int InitializePipe();

		/**
         * Execute Write-pipe kernel sequentially in a single queue
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
		int runWritePipeKernel();

        /**
         * Execute device-enqueue BFS kernels sequentially in a single queue
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int runDeviceEnqueueBFSKernels();

		/**
         * Reference implementation of BFS
        */
		void cpuRefImplementation();

		/**
		 * Converting adjacency matrix into CSR format
		*/
		int matToCSR();

		/**
		* insert val at the end of queue instance
		* @param val an integer value to be inserted at the end of the queue
		*/
		void enqueue(cl_uint val);

		/**
		* returns the front element of queue instance
		*/
		cl_uint dequeue();

		/**
		* Check whether queue is empty or not
		*/
		int underflow();
};
#endif // DEVICE_ENQUEUE_BFS_H
