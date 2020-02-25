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


#include "BufferBandwidth.hpp"


void BufferBandwidth::timedBufMappedRead( cl_command_queue queue,
                         cl_mem buf,
                         unsigned char v,
                         bool pcie )
{
    int t1 = sampleTimer->createTimer();
	int t2 = sampleTimer->createTimer();
	int t3 = sampleTimer->createTimer();

    cl_int ret;
    cl_event ev;
    void *ptr;
    cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    sampleTimer->resetTimer(t1);
    sampleTimer->resetTimer(t2);
    sampleTimer->resetTimer(t3);

	sampleTimer->startTimer(t1);

    if( !mapRW )
        mapFlag = CL_MAP_READ;

    ptr = (void * ) clEnqueueMapBuffer( queue,
                                        buf,
                                        CL_FALSE,
                                        mapFlag,
                                        0,
                                        nBytes,
                                        0, NULL,
                                        &ev,
                                        &ret );
    ASSERT_CL_RETURN( ret , "clEnqueueMapBuffer buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t1);
    
	sampleTimer->startTimer(t2);

    bool verify = readVerifyMemCPU_MT( ptr, v, nBytes );
	if(!verify)
		vFailure = true;

	sampleTimer->stopTimer(t2);

	sampleTimer->startTimer(t3);

    ret = clEnqueueUnmapMemObject( queue,
                                   buf,
                                   (void *) ptr,
                                   0, NULL, &ev );
    ASSERT_CL_RETURN( ret , "clEnqueueUnmapMemObject buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t3);

    if( pcie )
    {
        tlog->Timer( "PCIe B/W host->device (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
    else
    {
        const char *msg;

        if( mapRW )
            msg = "clEnqueueMapBuffer -- READ|WRITE (GBPS)";
        else
            msg = "clEnqueueMapBuffer -- READ (GBPS)";

        tlog->Timer( msg, sampleTimer->readTimer(t1), nBytes, 1 );
        tlog->Timer( "CPU read (GBPS)", sampleTimer->readTimer(t2), nBytes, 1 ); 
		tlog->Timer( "clEnqueueUnmapMemObject() (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );

	    if( verify )
            tlog->Msg( "\n Verification Passed!\n", "" );
        else
        {
            tlog->Error( "\n Verification Failed!\n", "" );
            vFailure = true;
        }
    }
}

void BufferBandwidth::timedSVMMappedRead( cl_command_queue queue,
                         cl_int* svmptr,
                         unsigned char v,
                         bool pcie )
{
    int t1 = sampleTimer->createTimer();
	int t2 = sampleTimer->createTimer();
	int t3 = sampleTimer->createTimer();

    cl_int ret;
    cl_event ev;
    
    cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    sampleTimer->resetTimer(t1);
    sampleTimer->resetTimer(t2);
    sampleTimer->resetTimer(t3);

	sampleTimer->startTimer(t1);

    if( !mapRW )
        mapFlag = CL_MAP_READ;

	ret = clEnqueueSVMMap(queue, CL_FALSE, mapFlag, svmptr, nBytes, 0, NULL, &ev);
	ASSERT_CL_RETURN( ret , "clEnqueueSVMMap svmptr failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t1);
    
	sampleTimer->startTimer(t2);

    bool verify = readVerifyMemCPU_MT( svmptr, v, nBytes );
	if(!verify)
		vFailure = true;

	sampleTimer->stopTimer(t2);

	sampleTimer->startTimer(t3);

	ret = clEnqueueSVMUnmap(queue, svmptr, 0, NULL, &ev);
    ASSERT_CL_RETURN( ret , "clEnqueueSVMUnmap svmptr failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t3);

    if( pcie )
    {
        tlog->Timer( "PCIe B/W host->device (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
    else
    {
        const char *msg;

        if( mapRW )
            msg = "clEnqueueSVMMap -- READ|WRITE (GBPS)";
        else
            msg = "clEnqueueSVMMap -- READ (GBPS)";

        tlog->Timer( msg, sampleTimer->readTimer(t1), nBytes, 1 );
        tlog->Timer( "CPU read (GBPS)", sampleTimer->readTimer(t2), nBytes, 1 ); 
		tlog->Timer( "clEnqueueSVMUnmap() (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );

	    if( verify )
            tlog->Msg( "\n Verification Passed!\n", "" );
        else
        {
            tlog->Error( "\n Verification Failed!\n", "" );
            vFailure = true;
        }
    }
}

void BufferBandwidth::timedSVMMappedWrite( cl_command_queue queue,
                          cl_int* svmptr,
                          unsigned char v,
                          bool pcie )
{
	int t1 = sampleTimer->createTimer();
	int t2 = sampleTimer->createTimer();
	int t3 = sampleTimer->createTimer();

    cl_int ret;
    cl_event ev;
    cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    sampleTimer->resetTimer(t1);
    sampleTimer->resetTimer(t2);
    sampleTimer->resetTimer(t3);
    
	sampleTimer->startTimer(t1);
    
    if( !mapRW )
        mapFlag = CL_MAP_WRITE_INVALIDATE_REGION;

    ret = clEnqueueSVMMap( queue, CL_FALSE, mapFlag, svmptr, nBytes, 0, NULL, &ev );
    ASSERT_CL_RETURN(ret, "clEnqueueSVMMap buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t1);
    
	sampleTimer->startTimer(t2);

    memset_MT( svmptr, v, nBytes );

	sampleTimer->stopTimer(t2);

	sampleTimer->startTimer(t3);

    ret = clEnqueueSVMUnmap( queue, svmptr, 0, NULL, &ev );
    ASSERT_CL_RETURN( ret , "clEnqueueSVMUnmap buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t3);

    if( pcie )
    {
		tlog->Timer( "PCIe B/W device->host (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );
    }
    else
    {
        const char *msg;

        if( mapRW )
            msg = "clEnqueueSVMMap -- READ|WRITE (GBPS)";
        else
            msg = "clEnqueueSVMMap -- WRITE (GBPS)";

        tlog->Timer( msg, sampleTimer->readTimer(t1), nBytes, 1 );

        tlog->Timer( "memset() (GBPS)", sampleTimer->readTimer(t2), nBytes, 1 );

        tlog->Timer( "clEnqueueSVMUnmap() (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );
    }
}

void BufferBandwidth::timedBufMappedWrite( cl_command_queue queue,
                          cl_mem buf,
                          unsigned char v,
                          bool pcie )
{
	int t1 = sampleTimer->createTimer();
	int t2 = sampleTimer->createTimer();
	int t3 = sampleTimer->createTimer();

    cl_int ret;
    cl_event ev;
    void *ptr;
    cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    sampleTimer->resetTimer(t1);
    
	sampleTimer->startTimer(t1);
    
    if( !mapRW )
        mapFlag = CL_MAP_WRITE_INVALIDATE_REGION;

    ptr = (void * ) clEnqueueMapBuffer( queue,
                                        buf,
                                        CL_FALSE,
                                        mapFlag,
                                        0,
                                        nBytes,
                                        0, NULL, 
                                        &ev,
                                        &ret );
    ASSERT_CL_RETURN(ret, "clEnqueueMapBuffer buf failed!");

    clFlush( queue );
	
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t1);

	sampleTimer->resetTimer(t2);
	sampleTimer->startTimer(t2);

    memset_MT( ptr, v, nBytes );

	sampleTimer->stopTimer(t2);

	sampleTimer->resetTimer(t3);
	sampleTimer->startTimer(t3);

    ret = clEnqueueUnmapMemObject( queue,
                                   buf,
                                   (void *) ptr,
                                   0, NULL, &ev );
    ASSERT_CL_RETURN( ret , "clEnqueueUnmapMemObject buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t3);

    if( pcie )
    {
		tlog->Timer( "PCIe B/W device->host (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );
    }
    else
    {
        const char *msg;

        if( mapRW )
            msg = "clEnqueueMapBuffer -- READ|WRITE (GBPS)";
        else
            msg = "clEnqueueMapBuffer -- WRITE (GBPS)";

        tlog->Timer( msg, sampleTimer->readTimer(t1), nBytes, 1 );

        tlog->Timer( "memset() (GBPS)", sampleTimer->readTimer(t2), nBytes, 1 );

        tlog->Timer( "clEnqueueUnmapMemObject() (GBPS)", sampleTimer->readTimer(t3), nBytes, 1 );
    }
}

void BufferBandwidth::timedBufMap( cl_command_queue queue,
                  cl_mem buf,
                  void **ptr,
                  bool quiet )
{
    int t1 = sampleTimer->createTimer();
    
    cl_int ret;
    cl_event ev;
    cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    sampleTimer->resetTimer(t1);
    sampleTimer->startTimer(t1);

    *ptr = (void * ) clEnqueueMapBuffer( queue,
                                         buf,
                                         CL_FALSE,
                                         mapFlag,
                                         0,
                                         nBytes,
                                         0, NULL,
                                         &ev,
                                         &ret );
    ASSERT_CL_RETURN( ret , "clEnqueueMapBuffer buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

    sampleTimer->stopTimer(t1);

    const char *msg;

    if( mapRW )
        msg = "clEnqueueMapBuffer -- READ|WRITE (GBPS)";
    else
        msg = "clEnqueueMapBuffer -- READ (GBPS)";

    if( !quiet )
        tlog->Timer( msg, sampleTimer->readTimer(t1), nBytes, 1 );
}

void BufferBandwidth::timedBufUnmap( cl_command_queue queue,
                    cl_mem buf,
                    void **ptr,
                    bool quiet )
{
    int t1 = sampleTimer->createTimer();
    cl_int ret;
    cl_event ev;

    sampleTimer->resetTimer(t1);
    sampleTimer->startTimer(t1);

    ret = clEnqueueUnmapMemObject( queue,
                                   buf,
                                   (void *) *ptr,
                                   0, NULL, &ev );
    ASSERT_CL_RETURN( ret , "clEnqueueUnmapMemObject buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

    sampleTimer->stopTimer(t1);

    if( !quiet )
        tlog->Timer( "clEnqueueUnmapMemObject() (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
}

void BufferBandwidth::timedBufCLRead( cl_command_queue queue,
                     cl_mem buf,
                     void *ptr,
                     unsigned char v,
                     bool pcie )
{
    int t1 = sampleTimer->createTimer();
    cl_int ret;
    cl_event ev;

    sampleTimer->resetTimer(t1);
    sampleTimer->startTimer(t1);

    ret = clEnqueueReadBuffer( queue,
                               buf,
                               CL_FALSE,
                               0,
                               nBytes,
                               ptr,
                               0, NULL,
                               &ev );
    ASSERT_CL_RETURN( ret , "clEnqueueReadBuffer buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

    sampleTimer->stopTimer(t1);

    bool verify = readVerifyMemCPU_MT( ptr, v, nBytes );
	if(!verify)
		vFailure = true;

    if( pcie )
    {
        tlog->Timer("PCIe B/W device->host (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
    else
    {
        tlog->Timer( "clEnqueueReadBuffer (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
}

void BufferBandwidth::timedBufCLWrite( cl_command_queue queue,
                      cl_mem buf,
                      void *ptr,
                      unsigned char v,
                      bool pcie )
{
    int t1 = sampleTimer->createTimer();
    cl_int ret;
    cl_event ev;

    memset( ptr, v, nBytes );

    sampleTimer->resetTimer(t1);
	sampleTimer->startTimer(t1);

    ret = clEnqueueWriteBuffer( queue,
                                buf,
                                CL_FALSE,
                                0,
                                nBytes,
                                ptr,
                                0, NULL,
                                &ev );
    ASSERT_CL_RETURN( ret ,"clEnqueueWriteBuffer buf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

    sampleTimer->stopTimer(t1);

    if( pcie )
    {
        tlog->Timer( "PCIe B/W host->device (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
    else
    {
        tlog->Timer( "clEnqueueWriteBuffer (GBPS)", sampleTimer->readTimer(t1), nBytes, 1 );
    }
}

void BufferBandwidth::timedBufCLCopy( cl_command_queue queue,
									 cl_mem srcBuf,
									 cl_mem dstBuf )
{
	int t1 = sampleTimer->createTimer();
    cl_int ret;
    cl_event ev;

	sampleTimer->resetTimer(t1);
	sampleTimer->startTimer(t1);

    ret = clEnqueueCopyBuffer( queue,
                               srcBuf,
                               dstBuf,
                               0, 0,
                               nBytes,
                               0, NULL,
                               &ev );
    ASSERT_CL_RETURN( ret ,"clEnqueueCopyBuffer srcBuf failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	sampleTimer->stopTimer(t1);

    tlog->Timer( "clEnqueueCopyBuffer (GBPS)", (sampleTimer->readTimer(t1)), nBytes, 1 );
}

void BufferBandwidth::timedKernel( cl_command_queue queue,
                  cl_kernel        kernel,
                  void *           bufSrc,
                  void *           bufDst,
                  unsigned char    v,
                  bool             quiet, 
				  bool			   svm)
{
     cl_int ret;
     cl_event ev=0;
     int t = sampleTimer->createTimer();
	 
     cl_uint nItemsPerThread = nItems / nThreads;

     size_t global_work_size[2] = { nThreads, 0 };
     size_t local_work_size[2] = { WS, 0 };

     cl_uint val=0;

     for(int i = 0; i < sizeof( cl_uint ); i++)
        val |= v << (i * 8);

	 if (svm)
	 {
		 clSetKernelArgSVMPointer(kernel, 0, (cl_int *)bufSrc);
		 clSetKernelArgSVMPointer(kernel, 1, (cl_int *)bufDst);
	 }
	 else
	 {
		clSetKernelArg( kernel, 0, sizeof(void *),  (void *) &bufSrc );
		clSetKernelArg( kernel, 1, sizeof(void *),  (void *) &bufDst );
	 }
     clSetKernelArg( kernel, 2, sizeof(cl_uint), (void *) &nItemsPerThread);
     clSetKernelArg( kernel, 3, sizeof(cl_uint), (void *) &val);
     clSetKernelArg( kernel, 4, sizeof(cl_uint), (void *) &nKLoops);

     sampleTimer->resetTimer(t);
     sampleTimer->startTimer(t);

     ret = clEnqueueNDRangeKernel( queue,
                                   kernel,
                                   1,
                                   NULL,
                                   global_work_size,
                                   local_work_size,
                                   0, NULL, &ev );
     ASSERT_CL_RETURN( ret , "clEnqueueNDRangeKernel failed!");

     clFlush( queue );
	 ret = spinForEventsComplete(1, &ev);
	 ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

     sampleTimer->stopTimer(t);

     if( !quiet )
         tlog->Timer( "clEnqueueNDRangeKernel() (secs)", 
                      sampleTimer->readTimer(t), nBytes, nKLoops,true);
}

void BufferBandwidth::timedSVMBufferKernelVerify( cl_command_queue queue, 
												 cl_kernel kernel, 
												 cl_int* bufSrc, 
												 cl_int* bufRes, 
												 unsigned char v, 
												 bool quiet)
{
	cl_int ret;
    cl_event ev;

    timedKernel( queue, kernel, bufSrc, bufRes, v, quiet, true );

	cl_map_flags mapFlag = CL_MAP_READ | CL_MAP_WRITE;

    if( !mapRW )
        mapFlag = CL_MAP_READ;

	ret = clEnqueueSVMMap(queue, CL_FALSE, mapFlag, bufRes, nBytesResult, 0, NULL, &ev);
	ASSERT_CL_RETURN( ret , "clEnqueueSVMMap bufRes failed!");
	
	clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

	cl_uint sum = 0;

     for(int i = 0; i < nThreads / WS; i++)
         sum += ((cl_uint *) bufRes)[i];

	 
	ret = clEnqueueSVMUnmap(queue, bufRes, 0, NULL, NULL);
	ASSERT_CL_RETURN( ret , "clEnqueueSVMUnmap bufRes failed!");

     bool verify;

     if( sum == nBytes / sizeof(cl_uint) )
         verify = true;
     else
     {
         verify = false;
         vFailure = true;
     }

	 if( !quiet )
     {
        if( verify )
            tlog->Msg( "\n Verification Passed!\n", "" );
        else
            tlog->Error( "\n Verification Failed!\n", "" );
     }
}

void BufferBandwidth::timedReadKernelVerify( cl_command_queue queue,
                            cl_kernel        kernel,
                            cl_mem           bufSrc,
                            cl_mem           bufRes,
                            unsigned char    v,
                            bool             quiet )
{
    cl_int ret;
    cl_event ev;

    timedKernel( queue, kernel, bufSrc, bufRes, v, quiet );

    ret = clEnqueueReadBuffer( queue,
                               bufRes,
                               CL_FALSE,
                               0,
                               nBytesResult,
                               memResult,
                               0, NULL,
                               &ev );

    ASSERT_CL_RETURN( ret , "clEnqueueReadBuffer bufRes failed!");

    clFlush( queue );
	ret = spinForEventsComplete(1, &ev);
	ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

     cl_uint sum = 0;

     for(int i = 0; i < nThreads / WS; i++)
         sum += ((cl_uint *) memResult)[i];

     bool verify;

     if( sum == nBytes / sizeof(cl_uint) )
         verify = true;
     else
     {
         verify = false;
         vFailure = true;
     }

	 if( !quiet )
     {
        if( verify )
            tlog->Msg( "\n Verification Passed!\n", "" );
        else
            tlog->Error( "\n Verification Failed!\n", "" );
     }
}

int BufferBandwidth::createSVMBuffers()
{
#ifdef CL_VERSION_2_0

	if (useSVM)
	{	
		// Create input buffer
		inputSVMBuffer = (cl_int*) clSVMAlloc(context,
											inSVMFlags,
											nBytes,
											0);
		if (inputSVMBuffer == NULL)
		{
			std::cout << "clSVMAlloc failed. (inputSVMBuffer)" << std::endl;
			return SDK_FAILURE; 
		}

		// Create resultSVMBuffer buffer
		resultSVMBuffer = (cl_int*) clSVMAlloc(context,
											CL_MEM_READ_WRITE,
											nBytesResult,
											0);
		if (resultSVMBuffer == NULL)
		{
			std::cout << "clSVMAlloc failed. (resultSVMBuffer)" << std::endl;
			return SDK_FAILURE; 
		}	

		// Create outputSVMBuffer buffer
		outputSVMBuffer = (cl_int*) clSVMAlloc(context,
											outSVMFlags,
											nBytes,
											0);
		if (outputSVMBuffer == NULL)
		{
			std::cout << "clSVMAlloc failed. (outputSVMBuffer)" << std::endl;
			return SDK_FAILURE; 
		}	    
	}

#endif // CL_VERSION_2_0
	return SDK_SUCCESS;
}

int BufferBandwidth::createBuffers()
{
   // host memory buffers

#ifdef _WIN32
   memIn =      (void *) _aligned_malloc( nBytes, nAlign );
   memOut =     (void *) _aligned_malloc( nBytes, nAlign );
   memResult =  (void *) _aligned_malloc( nBytesResult, nAlign );
   memScratch = (void *) _aligned_malloc( nBytes, nAlign );
#else
   memIn =      (void *) memalign( nAlign, nBytes );
   memOut =     (void *) memalign( nAlign, nBytes );
   memResult =  (void *) memalign( nAlign, nBytesResult );
   memScratch = (void *) memalign( nAlign, nBytes );
#endif

   if( memIn == NULL ||
       memOut == NULL ||
       memResult == NULL ||
       memScratch == NULL ) 
   {
       fprintf( stderr, "%s:%d: error: %s\n", \
                __FILE__, __LINE__, "could not allocate host buffers\n" );
       exit(SDK_FAILURE);
   }

   // CL buffers

   cl_int ret;
   void *hostPtr = NULL;

   if( inFlags & CL_MEM_USE_HOST_PTR ||
       inFlags & CL_MEM_COPY_HOST_PTR )
       hostPtr = memIn;

   inputBuffer = clCreateBuffer( context,
                                 inFlags,
                                 nBytes,
                                 hostPtr, &ret );

   CHECK_OPENCL_ERROR(ret, "clCreateBuffer failed. (inputBuffer)");

   hostPtr = NULL;

   if( outFlags & CL_MEM_USE_HOST_PTR ||
       outFlags & CL_MEM_COPY_HOST_PTR )
       hostPtr = memOut;

   outputBuffer = clCreateBuffer( context,
                                  outFlags,
                                  nBytes,
                                  hostPtr, &ret );

   CHECK_OPENCL_ERROR(ret, "clCreateBuffer failed. (outputBuffer)");

   hostPtr = NULL;

   if( copyFlags & CL_MEM_USE_HOST_PTR ||
       copyFlags & CL_MEM_COPY_HOST_PTR )
       hostPtr = memScratch;

   if( whichTest == 2 ||
       whichTest == 3 ||
	   signalA )
      copyBuffer = clCreateBuffer( context,
                                   copyFlags,
                                   nBytes,
                                   hostPtr, &ret );

   CHECK_OPENCL_ERROR(ret, "clCreateBuffer failed. (copyBuffer)");

   resultBuffer = clCreateBuffer( context,
                                  CL_MEM_READ_WRITE,
                                  nBytesResult,
                                  NULL, &ret );
   CHECK_OPENCL_ERROR(ret, "clCreateBuffer failed. (resultBuffer)");

   resultBuffer2 = clCreateBuffer( context,
                                  CL_MEM_READ_WRITE,
                                  nBytesResult,
                                  NULL, &ret );
   CHECK_OPENCL_ERROR(ret, "clCreateBuffer failed. (resultBuffer2)");

   if (useSVM)
   {
	   if (createSVMBuffers() != SDK_SUCCESS)
	   {
			return SDK_FAILURE;
	   }
   }

   return SDK_SUCCESS;
}

void BufferBandwidth::printHeader()
{
	//std::cout << "\nDevice " << std::setw(2) << devnum << "            " << devname << "\n";

#ifdef _WIN32
    std::cout << "Build:               _WINxx"; 
#ifdef _DEBUG
    std::cout << " DEBUG";
#else
    std::cout << " release";
#endif
    std::cout << "\n" ;
#else
#ifdef NDEBUG
    std::cout << "Build:               release\n";
#else
    std::cout << "Build:               DEBUG\n";
#endif
#endif

    std::cout << "GPU work items:      " << nThreads << std::endl <<
                 "Buffer size:         " << nBytes << std::endl <<
                 "CPU workers:         " << nWorkers << std::endl <<
                 "Timing loops:        " << nLoops << std::endl <<
                 "Repeats:             " << nRepeats << std::endl <<
                 "Kernel loops:        " << nKLoops << std::endl;

    std::cout << "inputBuffer:         ";

   for( int i = 0; i < nFlags; i++ )
      if( inFlags & flags[i].f )
          std::cout << flags[i].s << " ";

   std::cout << "\noutputBuffer:        ";

   for( int i = 0; i < nFlags; i++ )
      if( outFlags & flags[i].f )
         std::cout << flags[i].s << " ";

   if( whichTest == 2 ||
       whichTest == 3 ||
       signalA)
   {
       std::cout << "\ncopyBuffer:          " ;

      for( int i = 0; i < nFlags; i++ )
      if( copyFlags & flags[i].f )
          std::cout << flags[i].s << " ";
   }

   if (useSVM)
   {
	   std::cout << "\ninputSVMBuffer:      ";

	   for( int i = 0; i < nFlags; i++ )
		  if( inSVMFlags & flags[i].f )
			  std::cout << flags[i].s << " ";
   
	   std::cout << "\noutputSVMBuffer:     ";

	   for( int i = 0; i < nFlags; i++ )
		  if( outSVMFlags & flags[i].f )
			  std::cout << flags[i].s << " ";
   }

   std::cout << "\n\n";
}

void BufferBandwidth::printStats()
{
    if(timings)
	{
		std::cout << std::setw(21) << std::left << "Setup Time"
				<< setupTime << " secs" << std::endl;
	}
	
	if( printLog ) 
      tlog->printLog();

   tlog->printSummary( nSkip );

   std::cout << "\n" ;
   fflush(stdout);
}

void BufferBandwidth::runMapTest()
{
   int  nl = nLoops;

   while( nl-- )
   {
      tlog->loopMarker();

      tlog->Msg( "\n\n%s\n", "1. Host mapped write to inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufMappedWrite( queue, inputBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "2. GPU kernel read of inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "3. GPU kernel write to outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "4. Host mapped read of outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufMappedRead( queue, outputBuffer, nl & 0xff, false );

	  //If SVM supported
	  if (useSVM)
	  {
		tlog->Msg( "\n\n%s\n", "5. Host mapped write to inputSVMBuffer" );

		for(int i = 0; i < nRepeats; i++)
			timedSVMMappedWrite( queue, inputSVMBuffer, nl & 0xff, false );

		tlog->Msg( "\n\n%s\n", "6. GPU kernel execution using inputSVMBuffer" );

        for(int i = 0; i < nRepeats; i++)
			timedSVMBufferKernelVerify( queue, read_kernel, inputSVMBuffer, resultSVMBuffer, nl & 0xff, false );

		tlog->Msg( "\n\n%s\n", "7. GPU kernel write to outputSVMBuffer" );

		for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultSVMBuffer, outputSVMBuffer, nl & 0xff, false , true);

		tlog->Msg( "\n\n%s\n", "8. Host mapped read of outputSVMBuffer" );

		for(int i = 0; i < nRepeats; i++)
			timedSVMMappedRead( queue, outputSVMBuffer, nl & 0xff, false );
	  }

	  clFinish(queue);
      tlog->Msg( "%s\n", "" );
   }
}

void BufferBandwidth::runPCIeTest()
{
   int nl = nLoops;

   void *mappedPtr;
   while( nl-- )
   {
      tlog->loopMarker();

      tlog->Msg( "%s\n", "" );

      timedBufMap( queue, copyBuffer, &mappedPtr, true );
	  
      for(int i = 0; i < nRepeats; i++)
          timedBufCLWrite( queue, inputBuffer, mappedPtr, nl & 0xff, true );
	  
      for(int i = 0; i < nRepeats; i++)
          timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, true );

      for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, true );
	 
      for(int i = 0; i < nRepeats; i++)
          timedBufCLRead( queue, outputBuffer, mappedPtr, nl & 0xff, true );

      timedBufUnmap( queue, copyBuffer, &mappedPtr, true );

      tlog->Msg( "%s\n", "" );
   }
}

void BufferBandwidth::runPCIeTestNoblock()
{
   	int nl = nLoops;
    void *mappedPtr;

	while( nl-- )
	{
		tlog->loopMarker();

		tlog->Msg( "%s\n", "" );

		timedBufMap( queue, copyBuffer, &mappedPtr, true );
		
		int t1 = sampleTimer->createTimer();
		sampleTimer->resetTimer(t1);

		cl_int ret;
		cl_event ev;
		bool flag=true;
		memset( mappedPtr, nl & 0xff, nBytes );

		sampleTimer->resetTimer(t1);
		sampleTimer->startTimer(t1);
		
		for(int i = 0; i < loop; i++)
		{
			ret = clEnqueueWriteBuffer( queue,
										inputBuffer,
										CL_FALSE,
										0,
										nBytes,
										mappedPtr,
										0, NULL,
										&ev );
			ASSERT_CL_RETURN( ret , "clEnqueueWriteBuffer inputBuffer failed!");
		}
		clFlush( queue );
		ret = spinForEventsComplete(1, &ev);
		ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

		sampleTimer->stopTimer(t1);
		double avg=(sampleTimer->readTimer(t1))/loop;

		if( flag )
		{
			tlog->Timer( "PCIe B/W host->device (GBPS)", avg, nBytes, 1 );
		}
		else
		{
			tlog->Timer( "clEnqueueWriteBuffer (GBPS)", avg, nBytes, 1 );
		}
		timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, true );

		timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, true );
		
		sampleTimer->resetTimer(t1);
		sampleTimer->startTimer(t1);

		for(int i = 0; i < loop; i++)
		{
			ret = clEnqueueReadBuffer( queue,
								   outputBuffer,
								   CL_FALSE,
								   0,
								   nBytes,
								   mappedPtr,
								   0, NULL,
								   &ev );
			ASSERT_CL_RETURN( ret , "clEnqueueReadBuffer outputBuffer failed!");
		}
		clFlush( queue );
		ret = spinForEventsComplete(1, &ev);
		ASSERT_CL_RETURN(ret, "spinForEventsComplete failed!");

		sampleTimer->stopTimer(t1);
		avg=(sampleTimer->readTimer(t1))/loop;

		bool verify = readVerifyMemCPU_MT( mappedPtr, nl & 0xff, nBytes );

		if(flag)
		{
			tlog->Timer( "PCIe B/W device->host (GBPS)", avg, nBytes, 1 );
		}
		else
		{
			tlog->Timer( "clEnqueueReadBuffer (GBPS)", avg, nBytes, 1 );
		}
		timedBufUnmap( queue, copyBuffer, &mappedPtr, true );

		if(!flag)
		{
			if(!verify)
			{
				vFailure = true;
			}
		}
		tlog->Msg( "%s\n", "" );
	}
}

void BufferBandwidth::runReadWriteTest()
{
   
   int nl = nLoops;

   while( nl-- )
   {
      tlog->loopMarker();

      tlog->Msg( "\n\n%s\n", "1. Host CL write to inputBuffer" );
	 
      for(int i = 0; i < nRepeats; i++)
          timedBufCLWrite( queue, inputBuffer, memScratch, nl & 0xff, false );
	  
      tlog->Msg( "\n\n%s\n", "2. GPU kernel read of inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "3. GPU kernel write to outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "4. Host CL read of outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufCLRead( queue, outputBuffer, memScratch, nl & 0xff, false );
	 
      tlog->Msg( "%s\n", "" );
   }
}

void BufferBandwidth::runMappedReadWriteTest()
{
   int nl = nLoops;

   void *mappedPtr;

   while( nl-- )
   {
      tlog->loopMarker();

      tlog->Msg( "\n\n%s\n", "1. Mapping copyBuffer as mappedPtr" );

      timedBufMap( queue, copyBuffer, &mappedPtr, false );

      tlog->Msg( "\n\n%s\n", "2. Host CL write from mappedPtr to inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufCLWrite( queue, inputBuffer, mappedPtr, nl & 0xff, false );
	  
      tlog->Msg( "\n\n%s\n", "3. GPU kernel read of inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "4. GPU kernel write to outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "5. Host CL read of outputBuffer to mappedPtr" );
	  
	  
      for(int i = 0; i < nRepeats; i++)
          timedBufCLRead( queue, outputBuffer, mappedPtr, nl & 0xff, false );
	  
      tlog->Msg( "\n\n%s\n", "6. Unmapping copyBuffer" );

      timedBufUnmap( queue, copyBuffer, &mappedPtr, false );

      tlog->Msg( "%s\n", "" );
   }
}

void BufferBandwidth::runCopyTest()
{
   int nl = nLoops;

   while( nl-- )
   {
      tlog->loopMarker();

      tlog->Msg( "\n\n%s\n", "1. Host mapped write to copyBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufMappedWrite( queue, copyBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "2. CL copy of copyBuffer to inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufCLCopy( queue, copyBuffer, inputBuffer );

      tlog->Msg( "\n\n%s\n", "3. GPU kernel read of inputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedReadKernelVerify( queue, read_kernel, inputBuffer, resultBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "4. GPU kernel write to outputBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedKernel( queue, write_kernel, resultBuffer, outputBuffer, nl & 0xff, false );

      tlog->Msg( "\n\n%s\n", "5. CL copy of outputBuffer to copyBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufCLCopy( queue, outputBuffer, copyBuffer );

      tlog->Msg( "\n\n%s\n", "6. Host mapped read of copyBuffer" );

      for(int i = 0; i < nRepeats; i++)
          timedBufMappedRead( queue, copyBuffer, nl & 0xff, false );

      tlog->Msg( "%s\n", "" );
   }
}

void BufferBandwidth::computeGlobals()
{
	if( whichTest > 3 || whichTest < 0 )
	{
		std::cout << "Testtype index should be between 0 and 3!" << std::endl;
		exit(SDK_EXPECTED_FAILURE);
	}

	devnum = sampleArgs->deviceId;

    if( nWorkers > MAXWORKERS ) nWorkers = MAXWORKERS;
    if( nWorkers <= 0 ) nWorkers = 1;

	if( inFlagNum >=0 && inFlagNum < nFlags )
                 inFlags |= flags[ inFlagNum ].f;
	if( outFlagNum >=0 && outFlagNum < nFlags )
                outFlags |= flags[ outFlagNum ].f;
	if( copyFlagNum >=0 && copyFlagNum < nFlags )
                copyFlags |= flags[ copyFlagNum ].f;

    cl_mem_flags f = CL_MEM_READ_ONLY |
                     CL_MEM_WRITE_ONLY |
                     CL_MEM_READ_WRITE;

    if( (inFlags & f) == 0 )
             inFlags |= CL_MEM_READ_ONLY;

    if( (outFlags & f) == 0 )
             outFlags |= CL_MEM_WRITE_ONLY;

    f |= CL_MEM_USE_HOST_PTR |
         CL_MEM_COPY_HOST_PTR |
         CL_MEM_ALLOC_HOST_PTR |
         CL_MEM_USE_PERSISTENT_MEM_AMD;

    if( (copyFlags & f) == 0 )
             copyFlags = CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE;

    f = CL_MEM_READ_ONLY |
        CL_MEM_WRITE_ONLY |
        CL_MEM_READ_WRITE;

    if( (copyFlags & f) == 0 )
             copyFlags |= CL_MEM_READ_WRITE;

	//SVM
	f = CL_MEM_READ_ONLY | CL_MEM_WRITE_ONLY | CL_MEM_READ_WRITE;

	inSVMFlags = f & inFlags;
	outSVMFlags = f & outFlags;

    nSkip = nLoops > nSkip ? nSkip : 0;

    if( signalA )
    {
        inFlags = CL_MEM_READ_ONLY;
        outFlags = CL_MEM_WRITE_ONLY;
        copyFlags = CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE;
        nKLoops = 1;
        doHost = false;
    }

    // educated guess of optimal work size
    unsigned int minBytes = WS * sizeof( cl_uint ) * 4 * nWorkers;

    nBytes = ( nBytes / minBytes ) * minBytes;
    nBytes = nBytes < minBytes ? minBytes : nBytes;
    nItems = nBytes / ( 4 * sizeof(cl_uint) );
    
	if (nBytes > deviceInfo.maxMemAllocSize)
	{
		std::cout << "Specified buffer size exceeds the maximum device limit " << deviceInfo.maxMemAllocSize << "! Exiting" << std::endl;
		exit(SDK_EXPECTED_FAILURE);
	}

    int maxThreads = nBytes / ( 4 * sizeof( cl_uint ) );

    nThreads = deviceInfo.maxComputeUnits * nWF * WS;
 
    if( nThreads > maxThreads )
        nThreads = maxThreads;
    else
    {
        while( nItems % nThreads != 0 )
            nThreads += WS;
    }

    nBytesResult = ( nThreads / WS ) * sizeof(cl_uint);

	tlog = new TestLog( nLoops * nRepeats * 50 );
}

int BufferBandwidth::setupCL(void)
{
    cl_int status = 0;
    cl_device_type dType;

    if(sampleArgs->deviceType.compare("cpu") == 0)
    {
        dType = CL_DEVICE_TYPE_CPU;
    }
    else //deviceType = "gpu"
    {
        dType = CL_DEVICE_TYPE_GPU;
        if(sampleArgs->isThereGPU() == false)
        {
            std::cout << "GPU not found. Falling back to CPU device" << std::endl;
            dType = CL_DEVICE_TYPE_CPU;
        }
    }

    // Get platform
    cl_platform_id platform = NULL;
    int retValue = getPlatform(platform, sampleArgs->platformId,
                               sampleArgs->isPlatformEnabled());
    CHECK_ERROR(retValue, SDK_SUCCESS, "getPlatform() failed");

    // Display available devices.
    retValue = displayDevices(platform, dType);
    CHECK_ERROR(retValue, SDK_SUCCESS, "displayDevices() failed");

    // If we could find our platform, use it. Otherwise use just available platform.
    cl_context_properties cps[3] =
    {
        CL_CONTEXT_PLATFORM,
        (cl_context_properties)platform,
        0
    };

    context = clCreateContextFromType(
                  cps,
                  dType,
                  NULL,
                  NULL,
                  &status);
    CHECK_OPENCL_ERROR(status, "clCreateContextFromType failed.");

    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");

    //Set device info of given cl_device_id
    status = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "SDKDeviceInfo::setDeviceInfo() failed");
	char buildOption[100] = "";

	isOCL2_x = deviceInfo.checkOpenCL2_XCompatibility();

	if (!isOCL2_x)
		useSVM = false;
	else
		if (deviceInfo.detectSVM())
		{
			useSVM = true;
			strcat(buildOption, "-cl-std=CL2.0 ");
		}

#ifdef CL_VERSION_2_0
    // Create command queue
    cl_queue_properties prop[] = {0};
    queue = clCreateCommandQueueWithProperties(context,
                                        devices[sampleArgs->deviceId],
                                        prop,
                                        &status);
    CHECK_OPENCL_ERROR(status, "clCreateCommandQueueWithProperties failed.");

#else
	
	 queue = clCreateCommandQueue( context,
                                 devices[devnum],
                                 0,
                                 NULL );
      CHECK_OPENCL_ERROR(status, "clCreateCommandQueue failed.");
#endif

    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("BufferBandwidth_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
    buildData.flagsStr = std::string(buildOption);

    if(sampleArgs->isLoadBinaryEnabled())
    {
        buildData.binaryName = std::string(sampleArgs->loadBinary.c_str());
    }

    if(sampleArgs->isComplierFlagsSpecified())
    {
        buildData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

    retValue = buildOpenCLProgram(program, context, buildData);
    CHECK_ERROR(retValue, SDK_SUCCESS, "buildOpenCLProgram() failed");

    // get a kernel object handle for a kernel with the given name
    read_kernel = clCreateKernel(program, "read_kernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::read_kernel failed.");

    write_kernel = clCreateKernel(program, "write_kernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::write_kernel failed.");
	
    return SDK_SUCCESS;
}

int BufferBandwidth::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer)");

    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBuffer)");

	status = clReleaseMemObject(resultBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(resultBuffer)");

	status = clReleaseMemObject(resultBuffer2);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(resultBuffer2)");

	if( whichTest == 2 ||
       whichTest == 3 ||
	   signalA )
	{
		status = clReleaseMemObject(copyBuffer);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(copyBuffer)");
	}

#ifdef CL_VERSION_2_0

	if (useSVM)
	{
		clSVMFree(context, inputSVMBuffer);
		clSVMFree(context, resultSVMBuffer);
		clSVMFree(context, outputSVMBuffer);
	}

#endif

    status = clReleaseKernel(read_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(read_kernel)");
	
	status = clReleaseKernel(write_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(write_kernel)");

	clReleaseDevice(devices[sampleArgs->deviceId]);
	CHECK_OPENCL_ERROR(status, "clReleaseDevice failed.(devices[sampleArgs->deviceId])");

    status = clReleaseCommandQueue(queue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(queue)");

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");
	
    return SDK_SUCCESS;
}

int BufferBandwidth::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("BufferBandwidth_Kernels.cl");
    binaryData.flagsStr = std::string("");
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    return status;
}

int BufferBandwidth::setup()
{	
	int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    if (setupCL() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

	computeGlobals();

	//create buffers
	if (createBuffers() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	printHeader();

	return SDK_SUCCESS;
}

int BufferBandwidth::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize())
    {
        return SDK_FAILURE;
    }

#ifdef _WIN32
    Option* nWorkersOption = new Option;
    CHECK_ALLOCATION(nWorkersOption, "Memory allocation error.\n");

    nWorkersOption->_sVersion = "nwk";
    nWorkersOption->_lVersion = "numcpuwrk";
    nWorkersOption->_description = "Number of CPU workers (max: 32 (Linux: 1))";
    nWorkersOption->_type = CA_ARG_INT;
    nWorkersOption->_value = &nWorkers;

    sampleArgs->AddOption(nWorkersOption);

    if( nWorkers <= 0)
    {
        std::cout << "\nError: Number of CPU workers should be greater than 0!";
        exit(SDK_EXPECTED_FAILURE);
    }
    delete nWorkersOption;
#endif

	 Option* nLoopsOption = new Option;
    CHECK_ALLOCATION(nLoopsOption, "Memory allocation error.\n");
    nLoopsOption->_sVersion = "nl";
    nLoopsOption->_lVersion = "numLoops";
    nLoopsOption->_description = "Number of iterations to repeat overall bandwidth measurement";
    nLoopsOption->_type = CA_ARG_INT;
	nLoopsOption->_usage = "<n>";
    nLoopsOption->_value = &nLoops;
    sampleArgs->AddOption(nLoopsOption);
	delete nLoopsOption;

	Option* loopOption = new Option;
    CHECK_ALLOCATION(loopOption, "Memory allocation error.\n");
    loopOption->_sVersion = "lp";
    loopOption->_lVersion = "numIterBfr";
    loopOption->_description = "When -noblock is active, set the number of iterations to run read/write buffer calls";
    loopOption->_type = CA_ARG_INT;
    loopOption->_value = &loop;
	loopOption->_usage = "<n>";
    sampleArgs->AddOption(loopOption);
	delete loopOption;

	Option* nBytesOption = new Option;
    CHECK_ALLOCATION(nBytesOption, "Memory allocation error.\n");
    nBytesOption->_sVersion = "nb";
    nBytesOption->_lVersion = "numBytes";
    nBytesOption->_description = "Buffer size in bytes (min: 2048*CPU Workers)";
	nBytesOption->_type = CA_ARG_LONG;
    nBytesOption->_value = &nBytes;
	nBytesOption->_usage = "<n>";
    sampleArgs->AddOption(nBytesOption);
	delete nBytesOption;

	Option* nRepeatsOption = new Option;
    CHECK_ALLOCATION(nRepeatsOption, "Memory allocation error.\n");
    nRepeatsOption->_sVersion = "nr";
    nRepeatsOption->_lVersion = "numRepeats";
    nRepeatsOption->_description = "Repeat each timing <n> times (can't be 0)";
    nRepeatsOption->_type = CA_ARG_INT;
    nRepeatsOption->_value = &nRepeats;
	nRepeatsOption->_usage = "<n>";
    sampleArgs->AddOption(nRepeatsOption);
	delete nRepeatsOption;

	Option* nKLoopsOption = new Option;
    CHECK_ALLOCATION(nKLoopsOption, "Memory allocation error.\n");
    nKLoopsOption->_sVersion = "nk";
    nKLoopsOption->_lVersion = "numKernelLoops";
    nKLoopsOption->_description = "Number of loops in kernel";
    nKLoopsOption->_type = CA_ARG_INT;
    nKLoopsOption->_value = &nKLoops;
	nKLoopsOption->_usage = "<n>";
    sampleArgs->AddOption(nKLoopsOption);
	delete nKLoopsOption;

	Option* nWFOption = new Option;
    CHECK_ALLOCATION(nWFOption, "Memory allocation error.\n");
    nWFOption->_sVersion = "nw";
    nWFOption->_lVersion = "numWavefronts";
    nWFOption->_description = "# of wave fronts per SIMD (can't be 0) (default: 7)";
    nWFOption->_type = CA_ARG_INT;
    nWFOption->_value = &nWF;
	nWFOption->_usage = "<n>";
    sampleArgs->AddOption(nWFOption);
	delete nWFOption;

	Option* whichTestOption = new Option;
    CHECK_ALLOCATION(whichTestOption, "Memory allocation error.\n");
    whichTestOption->_sVersion = "ty";
    whichTestOption->_lVersion = "testType";
    whichTestOption->_description = "Type of test:\n" 
                 "							0 clEnqueue[Map,Unmap]\n" 
                 "							1 clEnqueue[Read,Write]\n" 
                 "							2 clEnqueueCopy\n" 
                 "							3 clEnqueue[Read,Write], prepinned\n";
    whichTestOption->_type = CA_ARG_INT;
    whichTestOption->_value = &whichTest;
	whichTestOption->_usage = "<n>";
    sampleArgs->AddOption(whichTestOption);
	delete whichTestOption;

	Option* pcieOption = new Option;
    CHECK_ALLOCATION(pcieOption, "Memory allocation error.\n");
    pcieOption->_sVersion = "dma";
    pcieOption->_lVersion = "pcie";
    pcieOption->_description = "Measure PCIe/interconnect bandwidth";
    pcieOption->_type = CA_NO_ARGUMENT;
    pcieOption->_value = &signalA;
    sampleArgs->AddOption(pcieOption);
	delete pcieOption;

	Option* noblockOption = new Option;
    CHECK_ALLOCATION(noblockOption, "Memory allocation error.\n");
    noblockOption->_sVersion = "nob";
    noblockOption->_lVersion = "noblock";
    noblockOption->_description = "When -pcie is active, measure PCIe/interconnect\n" 
			"							bandwidth using multiple back-to-back asynchronous\n"
			"							buffer copies\n";
    noblockOption->_type = CA_NO_ARGUMENT;
    noblockOption->_value = &signalB;
    sampleArgs->AddOption(noblockOption);
	delete noblockOption;

	Option* nSkipOption = new Option;
    CHECK_ALLOCATION(nSkipOption, "Memory allocation error.\n");
    nSkipOption->_sVersion = "s";
    nSkipOption->_lVersion = "nSkip";
    nSkipOption->_description = "Skip first <n> timings for average (default: 1)";
    nSkipOption->_type = CA_ARG_INT;
    nSkipOption->_value = &nSkip;
	nSkipOption->_usage = "<n>";
    sampleArgs->AddOption(nSkipOption);
	delete nSkipOption;

	Option* printLogOption = new Option;
    CHECK_ALLOCATION(printLogOption, "Memory allocation error.\n");
    printLogOption->_sVersion = "l";
    printLogOption->_lVersion = "printLog";
    printLogOption->_description = "Print complete timing log";
    printLogOption->_type = CA_NO_ARGUMENT;
    printLogOption->_value = &printLog;
    sampleArgs->AddOption(printLogOption);
	delete printLogOption;

	char description[] = "Memory flags. OK to use multiple\n"
		"							0 CL_MEM_READ_ONLY\n"
		"							1 CL_MEM_WRITE_ONLY\n"
		"							2 CL_MEM_READ_WRITE\n"
		"							3 CL_MEM_USE_HOST_PTR\n"
		"							4 CL_MEM_COPY_HOST_PTR\n"
		"							5 CL_MEM_ALLOC_HOST_PTR\n"
		"							6 CL_MEM_USE_PERSISTENT_MEM_AMD\n"
		"							7 CL_MEM_HOST_WRITE_ONLY\n"
		"							8 CL_MEM_HOST_READ_ONLY\n"
		"							9 CL_MEM_HOST_NO_ACCESS\n";

	Option* inFlagNumOption = new Option;
    CHECK_ALLOCATION(inFlagNumOption, "Memory allocation error.\n");
    inFlagNumOption->_sVersion = "if";
    inFlagNumOption->_lVersion = "inputflag";
    inFlagNumOption->_description = description;
    inFlagNumOption->_type = CA_ARG_INT;
    inFlagNumOption->_value = &inFlagNum;
	inFlagNumOption->_usage = "<n>";
    sampleArgs->AddOption(inFlagNumOption);
	delete inFlagNumOption;

	Option* outFlagNumOption = new Option;
    CHECK_ALLOCATION(outFlagNumOption, "Memory allocation error.\n");
    outFlagNumOption->_sVersion = "of";
    outFlagNumOption->_lVersion = "outputflag";
    outFlagNumOption->_description = description;
    outFlagNumOption->_type = CA_ARG_INT;
    outFlagNumOption->_value = &outFlagNum;
	outFlagNumOption->_usage = "<n>";
    sampleArgs->AddOption(outFlagNumOption);
	delete outFlagNumOption;

	Option* copyFlagNumOption = new Option;
    CHECK_ALLOCATION(copyFlagNumOption, "Memory allocation error.\n");
    copyFlagNumOption->_sVersion = "cf";
    copyFlagNumOption->_lVersion = "copyflag";
    copyFlagNumOption->_description = description;
    copyFlagNumOption->_type = CA_ARG_INT;
    copyFlagNumOption->_value = &copyFlagNum;
	copyFlagNumOption->_usage = "<n>";
    sampleArgs->AddOption(copyFlagNumOption);
	delete copyFlagNumOption;

	Option* doHostOption = new Option;
    CHECK_ALLOCATION(doHostOption, "Memory allocation error.\n");
    doHostOption->_sVersion = "hbw";
    doHostOption->_lVersion = "enablehostbw";
    doHostOption->_description = "enable/disable host mem B/W baseline. 0 or 1";
    doHostOption->_type = CA_ARG_INT;
    doHostOption->_value = &doHost;
    sampleArgs->AddOption(doHostOption);
	delete doHostOption;

	Option* mapRWOption = new Option;
    CHECK_ALLOCATION(mapRWOption, "Memory allocation error.\n");
    mapRWOption->_sVersion = "m";
    mapRWOption->_lVersion = "mapRW";
    mapRWOption->_description = "always map as MAP_READ | MAP_WRITE";
    mapRWOption->_type = CA_NO_ARGUMENT;
    mapRWOption->_value = &mapRW;
    sampleArgs->AddOption(mapRWOption);
	delete mapRWOption;

	Option* timingsOption = new Option;
    CHECK_ALLOCATION(timingsOption, "Memory allocation error.\n");
    timingsOption->_sVersion = "t";
    timingsOption->_lVersion = "timings";
    timingsOption->_description = "Print all timings including setup-time";
    timingsOption->_type = CA_NO_ARGUMENT;
    timingsOption->_value = &timings;
    sampleArgs->AddOption(timingsOption);
	delete timingsOption;

    return SDK_SUCCESS;
}

int BufferBandwidth::run()
{
	#ifdef MEM_MULTICORE
		launchThreads();
	#endif

    if( doHost )
       assessHostMemPerf( memIn, memOut, nBytes );
	if(signalA)
	{
		if(signalB)
		{
			runPCIeTestNoblock();
		}
		else
		{
			runPCIeTest();
		}
	}
	else
	{
		switch( whichTest )
		{
		   case 0: runMapTest(); break;	
		   case 1: runReadWriteTest(); break;
		   case 2: runCopyTest(); break;
		   case 3: runMappedReadWriteTest(); break;
		}
	}

	if((!signalA) && signalB) 
	{
		std::cout<<"-noblock will only work when -pcie or -dma is active!"<<std::endl;
		exit(SDK_FAILURE);
	}
    printStats();

	#ifdef MEM_MULTICORE
		shutdownThreads();
	#endif

    if( vFailure )
    {
        std::cerr << "Failed!" << std::endl;
		return SDK_FAILURE;
    }

    std::cout << "Passed!" << std::endl;
	
    return SDK_SUCCESS;
}

int main(int argc, char * argv[])
{
    BufferBandwidth clBufferBandwidth;

    if(clBufferBandwidth.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBufferBandwidth.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBufferBandwidth.sampleArgs->isDumpBinaryEnabled())
    {
        return clBufferBandwidth.genBinaryImage();
    }
    else
    {
        if(clBufferBandwidth.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(clBufferBandwidth.run() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
		
        if(clBufferBandwidth.cleanup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

    return SDK_SUCCESS;
}

