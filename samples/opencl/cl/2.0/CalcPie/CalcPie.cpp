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


#include "CalcPie.hpp"

int CalcPie::setupCalcPie()
{
    // allocate and init memory used by host
    cl_uint sizeBytes = length * sizeof(cl_float);

    randomX = (cl_float *) malloc(sizeBytes);
    CHECK_ALLOCATION(randomX, "Failed to allocate host memory. (input)");

    randomY = (cl_float *) malloc(sizeBytes);
    CHECK_ALLOCATION(randomY, "Failed to allocate host memory. (input)");

    // random initialisation of input
    for (cl_uint i=0;i<length;i++) {
		randomX[i] = (float)rand()/(float)RAND_MAX;
		randomY[i] = (float)rand()/(float)RAND_MAX;
    }

    return SDK_SUCCESS;
}

int
CalcPie::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("CalcPie_Kernels.cl");
    binaryData.flagsStr = std::string("");
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }
    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    return status;
}

int
CalcPie::setupCL(void)
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

	//check 2.x compatibility
	bool check2_x = deviceInfo.checkOpenCL2_XCompatibility();

	if (!check2_x)
	{
		OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
	}

    // Create command queue
    cl_queue_properties prop[] = {0};
    commandQueue = clCreateCommandQueueWithProperties(context,
                                        devices[sampleArgs->deviceId],
                                        prop,
                                        &status);
    CHECK_OPENCL_ERROR(status, "clCreateCommandQueue failed.");



    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("CalcPie_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
    buildData.flagsStr = std::string("");

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
    calc_pie_kernel = clCreateKernel(program, "calc_pie_kernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::calc_pie_kernel failed.");

    /* get default work group size */
    status =  kernelInfo.setKernelWorkGroupInfo(calc_pie_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    randomXBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY,
                      sizeof(cl_float) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (randomBuffer)");

    randomYBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY,
                      sizeof(cl_float) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (randomBuffer)");

    insideBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_WRITE,
                      sizeof(cl_int),
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (insideBuffer)");

    return SDK_SUCCESS;
}

int
CalcPie::runCalcPieKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;
    cl_event writeEvt;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - randomBuffer
    int status = clSetKernelArg(
				calc_pie_kernel,
				0,
				sizeof(cl_mem),
				(void *)&randomXBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(randomBuffer)");

    status = clSetKernelArg(
				calc_pie_kernel,
				1,
				sizeof(cl_mem),
				(void *)&randomYBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(randomBuffer)");

    // 2nd argument to the kernel - output buffer which holds "inside" hit count
    status = clSetKernelArg(
			    calc_pie_kernel,
			    2,
			    sizeof(cl_mem),
			    (void *)&insideBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inside_count)");
    
    inside = 0;
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 insideBuffer,
                 CL_FALSE,
                 0,
                 sizeof(cl_int),
                 &inside,
                 0,
                 NULL,
                 &writeEvt);

    status = waitForEventAndRelease(&writeEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvt) Failed");

    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed.(randomBuffer)");

    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 calc_pie_kernel,
                 1,
                 NULL,
                 &globalThreads,
                 &localThreads,
                 0,
                 NULL,
                 &ndrEvt);
    CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");

    status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

    return SDK_SUCCESS;
}

int
CalcPie::runCLKernels(void)
{
    cl_int status;

    status =  kernelInfo.setKernelWorkGroupInfo(calc_pie_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    //run the work-group level scan kernel
    status = runCalcPieKernel();

    return SDK_SUCCESS;
}

void
CalcPie::calcPieCPUReference(cl_float *pie)
{
    int insidecnt = 0;
	for (cl_uint i = 0; i < length; i++)
	{
		float r = sqrt((randomX[i] * randomX[i]) + (randomY[i] * randomY[i]));
		if (r <= 1)
			insidecnt++;
	}

	*pie = (cl_float)(insidecnt * 4) / length;
}

int CalcPie::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    Option* array_length = new Option;
    CHECK_ALLOCATION(array_length, "Memory allocation error. (array_length)");

    array_length->_sVersion = "x";
    array_length->_lVersion = "length";
    array_length->_description = "Length of the input array";
    array_length->_type = CA_ARG_INT;
    array_length->_value = &length;
    sampleArgs->AddOption(array_length);
    delete array_length;

    Option* num_iterations = new Option;
    CHECK_ALLOCATION(num_iterations, "Memory allocation error. (num_iterations)");

    num_iterations->_sVersion = "i";
    num_iterations->_lVersion = "iterations";
    num_iterations->_description = "Number of iterations for kernel execution";
    num_iterations->_type = CA_ARG_INT;
    num_iterations->_value = &iterations;

    sampleArgs->AddOption(num_iterations);
    delete num_iterations;

    return SDK_SUCCESS;
}

int CalcPie::setup()
{
	int   status;
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	status = setupCL();
    if (status != SDK_SUCCESS)
    {
        return status;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    if(setupCalcPie() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Move data host to device
    cl_event writeEvtX;
	cl_event writeEvtY;
    
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 randomXBuffer,
                 CL_FALSE,
                 0,
                 sizeof(cl_float) * length,
                 randomX,
                 0,
                 NULL,
                 &writeEvtX);

    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed.(randomBuffer)");

	status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");

    status = waitForEventAndRelease(&writeEvtX);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvtX) Failed");

    status = clEnqueueWriteBuffer(
                 commandQueue,
                 randomYBuffer,
                 CL_FALSE,
                 0,
                 sizeof(cl_float) * length,
                 randomY,
                 0,
                 NULL,
                 &writeEvtY);

    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed.(randomBuffer)");


    status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");

    status = waitForEventAndRelease(&writeEvtY);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvtY) Failed");

    return SDK_SUCCESS;
}


int CalcPie::run()
{
    int status = 0;

    //warm up run
    if(runCLKernels() != SDK_SUCCESS)
      {
	return SDK_FAILURE;
      }
    
    std::cout << "Executing kernel for " << iterations
              << " iterations" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < iterations; i++)
    {
        // Arguments are set and execution call is enqueued on command buffer
        if(runCLKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

    sampleTimer->stopTimer(timer);
    kernelTime = (double)(sampleTimer->readTimer(timer));

    return SDK_SUCCESS;
}

int CalcPie::verifyResults()
{
  int status = SDK_SUCCESS;
  
  if(sampleArgs->verify)
  {
      // Read the device output buffer
      cl_float pieValue, gpuPie;
      cl_int *gpuInsideCount;
      int status = mapBuffer(insideBuffer,
                             gpuInsideCount,
                             sizeof(cl_int),
                             CL_MAP_READ);
      CHECK_ERROR(status, SDK_SUCCESS,
                  "Failed to map device buffer.(resultBuf)");
      gpuPie = (cl_float)(*gpuInsideCount * 4)/length;

	  status = unmapBuffer(insideBuffer, gpuInsideCount);
	  CHECK_ERROR(status, SDK_SUCCESS,
                  "Failed to unmap device buffer.(resultBuf)");

      // reference implementation
      calcPieCPUReference(&pieValue);

      if(!sampleArgs->quiet)
      {
		std::cout << "GPUInsideCount: " << *gpuInsideCount;
        std::cout << " CPUValue :"  << pieValue << " GPUValue: " << gpuPie << std::endl;
      }

      // compare the results and see if they match
      float epsilon = 1e-2f;
      if(::fabs(gpuPie - pieValue) <=  epsilon)
      {
		std::cout << "Passed!\n" << std::endl;
		status = SDK_SUCCESS;
      }
      else
	  {
		std::cout << "Failed\n" << std::endl;
        status = SDK_FAILURE;
	  }
      
  }

  return status;
}

void CalcPie::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] =
        {
            "Samples",
            "Setup Time(sec)",
            "Avg. kernel time (sec)",
            "Samples/sec"
        };
        std::string stats[4];
        double avgKernelTime = kernelTime / iterations;

        stats[0] = toString(length, std::dec);
        stats[1] = toString(setupTime, std::dec);
        stats[2] = toString(avgKernelTime, std::dec);
        stats[3] = toString((length/avgKernelTime), std::dec);

        printStatistics(strArray, stats, 4);
    }
}

int CalcPie::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status = 0;

    status = clReleaseKernel(calc_pie_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(calc_pie_kernel)");

	status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

    status = clReleaseMemObject(randomXBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(randomBuffer)");

    status = clReleaseMemObject(randomYBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(randomBuffer)");
	
	status = clReleaseMemObject(insideBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(insideBuffer)");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");

	// release program resources
    FREE(randomX);
    FREE(randomY);

    return SDK_SUCCESS;
}

template<typename T>
int CalcPie::mapBuffer(cl_mem deviceBuffer, T* &hostPointer,
                         size_t sizeInBytes, cl_map_flags flags)
{
    cl_int status;
    hostPointer = (T*) clEnqueueMapBuffer(commandQueue,
                                          deviceBuffer,
                                          CL_TRUE,
                                          flags,
                                          0,
                                          sizeInBytes,
                                          0,
                                          NULL,
                                          NULL,
                                          &status);
    CHECK_OPENCL_ERROR(status, "clEnqueueMapBuffer failed");

    status = clFinish(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFinish failed.");

    return SDK_SUCCESS;
}

int
CalcPie::unmapBuffer(cl_mem deviceBuffer, void* hostPointer)
{
    cl_int status;
    status = clEnqueueUnmapMemObject(commandQueue,
                                     deviceBuffer,
                                     hostPointer,
                                     0,
                                     NULL,
                                     NULL);
    CHECK_OPENCL_ERROR(status, "clEnqueueUnmapMemObject failed");

    status = clFinish(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFinish failed.");

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{

    CalcPie clCalcPie;
	int status = 0;
	
    // Initialize
    if(clCalcPie.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clCalcPie.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clCalcPie.sampleArgs->isDumpBinaryEnabled())
    {
        //GenBinaryImage
        return clCalcPie.genBinaryImage();
    }

    // Setup
	status = clCalcPie.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    // Run
    if(clCalcPie.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // VerifyResults
    if(clCalcPie.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Cleanup
    if (clCalcPie.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clCalcPie.printStats();
    return SDK_SUCCESS;
}
