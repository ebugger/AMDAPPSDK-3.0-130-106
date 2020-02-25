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


#include "BuiltInScan.hpp"

int BuiltInScan::setupBuiltInScan()
{
    // allocate and init memory used by host
    cl_uint sizeBytes = length * sizeof(cl_float);

    input = (cl_float *) malloc(sizeBytes);
    CHECK_ALLOCATION(input, "Failed to allocate host memory. (input)");

    // random initialisation of input
    fillRandom<cl_float>(input, length, 1, 0, 10);

    if(sampleArgs->verify)
    {
        verificationOutput = (cl_float *) malloc(sizeBytes);
        CHECK_ALLOCATION(verificationOutput,
                         "Failed to allocate host memory. (verificationOutput)");
        memset(verificationOutput, 0, sizeBytes);
    }

    // Unless quiet mode has been enabled, print the INPUT array
    if(!sampleArgs->quiet)
    {
        printArray<cl_float>("Input : ", input, length, 1);
    }

    return SDK_SUCCESS;
}

int
BuiltInScan::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("BuiltInScan_Kernels.cl");
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
BuiltInScan::setupCL(void)
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

	int majorRev, minorRev;
	if (sscanf(deviceInfo.deviceVersion, "OpenCL %d.%d", &majorRev, &minorRev) == 2) 
	{
        if (majorRev < 2) {
            OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
        }
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
    buildData.kernelName = std::string("BuiltInScan_Kernels.cl");
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
    group_kernel = clCreateKernel(program, "group_scan_kernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::group_scan_kernel failed.");

    global_kernel = clCreateKernel(program, "global_scan_kernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::global_scan_kernel failed.");

    /* get default work group size */
    status =  kernelInfo.setKernelWorkGroupInfo(group_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    //sanity check on length
    cl_uint wg_size = kernelInfo.kernelWorkGroupSize;
    cl_int  rounded_length;

    if((length%wg_size) || (length <= 0))
      {

	rounded_length = ((length/wg_size) + 1)*wg_size;

	std::cout << "----------------------------------";
	std::cout << "----------------------------------" <<std::endl;
	std::cout << "MESSAGE: Input length should be positive multiple of " << wg_size;
	std::cout << "." << std::endl;

	std::cout << "         Given(or default) input length " << length;
	std::cout << " is rounded to " << rounded_length;
	std::cout << "." << std::endl;
	std::cout << "----------------------------------";
	std::cout << "----------------------------------" <<std::endl;

	length = rounded_length;
      }

    inputBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY,
                      sizeof(cl_float) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer)");

    outputBuffer = clCreateBuffer(
                       context,
                       CL_MEM_WRITE_ONLY,
                       sizeof(cl_float) * length,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBuffer)");

    return SDK_SUCCESS;
}

cl_uint 
BuiltInScan::findStages(cl_uint data_size, cl_uint wg_size)
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
  
  return (cl_uint)(log2data - log2wg); 
}

int
BuiltInScan::runGroupKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - inputBuffer
    int status = clSetKernelArg(
				group_kernel,
				0,
				sizeof(cl_mem),
				(void *)&inputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBuffer)");

    // 2nd argument to the kernel - outputBuffer
    status = clSetKernelArg(
			    group_kernel,
			    1,
			    sizeof(cl_mem),
			    (void *)&outputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");
    
    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 group_kernel,
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
BuiltInScan::runGlobalKernel()
{
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;

    // Set number of threads needed for global_kernel.
    size_t globalThreads = length/2;

    //find number of stages
    cl_uint stages = (cl_uint)findStages(length,localThreads);

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - Global Buffer
    for(cl_uint k = 0; k < stages; ++k)
      {
	int status = clSetKernelArg(global_kernel,
				    0,
				    sizeof(cl_mem),
				    (void *)&outputBuffer);
	CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");
	
	// 2nd argument to the kernel - offset
	status = clSetKernelArg(global_kernel,
				1,
				sizeof(cl_int),
				(void*)&k);
	CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(offset)");
	
	// Run the kernel
	cl_event ndrEvt;
	status = clEnqueueNDRangeKernel(
					commandQueue,
					global_kernel,
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
      }
    
    return SDK_SUCCESS;
}

int
BuiltInScan::runCLKernels(void)
{
    cl_int status;

    status =  kernelInfo.setKernelWorkGroupInfo(group_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    //run the work-group level scan kernel
    status = runGroupKernel();

    //run global kernels for stage decided by input length
    status = runGlobalKernel();

    return SDK_SUCCESS;
}

void
BuiltInScan::builtInScanCPUReference(
    cl_float * output,
    cl_float * input,
    const cl_uint length)
{
    output[0] = input[0];

    for(cl_uint i = 1; i< length; ++i)
    {
        output[i] = input[i] + output[i-1];
    }
}

int BuiltInScan::initialize()
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

int BuiltInScan::setup()
{

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    int retStatus = setupCL();
    if(retStatus != SDK_SUCCESS)
    {
        return retStatus;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    if(setupBuiltInScan() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Move data host to device
    cl_event writeEvt;
    cl_int   status;
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 inputBuffer,
                 CL_FALSE,
                 0,
                 sizeof(cl_float) * length,
                 input,
                 0,
                 NULL,
                 &writeEvt);

    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed.(inputBuffer)");
    status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");
    status = waitForEventAndRelease(&writeEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvt) Failed");

    return SDK_SUCCESS;
}


int BuiltInScan::run()
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

int BuiltInScan::verifyResults()
{
  int status = SDK_SUCCESS;
  if(sampleArgs->verify)
    {
      // Read the device output buffer
      cl_float *ptrOutBuff;
      int status = mapBuffer(outputBuffer, 
			     ptrOutBuff,  
			     length * sizeof(cl_float),
			     CL_MAP_READ);
      CHECK_ERROR(status, SDK_SUCCESS, 
		  "Failed to map device buffer.(resultBuf)");

      // reference implementation
      builtInScanCPUReference(verificationOutput, input, length);
      
      // compare the results and see if they match
      float epsilon = length * 1e-7f;
      if(compare(ptrOutBuff, verificationOutput, length, epsilon))
        {
	  std::cout << "Passed!\n" << std::endl;
	  status = SDK_SUCCESS;
        }
        else
	  {
            std::cout << "Failed\n" << std::endl;
            status = SDK_FAILURE;
	  }
      
      if(!sampleArgs->quiet)
        {
	  printArray<cl_float>("Output : ", ptrOutBuff, length, 1);
        }
      
      // un-map outputBuffer
        int result = unmapBuffer(outputBuffer, ptrOutBuff);
        CHECK_ERROR(result, SDK_SUCCESS,
                    "Failed to unmap device buffer.(resultCountBuf)");
    }
  return status;
}

void BuiltInScan::printStats()
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

int BuiltInScan::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status = 0;

    status = clReleaseKernel(group_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

    status = clReleaseKernel(global_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer)");

    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBuffer)");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");

    return SDK_SUCCESS;
}

template<typename T>
int BuiltInScan::mapBuffer(cl_mem deviceBuffer, T* &hostPointer,
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
BuiltInScan::unmapBuffer(cl_mem deviceBuffer, void* hostPointer)
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
	int status = 0;
    BuiltInScan clBuiltInScan;
    // Initialize
    if(clBuiltInScan.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBuiltInScan.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_EXPECTED_FAILURE;
    }

    if(clBuiltInScan.sampleArgs->isDumpBinaryEnabled())
    {
        //GenBinaryImage
        return clBuiltInScan.genBinaryImage();
    }

    // Setup
	status = clBuiltInScan.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    // Run
    if(clBuiltInScan.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // VerifyResults
    if(clBuiltInScan.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Cleanup
    if (clBuiltInScan.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clBuiltInScan.printStats();
    return SDK_SUCCESS;
}
