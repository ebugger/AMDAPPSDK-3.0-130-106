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


#include "FineGrainSVM.hpp"

int FineGrainSVM::setupFineGrainSVM()
{
    // allocate and init memory used by host
    cl_uint sizeBytes = length * sizeof(cl_float);

    buffer = (int *) clSVMAlloc(context, CL_MEM_SVM_FINE_GRAIN_BUFFER, (length+1)*sizeof(int), 0);
    CHECK_ALLOCATION(buffer, "Failed to allocate SVM memory. (buffer)");
    *buffer = 0;

    atomicBuffer = (int *) clSVMAlloc(context, CL_MEM_SVM_FINE_GRAIN_BUFFER | CL_MEM_SVM_ATOMICS, (length+1)*sizeof(int), 0);
    CHECK_ALLOCATION(atomicBuffer, "Failed to allocate SVM memory. (atomicBuffer)");

    memset(atomicBuffer, 0, (length+1)*sizeof(int));

    return SDK_SUCCESS;
}

int
FineGrainSVM::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("FineGrainSVM_Kernels.cl");
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
FineGrainSVM::setupCL(void)
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

	//check SVM capabilities Finegrain and Atomics

	if (!(deviceInfo.svmcaps & CL_DEVICE_SVM_ATOMICS))
	{
		OPENCL_EXPECTED_ERROR("Unsupported device! Device does not support SVM Atomics");
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
    buildData.kernelName = std::string("FineGrainSVM_Kernels.cl");
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
    fine_grain_ldstore = clCreateKernel(program, "fine_grain_ldstore", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::fine_grain_ldstore failed.");

    /* get default work group size */
    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_ldstore,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");
	
    return SDK_SUCCESS;
}

int
FineGrainSVM::runFineGrainSVMKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;
    
    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - randomBuffer
    int status = clSetKernelArgSVMPointer(
				fine_grain_ldstore,
				0,
				buffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(buffer)");

    status = clSetKernelArgSVMPointer(
				fine_grain_ldstore,
				1,
				atomicBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(atomicBuffer)");

    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 fine_grain_ldstore,
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

    for (cl_uint i=0;i<length;i++)
    {
	buffer[i] = 64;
    	std::atomic_store_explicit ((std::atomic<int>*)&atomicBuffer[i], 99, std::memory_order_release);
    }

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

    return SDK_SUCCESS;
}

int
FineGrainSVM::runCLKernels(void)
{
    cl_int status;

    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_ldstore,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    //run the work-group level scan kernel
    status = runFineGrainSVMKernel();

    return SDK_SUCCESS;
}

int FineGrainSVM::initialize()
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

int FineGrainSVM::setup()
{
	int retStatus;

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	retStatus = setupCL();
    if (retStatus != SDK_SUCCESS)
    {
        return retStatus;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    if(setupFineGrainSVM() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}


int FineGrainSVM::run()
{
    int status = 0;
    
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

int FineGrainSVM::verifyResults()
{
  int status = SDK_SUCCESS;
  
  if(sampleArgs->verify)
  {
	int passed=1;
	for (cl_uint i=0;i<length;i++)
		if (buffer[i] != (64+i))
			passed = 0;

	if (passed)
		std::cout << "Passed \n";
	else
		std::cout << "Failed \n";
  }

  return status;
}

void FineGrainSVM::printStats()
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

int FineGrainSVM::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status = 0;

    status = clReleaseKernel(fine_grain_ldstore);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{

    FineGrainSVM clFineGrainSVM;
	int retStatus;

    // Initialize
    if(clFineGrainSVM.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clFineGrainSVM.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clFineGrainSVM.sampleArgs->isDumpBinaryEnabled())
    {
        //GenBinaryImage
        return clFineGrainSVM.genBinaryImage();
    }

    // Setup
	retStatus = clFineGrainSVM.setup();
    if(retStatus != SDK_SUCCESS)
    {
        return retStatus;
    }

    // Run
    if(clFineGrainSVM.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // VerifyResults
    if(clFineGrainSVM.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Cleanup
    if (clFineGrainSVM.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clFineGrainSVM.printStats();
    return SDK_SUCCESS;
}
