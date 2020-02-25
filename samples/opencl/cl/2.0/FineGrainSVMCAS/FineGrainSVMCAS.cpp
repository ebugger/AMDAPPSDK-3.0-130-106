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


#include "FineGrainSVMCAS.hpp"

int FineGrainSVMCAS::setupFineGrainSVMCAS()
{
    // allocate and init memory used by host
    cl_uint sizeBytes = length * sizeof(cl_float);

    list = (int *) clSVMAlloc(context, CL_MEM_SVM_FINE_GRAIN_BUFFER | CL_MEM_SVM_ATOMICS, (2*length)*sizeof(int), 4);
    CHECK_ALLOCATION(list, "Failed to allocate SVM memory. (list)");

    return SDK_SUCCESS;
}

int
FineGrainSVMCAS::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("FineGrainSVMCAS_Kernels.cl");
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
FineGrainSVMCAS::setupCL(void)
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
    buildData.kernelName = std::string("FineGrainSVMCAS_Kernels.cl");
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
    fine_grain_cas_link_kernel = clCreateKernel(program, "linkKernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::fine_grain_ldstore failed.");

    /* get default work group size */
    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_cas_link_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    // get a kernel object handle for a kernel with the given name
    fine_grain_cas_unlink_kernel = clCreateKernel(program, "unlinkKernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel::fine_grain_unlink_kernel failed.");

    /* get default work group size */
    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_cas_unlink_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    return SDK_SUCCESS;
}

int
FineGrainSVMCAS::runFineGrainSVMCASLinkKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - randomBuffer
    int status = clSetKernelArgSVMPointer(
				fine_grain_cas_link_kernel,
				0,
				list);
    CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(buffer)");
 
    for (cl_int i=0;i<2*length;i++)
	list[i] = 0;	

    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 fine_grain_cas_link_kernel,
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

    for (int i=length; i < 2*length; i++) 
	{
		int head = list[0];
		do {
			list[i] = head;
		} while (!std::atomic_compare_exchange_strong((std::atomic<int>*)&list[0], &head, i));
    }

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

    if(!sampleArgs->quiet)
        for (cl_int k=0;k<2*length;k++) 
             std::cout << " k = " << k << " list[" << k << "] = " <<  list[k] << "\n";

    int l = 0;
    for(cl_int k=0;k<2*length;k++) {
        l +=  list[k];
    }

    if (l == (2*length*(2*length-1)/2))
	push_pass = 1;

    if(!sampleArgs->quiet)
	std::cout << " l = " << l;

    return SDK_SUCCESS;
}

int
FineGrainSVMCAS::runFineGrainSVMCASUnLinkKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - randomBuffer
    int status = clSetKernelArgSVMPointer(
				fine_grain_cas_unlink_kernel,
				0,
				list);
    CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(list)");
 
    for (cl_int i=0;i<2*length;i++)
	list[i] = 0;	

    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 fine_grain_cas_unlink_kernel,
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

    int next;
    for (cl_int i=length;i<2*length;i++) {
	int head = list[0];
	do {
 		if (head == 0) break;
		next = list[list[0]];
	} while (!std::atomic_compare_exchange_strong ((std::atomic<int>*)&list[0], &head, next ));		
}

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

    if (list[0] == 0)
	pop_pass = 1;

    return SDK_SUCCESS;
}

int
FineGrainSVMCAS::runCLKernels(void)
{
    cl_int status;

    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_cas_link_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    //run the work-group level scan kernel
    status = runFineGrainSVMCASLinkKernel();

    status =  kernelInfo.setKernelWorkGroupInfo(fine_grain_cas_unlink_kernel,
              devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

    status = runFineGrainSVMCASUnLinkKernel();

    return SDK_SUCCESS;
}

int FineGrainSVMCAS::initialize()
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

int FineGrainSVMCAS::setup()
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

    if(setupFineGrainSVMCAS() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}


int FineGrainSVMCAS::run()
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

int FineGrainSVMCAS::verifyResults()
{
  int status = SDK_SUCCESS;
  if(sampleArgs->verify)
    {
	if (push_pass && pop_pass)
		std::cout << "Passed! \n ";
	else
		std::cout << "Failed! \n ";
    }
  return status;
}

void FineGrainSVMCAS::printStats()
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

int FineGrainSVMCAS::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status = 0;

    status = clReleaseKernel(fine_grain_cas_link_kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

    status = clReleaseKernel(fine_grain_cas_unlink_kernel);
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

    FineGrainSVMCAS clFineGrainSVMCAS;
	int retStatus;

    // Initialize
    if(clFineGrainSVMCAS.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clFineGrainSVMCAS.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clFineGrainSVMCAS.sampleArgs->isDumpBinaryEnabled())
    {
        //GenBinaryImage
        return clFineGrainSVMCAS.genBinaryImage();
    }

    // Setup
	retStatus = clFineGrainSVMCAS.setup();
    if(retStatus != SDK_SUCCESS)
    {
        return retStatus;
    }

    // Run
    if(clFineGrainSVMCAS.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // VerifyResults
    if(clFineGrainSVMCAS.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Cleanup
    if (clFineGrainSVMCAS.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clFineGrainSVMCAS.printStats();
    return SDK_SUCCESS;
}
