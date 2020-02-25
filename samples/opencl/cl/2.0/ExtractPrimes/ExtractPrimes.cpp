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


#include "ExtractPrimes.hpp"

int ExtractPrimes::setupExtractPrimes()
{
    // allocate and init memory used by host
    cl_uint sizeBytes = length * sizeof(cl_int);

    input = (cl_int *) malloc(sizeBytes);
    CHECK_ALLOCATION(input, "Failed to allocate host memory. (input)");

    primes = (cl_int *) malloc(sizeBytes);
    CHECK_ALLOCATION(input, "Failed to allocate host memory. (primes)");

    outPrimes = (cl_int *) malloc(sizeBytes);
    CHECK_ALLOCATION(input, "Failed to allocate host memory. (outPrimes)");

    // random initialisation of input
    fillRandom<cl_int>(input, length, 1, 0, 100);

    if(sampleArgs->verify)
    {
        verificationOutput = (cl_int *) malloc(sizeBytes);
        CHECK_ALLOCATION(verificationOutput,
                         "Failed to allocate host memory. (verificationOutput)");
        memset(verificationOutput, 0, sizeBytes);
    }

    // Unless quiet mode has been enabled, print the INPUT array
    if(!sampleArgs->quiet)
    {
        printArray<cl_int>("Input : ", input, length, 1);
    }

    return SDK_SUCCESS;
}

int
ExtractPrimes::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("ExtractPrimes_Kernels.cl");
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
ExtractPrimes::setupCL(void)
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
	if (sscanf_s(deviceInfo.deviceVersion, "OpenCL %d.%d", &majorRev, &minorRev) == 2) 
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


	// Create Device command Queue for en-queuing commands inside the kernel  :
	if(devEnqueue)
	{
		{
		// The block is to move the declaration of prop closer to its use
			cl_queue_properties prop[] = {
			CL_QUEUE_PROPERTIES, CL_QUEUE_ON_DEVICE|CL_QUEUE_ON_DEVICE_DEFAULT,
			CL_QUEUE_SIZE, deviceInfo.maxQueueSize,
			0
			};
			//const cl_queue_properties prop = 0 ;
			devcommandQueue = clCreateCommandQueueWithProperties(
							   context,
							   devices[sampleArgs->deviceId],
							   prop,
							   &status);
			CHECK_OPENCL_ERROR( status, "clCreateCommandQueueWithProperties failed.");
			
		}
	}



    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("ExtractPrimes_Kernels.cl");
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

	if(devEnqueue)
	{	
		// get a kernel object handle for a kernel with the given name
		extract_primes_kernel = clCreateKernel(program, "extract_primes_kernel", &status);
		CHECK_OPENCL_ERROR(status, "clCreateKernel::extract_primes_kernel failed.");

		/* get default work group size */
		status =  kernelInfo.setKernelWorkGroupInfo(extract_primes_kernel,
				  devices[sampleArgs->deviceId]);
		CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");
	}
	else
	{
		// get a kernel object handle for a kernel with the given name
		set_primes_kernel = clCreateKernel(program, "set_primes_kernel", &status);
		CHECK_OPENCL_ERROR(status, "clCreateKernel::set_primes_kernel failed.");

		// get a kernel object handle for a kernel with the given name
		get_primes_kernel = clCreateKernel(program, "get_primes_kernel", &status);
		CHECK_OPENCL_ERROR(status, "clCreateKernel::get_primes_kernel failed.");

		// get a kernel object handle for a kernel with the given name
		group_kernel = clCreateKernel(program, "group_scan_kernel", &status);
		CHECK_OPENCL_ERROR(status, "clCreateKernel::group_scan_kernel failed.");

		global_kernel = clCreateKernel(program, "global_scan_kernel", &status);
		CHECK_OPENCL_ERROR(status, "clCreateKernel::global_scan_kernel failed.");

		/* get default work group size */
		status =  kernelInfo.setKernelWorkGroupInfo(group_kernel,
				  devices[sampleArgs->deviceId]);
		CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

	}
    
    inputBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY,
                      sizeof(cl_int) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer)");

    primesBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_WRITE,
                      sizeof(cl_int) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (primesBuffer)");

    outPrimesBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_WRITE,
                      sizeof(cl_int) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outPrimesBuffer)");

    outputBuffer = clCreateBuffer(
                       context,
                       CL_MEM_WRITE_ONLY,
                       sizeof(cl_int) * length,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBuffer)");


	errorBuffer = clCreateBuffer(
                       context,
                       CL_MEM_READ_WRITE,
                       sizeof(cl_uint4),
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (errorBuffer)");

    return SDK_SUCCESS;
}

cl_uint 
ExtractPrimes::findStages(cl_uint data_size, cl_uint wg_size)
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

int ExtractPrimes::extractPrimesEnqueueKernel()
{

	size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

	int status = mapBuffer( errorBuffer, errorBuffer_cpu, sizeof(cl_uint4), CL_MAP_WRITE );
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(errorBuffer)");


	memset(errorBuffer_cpu,0,sizeof(cl_uint4));


	status = unmapBuffer( errorBuffer, errorBuffer_cpu );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(errorBuffer)");

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - inputBuffer
    status = clSetKernelArg(
				extract_primes_kernel,
				0,
				sizeof(cl_mem),
				(void *)&inputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBuffer)");

    // 2nd argument to the kernel - primesBuffer
    status = clSetKernelArg(
			    extract_primes_kernel,
			    1,
			    sizeof(cl_mem),
			    (void *)&primesBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(primesBuffer)");

	// 3rd argument to the kernel - outputBuffer
    status = clSetKernelArg(
			    extract_primes_kernel,
			    2,
			    sizeof(cl_mem),
			    (void *)&outputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");

	// 4rd argument to the kernel - outPrimesBuffer
    status = clSetKernelArg(
			    extract_primes_kernel,
			    3,
			    sizeof(cl_mem),
			    (void *)&outPrimesBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outPrimesBuffer)");

	// 5th argument to the kernel - globalThreads
    status = clSetKernelArg(
			    extract_primes_kernel,
			    4,
			    sizeof(size_t),
			    (void *)&globalThreads);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(globalThreads)");

	// 6th argument to the kernel - length
    status = clSetKernelArg(
			    extract_primes_kernel,
			    5,
			    sizeof(cl_uint),
			    (void *)&length);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(length)");

	// 7th argument to the kernel - globalThreads
    status = clSetKernelArg(
			    extract_primes_kernel,
			    6,
			    sizeof(size_t),
			    (void *)&localThreads);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(globalThreads)");


	// 8th argument to the kernel - errorBuffer
    status = clSetKernelArg(
			    extract_primes_kernel,
			    7,
			    sizeof(cl_mem),
			    (void *)&errorBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(errorBuffer)");
    
    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 extract_primes_kernel,
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

	status = clFinish(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFinish failed.(commandQueue)");

	status = mapBuffer( errorBuffer, errorBuffer_cpu, sizeof(cl_uint4) , CL_MAP_WRITE );
        CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(outputBuffer)");


	cl_uint errorCheck		= CL_SUCCESS;
	cl_int errorCodes		= CL_SUCCESS;
	errorCheck				= errorBuffer_cpu[3]; 
	errorCodes				= (cl_int)errorBuffer_cpu[2]; 


	if(errorCheck == ENQUEUE_KERNEL_FAILURE_LEVEL1)   //TODO  Need to update the error codes for OpenCL 2.0 in CLUtil.hpp file .
	{
		CHECK_OPENCL_ERROR(errorCodes, "Enqueueing Kernel in the openCL device Failed in extract_primes_kernel kernel)");
	}
	else if(errorCheck == ENQUEUE_KERNEL_FAILURE_LEVEL2)
	{
		CHECK_OPENCL_ERROR(errorCodes, "Enqueueing Kernel in the openCL device Failed in group_scan_kernel_enqueue kernel)");
	}
	else if(errorCheck == ENQUEUE_KERNEL_FAILURE_LEVEL3)
	{
		CHECK_OPENCL_ERROR(errorCodes, "Enqueueing Kernel in the openCL device Failed in global_scan_kernel_enqueue kernel)");
	}


    return SDK_SUCCESS;
}

int
ExtractPrimes::runSetPrimesKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - inputBuffer
    int status = clSetKernelArg(
				set_primes_kernel,
				0,
				sizeof(cl_mem),
				(void *)&inputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBuffer)");

    // 2nd argument to the kernel - outputBuffer
    status = clSetKernelArg(
			    set_primes_kernel,
			    1,
			    sizeof(cl_mem),
			    (void *)&primesBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(primesBuffer)");
    
    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 set_primes_kernel,
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
ExtractPrimes::runGroupKernel()
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
				(void *)&primesBuffer);
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
ExtractPrimes::runGlobalKernel()
{
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;

    // Set number of threads needed for global_kernel.
    size_t globalThreads = length/2;

    //find number of stages
    cl_uint stages = (cl_uint)findStages(length,(cl_uint)localThreads);

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
ExtractPrimes::runGetPrimesKernel()
{
    size_t dataSize      = length;
    size_t localThreads  = kernelInfo.kernelWorkGroupSize;
    size_t globalThreads = dataSize;

    // Set appropriate arguments to the kernel
    // 1st argument to the kernel - inputBuffer
    int status = clSetKernelArg(
				get_primes_kernel,
				0,
				sizeof(cl_mem),
				(void *)&inputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBuffer)");

    // 2nd argument to the kernel - outputBuffer
    status = clSetKernelArg(
			    get_primes_kernel,
			    1,
			    sizeof(cl_mem),
			    (void *)&outputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");
    
    // 3rd argument to the kernel - outputBuffer
    status = clSetKernelArg(
			    get_primes_kernel,
			    2,
			    sizeof(cl_mem),
			    (void *)&outPrimesBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outPrimesBuffer)");
    
    // Enqueue a kernel run call
    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 get_primes_kernel,
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
ExtractPrimes::runCLKernels(void)
{
    cl_int status;
	
	if(devEnqueue)
	{

		status =  kernelInfo.setKernelWorkGroupInfo(extract_primes_kernel,
				  devices[sampleArgs->deviceId]);
		CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");

		//run ExtractPrimesKernel for the given length
		status = extractPrimesEnqueueKernel();
	}
	else
	{
		status =  kernelInfo.setKernelWorkGroupInfo(group_kernel,
				  devices[sampleArgs->deviceId]);
		CHECK_ERROR(status, SDK_SUCCESS, "setKErnelWorkGroupInfo() failed");
		//run set primes kernels for stage decided by input length
		status = runSetPrimesKernel();

		//run the work-group level scan kernel
		status = runGroupKernel();

		//run global kernels for stage decided by input length
		status = runGlobalKernel();

		//run global kernels for stage decided by input length
		status = runGetPrimesKernel();

	}

    return SDK_SUCCESS;
}

int
ExtractPrimes::isPrime(int num) {
    int k;
    if (num == 0 || num == 1) return 0;
    for (k = 2;k*k<=num;k++)
        if ((num % k) == 0) return 0;
    return 1;
}

void
ExtractPrimes::extractPrimesCPUReference(
    cl_int * output,
    cl_int * input,
    const cl_uint length,
    cl_int *outLength)
{
    int k = 0;
    for(cl_uint i = 0; i< length; ++i)
    {
	if (isPrime(input[i]))
        	output[k++] = input[i];
    }
    *outLength = k;
}

int ExtractPrimes::initialize()
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
    array_length->_description = "Length of the input array. Must be a a power of 2. Sample adjusts the value suitably, if not";
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

	Option* usingKernelEnqueueOption= new Option;
    CHECK_ALLOCATION(usingKernelEnqueueOption, "Memory allocation error.\n");

    usingKernelEnqueueOption->_sVersion = "eq";
    usingKernelEnqueueOption->_lVersion = "devEnqueue";
    usingKernelEnqueueOption->_description = "Enable device-side path (default). Set this value to 0 for host-side iterative approach";
    usingKernelEnqueueOption->_type = CA_ARG_INT;
    usingKernelEnqueueOption->_value = &devEnqueue;
    sampleArgs->AddOption(usingKernelEnqueueOption);

	delete usingKernelEnqueueOption;

    return SDK_SUCCESS;
}

int ExtractPrimes::setup()
{
	//length should bigger then 0 and  power of 2
    if(length<=0)
    {
        length = 512;
    }

    if(isPowerOf2(length))
    {
        length = roundToPowerOf2(length);
    }


	printf("***********************************************************************************\n");
	if(devEnqueue)
		printf("ExtractPrimes Using OpenCL 2.0 Device-Side Enqueue Feature \n");
	else
		printf("ExtractPrimes Using OpenCL 2.0 Without Device-Side Enqueue Feature \n");
	printf("***********************************************************************************\n");

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	cl_int   status;
	status = setupCL();
    if (status != SDK_SUCCESS)
    {
        return status;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    if(setupExtractPrimes() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Move data host to device
    cl_event writeEvt;
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 inputBuffer,
                 CL_FALSE,
                 0,
                 sizeof(cl_int) * length,
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


int ExtractPrimes::run()
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

int ExtractPrimes::verifyResults()
{
  int status = SDK_SUCCESS;
  if(sampleArgs->verify)
    {
      // Read the device output buffer
      cl_int *ptrOutBuff;
      cl_int outLength;
      int status = mapBuffer(outPrimesBuffer, 
			     ptrOutBuff,  
			     length * sizeof(cl_int),
			     CL_MAP_READ);
      CHECK_ERROR(status, SDK_SUCCESS, 
		  "Failed to map device buffer.(resultBuf)");

      // reference implementation
      extractPrimesCPUReference(verificationOutput, input, length, &outLength);
      
      // compare the results and see if they match
       if(compare(ptrOutBuff, verificationOutput, outLength))
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
			printArray<cl_int>("Output : ", ptrOutBuff, outLength, 1);
        }
      
    }
  return status;
}

void ExtractPrimes::printStats()
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

int ExtractPrimes::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status = 0;
	if(devEnqueue)
	{
		status = clReleaseKernel(extract_primes_kernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");
		
		status = clReleaseCommandQueue(devcommandQueue);
		CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(devcommandQueue)");
		
	}
	else
	{
		status = clReleaseKernel(set_primes_kernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

		status = clReleaseKernel(get_primes_kernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

		status = clReleaseKernel(group_kernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");

		status = clReleaseKernel(global_kernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(program)");
	}

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer)");
	
	status = clReleaseMemObject(primesBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(primesBuffer)");

	status = clReleaseMemObject(outPrimesBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outPrimesBuffer)");
	
    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBuffer)");
	
	status = clReleaseMemObject(errorBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(errorBuffer)");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");

    return SDK_SUCCESS;
}

template<typename T>
int ExtractPrimes::mapBuffer(cl_mem deviceBuffer, T* &hostPointer,
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
ExtractPrimes::unmapBuffer(cl_mem deviceBuffer, void* hostPointer)
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
    ExtractPrimes clExtractPrimes;
	
    // Initialize
    if(clExtractPrimes.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clExtractPrimes.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clExtractPrimes.sampleArgs->isDumpBinaryEnabled())
    {
        //GenBinaryImage
        return clExtractPrimes.genBinaryImage();
    }

    // Setup
	status = clExtractPrimes.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    // Run
    if(clExtractPrimes.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // VerifyResults
    if(clExtractPrimes.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Cleanup
    if (clExtractPrimes.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clExtractPrimes.printStats();
    return SDK_SUCCESS;
}
