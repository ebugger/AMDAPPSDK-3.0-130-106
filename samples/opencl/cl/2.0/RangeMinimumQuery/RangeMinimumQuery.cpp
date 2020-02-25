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

#include "RangeMinimumQuery.hpp"


int
RangeMinimumQuery::getMinAlignment(unsigned int* minAlignment)
{
	cl_int status = 0; 
	cl_uint result = 0;
	size_t deviceList;
	status = clGetContextInfo (context,
                      CL_CONTEXT_DEVICES,
                      0,
                      NULL,
                      &deviceList);
	CHECK_OPENCL_ERROR(status, "clGetContextInfo failed.(deviceSize).");

	for (int i = 0; i < (int)(deviceList/sizeof(cl_device_id)); i++) 
    {    
        cl_uint alignment = 0;     
        status = clGetDeviceInfo(devices[i],
		              CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                      sizeof(cl_uint),
                      (void*)&alignment,
                      NULL);
        CHECK_OPENCL_ERROR(status, "clGetContextInfo failed.(deviceSize).");
        alignment >>= 3;    // convert bits to bytes
        result = (alignment > result) ? alignment : result;
    }

	*minAlignment = result;
	return SDK_SUCCESS;
}

int
RangeMinimumQuery::setupRangeMinimumQuery()
{
	unsigned int minAlignment; 
	int status = getMinAlignment(&minAlignment);
	CHECK_ERROR(status, SDK_SUCCESS, "getMinAlignment() failed.");

	// Allocate the SVM memory buffer for input array
	inputSVMBuffer = (cl_uint*)clSVMAlloc(context,
										  CL_MEM_READ_ONLY,
										  numInputs*sizeof(int),
										  minAlignment);
	if (inputSVMBuffer == NULL)
	{
		std::cout << "clSVMAlloc failed. (inputSVMBuffer)" << std::endl;
		return SDK_FAILURE;
	}

	// updating SVM Buffer for input array
	cl_event svmUnmapEvent;;
	status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_WRITE,
							inputSVMBuffer, (numInputs * sizeof(int)),
							0, NULL, NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (inputSVMBuffer) failed!");

	for (cl_uint i = 0; i < numInputs; ++i)
	{
		inputSVMBuffer[i] = (cl_uint)(rand());
	}
	
	status = clEnqueueSVMUnmap(commandQueue, inputSVMBuffer, 0, NULL, &svmUnmapEvent);
	CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (inputSVMBuffer) failed!");
	status = waitForEventAndRelease(&svmUnmapEvent);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(svmUnmapEvent) Failed");

    // Allocate the memory buffer for output array
    #if defined WIN32
    output = (cl_uint *)_aligned_malloc(numWorkGroups*sizeof(int), minAlignment);
    #elif defined __linux__
    output = (cl_uint *)memalign(minAlignment, numWorkGroups*sizeof(int));
    #endif
    CHECK_ALLOCATION(output, "Allocation failed(output)");

	// Set the output data
    for(cl_uint i = 0; i < numWorkGroups; ++i)
    {
        output[i] = 0;
    }

    return SDK_SUCCESS;
}

int
RangeMinimumQuery::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("RangeMinimumQuery_Kernels.cl");
    binaryData.flagsStr = std::string("");
    if( sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string( sampleArgs->flags.c_str());
    }
    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    return status;
}

int
RangeMinimumQuery::setupCL(void)
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
            std::cout << "GPU not found. Falling back to CPU" << std::endl;
            dType = CL_DEVICE_TYPE_CPU;
        }
    }

	// Get platform
    cl_platform_id platform = NULL;
    int retValue = getPlatform(platform, sampleArgs->platformId,
                               sampleArgs->isPlatformEnabled());
    CHECK_ERROR(retValue, SDK_SUCCESS, "getPlatform() failed.");

    // Display available devices.
    retValue = displayDevices(platform, dType);
    CHECK_ERROR(retValue, SDK_SUCCESS, "displayDevices() failed.");

	// If we could find our platform, use it. Otherwise use just available platform.
    cl_context_properties cps[3] = {CL_CONTEXT_PLATFORM, (cl_context_properties)platform, 0};
    context = clCreateContextFromType(cps,
                                      dType,
                                      NULL,
                                      NULL,
                                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateContextFromType failed.");

    // getting device on which to run the sample
    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed ");

    //Set device info of given cl_device_id
    retValue = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
    CHECK_ERROR(retValue, SDK_SUCCESS, "SDKDeviceInfo::setDeviceInfo() failed" );

	// Check of OPENCL_C_VERSION if device version is 2.0 or higher
	isOpenCL2_XSupported = deviceInfo.checkOpenCL2_XCompatibility();
    if (!isOpenCL2_XSupported)
	{
		OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
	}

	// Set parameters
    if(deviceInfo.maxComputeUnits > 1)
    {
        numComputeUnits = deviceInfo.maxComputeUnits;
    }
    if((localSize > deviceInfo.maxWorkGroupSize) || (localSize < 2))
    {
        localSize = (cl_uint)deviceInfo.maxWorkGroupSize;
    }
    if(!isPowerOf2(localSize))
    {
        localSize = roundToPowerOf2(localSize);
    }

	// make sure Start Index is within the array boundry
	if (startIndex > (numInputs-1))
	{
		std::cout << "\nInvalid Index : Number Of Input should be greater than startIndex <" << startIndex << ">.\nPlease see --help option\n" << std::endl;
		return SDK_EXPECTED_FAILURE;
	}

	if(isLastIndex)
		endIndex = endIndex;
	else 
		endIndex = numInputs -1;

	// make sure last Index is within the array boundry
	if (endIndex > (numInputs-1))
	{
		std::cout << "\nInvalid Index : Last Index should be less than Total Number of Input " << numInputs -1 << ".\nPlease see --help option\n" << std::endl;
		return SDK_EXPECTED_FAILURE;
	}

	// make sure last Index is less than the first Index
	if (endIndex <= startIndex)
	{
		std::cout << "\nInvalid Index : Last Index should be greater than the Start Index .\nPlease see --help option\n" << std::endl;
		return SDK_EXPECTED_FAILURE;
	}

	// setting number of work-groups
	sizeRMQ = (endIndex - startIndex) + 1;
	numWorkGroups = (sizeRMQ % localSize) ? (sizeRMQ / localSize + 1) : sizeRMQ / localSize;

	// As per the OpenCL 2.0, NDRange is flexible and need not be a multiple of workgroup dimension 
	globalWorkItems = sizeRMQ ; 
	localWorkItems = localSize;

    if(!sampleArgs->quiet)
    {
        std::cout << "\n Sample parameters :" << std::endl;
        std::cout << "\t Local Size : " << localSize << std::endl;
        std::cout << "\t Number of WGs Per Kernel : " << numWGsPerKernel << std::endl;
        std::cout << "\t Available Compute Units : " << numComputeUnits << std::endl;
        std::cout << "\t RMQ Range startIndex " << startIndex << " and LastIndex " << endIndex  << std::endl;
        std::cout << "\t RMQ Range Size  : " << sizeRMQ << std::endl;
		std::cout << "\t Total Number of array elemments is : " << numInputs << std::endl << std::endl;
    }

	cl_queue_properties *props = NULL;
	commandQueue = clCreateCommandQueueWithProperties(context, devices[sampleArgs->deviceId], props, &status);
	CHECK_OPENCL_ERROR(status, "clCreateCommandQueueWithProperties failed(commandQueue)");
	
	//Setup application data for input array
	if (setupRangeMinimumQuery() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	// create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("RangeMinimumQuery_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
    buildData.flagsStr = std::string("-cl-std=CL2.0");  /*check it*/
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
	
	// Create reduceRMQ kernel object
    reduceRMQKernel = clCreateKernel(program, "reduceRMQ", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(reduceRMQ).");

	// Create device buffer for output array
    outputBuffer = clCreateBuffer(context, CL_MEM_USE_HOST_PTR, numWorkGroups*sizeof(int), (void *)output, &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(outputBuffer)");

    return SDK_SUCCESS;
}

int
RangeMinimumQuery::initialize()
{
    // Call base class Initialize to get default configuration
    CHECK_ERROR(sampleArgs->initialize(), SDK_SUCCESS,
                "OpenCL Resources Initialization failed");

	Option* num_inputs = new Option;
    CHECK_ALLOCATION(num_inputs, "Allocation failed(num_inputs)");
    num_inputs->_sVersion = "x";
    num_inputs->_lVersion = "numInputs";
    num_inputs->_description = "Array Length";
    num_inputs->_type = CA_ARG_INT;
    num_inputs->_value = &numInputs;
    sampleArgs->AddOption(num_inputs);
    delete num_inputs;

    Option* start_index = new Option;
    CHECK_ALLOCATION(start_index, "Allocation failed(start_index)");
    start_index->_sVersion = "m";
    start_index->_lVersion = "startIndex";
    start_index->_description = "Start Index for RMQ <Default value for m = 10>";
    start_index->_type = CA_ARG_INT;
    start_index->_value = &startIndex;
    sampleArgs->AddOption(start_index);
    delete start_index;

    Option* end_index = new Option;
    CHECK_ALLOCATION(end_index, "Allocation failed(end_index)");
    end_index->_sVersion = "n";
    end_index->_lVersion = "endIndex";
    end_index->_description = "Last Index for RMQ <default value for n = 999999>";
    end_index->_type = CA_ARG_INT;
    end_index->_value = &endIndex;
    sampleArgs->AddOption(end_index);
    delete end_index;

    Option* numLoops = new Option;
    CHECK_ALLOCATION(numLoops, "Allocation failed(numLoops)");
    numLoops->_sVersion = "i";
    numLoops->_lVersion = "iterations";
    numLoops->_description = "Number of iterations of kernel execution";
    numLoops->_type = CA_ARG_INT;
    numLoops->_value = &iterations;
    sampleArgs->AddOption(numLoops);
    delete numLoops;
    Option* local_size = new Option;
    CHECK_ALLOCATION(local_size, "Memory allocation error(local_size).\n");
    local_size->_sVersion = "l";
    local_size->_lVersion = "localsize";
    local_size->_description =
        "Number of Work items per Work Group(should be 2 ^ N)";
    local_size->_type = CA_ARG_INT;
    local_size->_value = &localSize;
    sampleArgs->AddOption(local_size);
    delete local_size;

    return SDK_SUCCESS;
}

int
RangeMinimumQuery::setup()
{
	int retStatus;
    if (iterations == 0)
    {
        iterations = 1;
    }

	if(endIndex != NUM_OF_INPUTS-1)
		isLastIndex = true;

    timer = sampleTimer->createTimer();

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	retStatus = setupCL();
    if(retStatus != SDK_SUCCESS)
    {
        return retStatus;
    }
	
    sampleTimer-> stopTimer(timer);
    setupTime = sampleTimer-> readTimer(timer) * 1000;

    return SDK_SUCCESS;
}

void
RangeMinimumQuery::cpuRefImplementation()
{
	refOut = 0;
	RMQIndex = startIndex;
	minElement = inputSVMBuffer[startIndex];
	cl_uint range = startIndex+sizeRMQ;

	for (cl_uint i = startIndex + 1; i < range; ++i)
	{
		if (minElement > inputSVMBuffer[i])
	    {
			minElement = inputSVMBuffer[i];
			RMQIndex = i;
	    }
	}
	refOut = minElement;
}

int RangeMinimumQuery::verifyResults()
{
    if(sampleArgs->verify)
    {

		std::cout << " Kernel Verfication - ";
        // Calculate the reference output
        cpuRefImplementation();

        // Compare the results and see if they match
		if(actOut != refOut)
		{
            std::cout << "Failed! with actOut " << actOut << "and refOut" << refOut << "\n" << std::endl;
			//std::cout << "Failed! \n" << std::endl;
			return SDK_FAILURE;
	    }
	    else
	    {
			std::cout << "Passed\n\nRMQ for input arr[" << startIndex << "," << endIndex << "] is " << RMQIndex << " \nminimum value within this range is " << minElement << "\n" << std::endl;
	        return SDK_SUCCESS;
	    }
    }
    return SDK_SUCCESS;
}

int RangeMinimumQuery::runKernels(void)
{
	int status = SDK_SUCCESS;
    
	// Set arguments for reduceRMQ kernel
	status = clSetKernelArgSVMPointer(reduceRMQKernel, 0, (inputSVMBuffer + startIndex));
    CHECK_OPENCL_ERROR(status, "runKernels::reduceRMQKernel failed for clSetKernelArgSVMPointer(inputSVMBuffer).");
	status = clSetKernelArg(reduceRMQKernel, 1, sizeof(cl_mem), (void*)&outputBuffer);
	CHECK_OPENCL_ERROR(status, "runKernels::reduceRMQKernel failed for clSetKernelArg(outputBuffer).");
	status = clSetKernelArg(reduceRMQKernel, 2, localSize*sizeof(cl_uint), NULL);
	CHECK_OPENCL_ERROR(status, "runKernels::reduceRMQKernel failed for clSetKernelArg(local memory).");

	// launch kernel
	status = clEnqueueNDRangeKernel(commandQueue,
									reduceRMQKernel,
									1,
									NULL,
									&globalWorkItems,
									&localWorkItems,
									0,
									NULL,
									NULL); 
	CHECK_OPENCL_ERROR(status, "runKernels::clEnqueueNDRangeKernel(reduceRMQKernel) failed.");

	// read data from output buffer
	void *ptr;
    cl_map_flags mapFlags = CL_MAP_READ;
    cl_event event;
	ptr = (void *)clEnqueueMapBuffer(commandQueue, 
                                     outputBuffer,
                                     CL_TRUE,
                                     mapFlags,
                                     0,
                                     numWorkGroups*sizeof(int),
                                     0, NULL, NULL,
                                     &status);
    CHECK_OPENCL_ERROR(status, "clEnqueueMapBuffer(outputBuffer) Failed.");

    status = clEnqueueUnmapMemObject(commandQueue,
                                     outputBuffer,
                                     (void *)ptr,
                                     0, NULL, &event);
    CHECK_OPENCL_ERROR(status, "clEnqueueUnmapMemObject(outputBuffer) Failed.");

	status = waitForEventAndRelease(&event);
	CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(event) Failed");

	// Calculate actual output
	actOut = 0;
	RMQIndex = startIndex;
	minElement = output[0];
	for(cl_uint i = 1; i < numWorkGroups; ++i)
	{
		if (minElement > output[i])
		{
			minElement = output[i];
			RMQIndex = i;
		}
	}
	actOut = minElement;

	if(!(sampleArgs->verify))
    {
		std::cout << "\n\nRMQ for input arr[" << startIndex << "," << endIndex << "] is " << RMQIndex << " \nAnd Minimum value within this range is " << minElement << "\n" << std::endl;
	}

    return SDK_SUCCESS;
}

int
RangeMinimumQuery::run()
{
    int status = SDK_SUCCESS;

	// Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
        if(runKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

    std::cout << "\n Executing kernel for " << iterations
                      << " iterations" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < iterations; i++)
    {
		if (runKernels() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
    }

    sampleTimer->stopTimer(timer);
    seqTime = sampleTimer-> readTimer(timer) * 1000;
    seqTime = seqTime / iterations;
    seqTime = seqTime/1000;

    return SDK_SUCCESS;
}

void
RangeMinimumQuery::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[5] = {"RMQ Size", "RMQ Value", "RMQ Index", "Setup Time(ms)", "Average Kernel Time(s)"};
        std::string stats[5];
		stats[0]  = toString(sizeRMQ, std::dec);
		stats[1] = toString(minElement, std::dec);
		stats[2]  = toString(RMQIndex, std::dec);
        stats[3]  = toString(setupTime, std::dec);
        stats[4]  = toString(seqTime, std::dec);
        printStatistics(strArray, stats, 5);
    }
}

int
RangeMinimumQuery::cleanup()
{
    
    cl_int status;
	/* freeing SVM input Buffer */
    clSVMFree(context, inputSVMBuffer);
   
	// Releases OpenCL resources (Context, Memory etc.)
    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject(outputBuffer) failed.");
    status = clReleaseKernel(reduceRMQKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel(reduceRMQKernel) failed.");
    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram(program) failed.");
    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue(commandQueue) failed.");
    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext(context) failed.");

    // freeing output memory
    if(output)
    {
       #if defined WIN32
       _aligned_free(output);
       #elif defined __linux__
       free(output);
       #endif
    }

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    int status = 0;
	RangeMinimumQuery clRangeMinimumQuery;
	if (clRangeMinimumQuery.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if (clRangeMinimumQuery.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if (clRangeMinimumQuery.sampleArgs->isDumpBinaryEnabled())
    {
		return clRangeMinimumQuery.genBinaryImage();
    }
	status = clRangeMinimumQuery.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }
	if (clRangeMinimumQuery.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if (clRangeMinimumQuery.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if (clRangeMinimumQuery.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	clRangeMinimumQuery.printStats();
    return SDK_SUCCESS;
}
