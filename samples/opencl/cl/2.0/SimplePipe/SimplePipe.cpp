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

#include "SimplePipe.hpp"


int
SimplePipe::getMinAlignment(int* minAlignment)
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
SimplePipe::setupSimplePipe()
{
	
    // Make sure numPackets is multiple of localSize
	numPackets = (numPackets / localSize);
    numPackets = numPackets ? numPackets * localSize : localSize;

	// Set numPackets Per Pipe
	if(!isMultiPipe)
	{
		numPacketsPerPipe = numPackets;
	}
	else
	{
		numPacketsPerPipe = numPackets/NUM_PIPES;
	}

	// Get the size of largest OpenCL built-in data type
	int minAlignment; 
	int retValue = getMinAlignment(&minAlignment);
	CHECK_ERROR(retValue, SDK_SUCCESS, "getMinAlignment() failed.");
	
	// Allocate the memory for input array
        #if defined WIN32
        input = (cl_uint *)_aligned_malloc(packetSize*numPackets, minAlignment);
        #elif defined __linux__
        input = (cl_uint *)memalign(minAlignment, packetSize*numPackets);
        #endif
        CHECK_ALLOCATION(input, "Allocation failed(input)");

        // Allocate the memory for output array
        #if defined WIN32
        output = (cl_uint *)_aligned_malloc(packetSize*numPackets, minAlignment);
        #elif defined __linux__
        output = (cl_uint *)memalign(minAlignment, packetSize*numPackets);
        #endif
        CHECK_ALLOCATION(output, "Allocation failed(output)");

    // Set the input data
    for(cl_uint i = 0; i < numPackets; ++i)
    {
        input[i] = (cl_uint)(rand() % 5);
    }

    return SDK_SUCCESS;
}

int
SimplePipe::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("SimplePipe_Kernels.cl");
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
SimplePipe::setupCL(void)
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

	// Set parameters
    if(deviceInfo.maxComputeUnits > 1)
    {
        numComputeUnits = deviceInfo.maxComputeUnits;
    }
	if(numPackets == 0)
    {
        numPackets = NUM_OF_PACKETS;
    }
	if(packetSize == 0)
    {
        packetSize = PACKET_SIZE;
    }
	
	// Check isMultiPipe flag
	if(isMultiPipe)
	{
		numPipes = NUM_PIPES;
	}

	// Check localSize and it should be power of 2.
    if((localSize > deviceInfo.maxWorkGroupSize) || (localSize < 2))
    {
        localSize = (cl_uint)deviceInfo.maxWorkGroupSize;
    }
    if(!isPowerOf2(localSize))
    {
        localSize = roundToPowerOf2(localSize);
    }

	//Setup application data for input array
    if(setupSimplePipe() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	localWorkItems = localSize;
    globalWorkItems = numPackets;

	if(!sampleArgs->quiet)
    {
        std::cout << "\n Sample parameters :" << std::endl;
        std::cout << "\t Local Size : " << localSize << std::endl;
        std::cout << "\t Number of WGs Per Kernel : " << numWGsPerKernel << std::endl;
        std::cout << "\t Available Compute Units : " << numComputeUnits << std::endl;
		std::cout << "\t isMultiPipe: " << (!(!isMultiPipe)) << std::endl;
		std::cout << "\t Number of Pipes : " << numPipes << std::endl;
		std::cout << "\t Kernel Type : " << kernelType << std::endl;
		std::cout << "\t Packet Size (in Bytes): " << packetSize << std::endl;
		std::cout << "\t Total Number of Packets: " << numPackets << std::endl;
        std::cout << "\t Number of packets Per Pipe : " << numPacketsPerPipe << std::endl <<
                  std::endl;
    }
    
    // Check of OPENCL_C_VERSION if device version is 2.0	
    int majorRev, minorRev;
    if (sscanf(deviceInfo.deviceVersion, "OpenCL %d.%d", &majorRev, &minorRev) == 2) 
    {
      if (majorRev < 2) {
	    OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
      }
    }

	cl_queue_properties *props = NULL;
	commandQueue = clCreateCommandQueueWithProperties(context, devices[sampleArgs->deviceId],
                                        props, &status);
	CHECK_OPENCL_ERROR(status, "clCreateCommandQueueWithProperties failed(commandQueue)");
	
	// create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("SimplePipe_Kernels.cl");
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

	// Initialize the kernel objects
	if(!isMultiPipe)  /* Single Pipe Use cases */
	{
		if(kernelType == 0){
			std::cout << "\n\n Running Kernel WITH WORK-ITEM RESERVE built-in pipe function (reserve_read/write_pipe)...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "pipeWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeWrite).");

			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "pipeRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeRead).");
		}
		else if(kernelType == 1){  /* Use Pipe Work-group built-in functions */
			std::cout << "\n\n Running Kernel WITH WORK-GROUP RESERVE built-in pipe function (work_group_reserve_read/write_pipe)...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "pipeWorkGroupWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeWorkGroupWrite).");
	
			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "pipeWorkGroupRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeWorkGroupRead).");
		}
		else if(kernelType == 2){  /* Use Pipe Convenience built-in functions */
			std::cout << "\n\n Running Kernel WITHOUT RESERVE built-in pipe function ...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "pipeConvenienceWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeConvenienceWrite).");

			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "pipeConvenienceRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(pipeConvenienceRead).");
		}
		else {
			std::cout << "\n Invalid Kernel Type : kernelType should be between 0-2" << std::endl;	
			return SDK_FAILURE;
		}
	}
	else  /* Multiple Pipe Use cases */
	{
		if(kernelType == 0){
			std::cout << "\n\n Running Kernel WITH WORK-ITEM RESERVE built-in pipe function (reserve_read/write_pipe)...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "multiplePipeWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeWrite).");

			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "multiplePipeRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeRead).");
		}
		else if(kernelType == 1){
			std::cout << "\n\n Running Kernel WITH WORK-GROUP RESERVE built-in pipe function (work_group_reserve_read/write_pipe)...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "multiplePipeWorkGroupWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeWorkGroupWrite).");

			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "multiplePipeWorkGroupRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeWorkGroupRead).");
		}
		else if(kernelType == 2){
			std::cout << "\n\n Running Kernel WITHOUT RESERVE built-in pipe function ...";
			// Create Producer kernel object
			writePipeKernel = clCreateKernel(program, "multiplePipeConvenienceWrite", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeConvenienceWrite).");

			// Create Consumer kernel object 
			readPipeKernel = clCreateKernel(program, "multiplePipeConvenienceRead", &status);
			CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(multiplePipeConvenienceRead).");
		}
		else {
			std::cout << "\n Invalid Kernel Type : kernelType should be between 0-2" << std::endl;	
			return SDK_FAILURE;
		}
	}

	// Create device buffers and intialize it using input array
	inputBuffer = clCreateBuffer(context,  CL_MEM_COPY_HOST_PTR, packetSize * numPackets, (void *)input, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(inputBuffer)");

	// Create device buffers for output array
	outputBuffer = clCreateBuffer(context, CL_MEM_USE_HOST_PTR, packetSize * numPackets, (void *)output, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(outputBuffer)");

	// Create Pipe memory object
	if(!isMultiPipe)  /* Single Pipe Use cases */
	{
		pipe[0] = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, packetSize, numPackets, NULL, &status);
		CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(pipe[0])");
	}
	else  /* Multiple Pipe Use cases */
	{  
		for(cl_uint i = 0; i < NUM_PIPES; ++i)
		{
			pipe[i] = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, packetSize, numPackets/NUM_PIPES, NULL, &status);
			CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(pipe[])");
		}
	}

    return SDK_SUCCESS;
}

int
SimplePipe::initialize()
{
    // Call base class Initialize to get default configuration
    CHECK_ERROR(sampleArgs->initialize(), SDK_SUCCESS,
                "OpenCL Resources Initialization failed");

    Option* num_packets = new Option;
    CHECK_ALLOCATION(num_packets, "Allocation failed(num_packets)");
    num_packets->_sVersion = "x";
    num_packets->_lVersion = "numPackets";
    num_packets->_description = "Total Number of Packets to communicate between two kernels using Pipes";
    num_packets->_type = CA_ARG_INT;
	num_packets->_value = &numPackets;
    sampleArgs->AddOption(num_packets);
    delete num_packets;

	Option* packet_size = new Option;
    CHECK_ALLOCATION(packet_size, "Allocation failed(packet_size)");
    packet_size->_sVersion = "y";
    packet_size->_lVersion = "packetSize";
    packet_size->_description = "Packet Size(Bytes)";
    packet_size->_type = CA_ARG_INT;
	packet_size->_value = &packetSize;
    sampleArgs->AddOption(packet_size);
    delete packet_size;

    Option* numLoops = new Option;
    CHECK_ALLOCATION(numLoops, "Allocation failed(numLoops)");
    numLoops->_sVersion = "i";
    numLoops->_lVersion = "iterations";
    numLoops->_description = "Number of iterations of kernel execution";
    numLoops->_type = CA_ARG_INT;
    numLoops->_value = &iterations;
    sampleArgs->AddOption(numLoops);
    delete numLoops;

	Option* multi_pipe = new Option;
    CHECK_ALLOCATION(multi_pipe, "Memory allocation error(multi_pipe).\n");
    multi_pipe->_sVersion = "mp";
    multi_pipe->_lVersion = "multiPipe";
    multi_pipe->_description = "Flag indicating singlePipe or multiPipe use cases: \n\t\t\t\t\t\t  0 SinglePipe [Default]\n\t\t\t\t\t\t  1 MultiPipe";
    multi_pipe->_type = CA_ARG_INT;
    multi_pipe->_value = &isMultiPipe;
    sampleArgs->AddOption(multi_pipe);
    delete multi_pipe;

	Option* kernel_type = new Option;
    CHECK_ALLOCATION(kernel_type, "Memory allocation error(kernel_type).\n");
    kernel_type->_sVersion = "type";
    kernel_type->_lVersion = "kernelType";
    kernel_type->_description = "Type of Built-in Pipe Functions: \n\t\t\t\t\t\t  0 reserve_read/write_pipe (i.e. work-item based) [Default]\n\t\t\t\t\t\t  1 work_group_reserve_read/write_pipe\n\t\t\t\t\t\t  2 Convenience [without using reserve Built-in Pipe Functions]";
    kernel_type->_type = CA_ARG_INT;
	kernel_type->_value = &kernelType;
    sampleArgs->AddOption(kernel_type);
    delete kernel_type;

	Option* num_workgroups = new Option;
    CHECK_ALLOCATION(num_workgroups, "Memory allocation error(num_workgroups).\n");
    num_workgroups->_sVersion = "w";
    num_workgroups->_lVersion = "workgroups";
    num_workgroups->_description = "Number of WorkGroups per Kernel execution";
    num_workgroups->_type = CA_ARG_INT;
    num_workgroups->_value = &numWGsPerKernel;
    sampleArgs->AddOption(num_workgroups);
    delete num_workgroups;

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
SimplePipe::setup()
{
     int retValue;
     
	if (iterations == 0)
    {
        iterations = 1;
    }
	Timer = sampleTimer->createTimer();

    sampleTimer->resetTimer(Timer);
    sampleTimer->startTimer(Timer);

    retValue = setupCL();
    if(retValue != SDK_SUCCESS)
    {
        return retValue;
    }
	sampleTimer-> stopTimer(Timer);
    setupTime = sampleTimer-> readTimer(Timer) * 1000;

	return SDK_SUCCESS;
}

void
SimplePipe::cpuRefImplementation()
{
	refOut = 0;
	for(cl_uint i = 0; i < numPackets; ++i)
		refOut += input[i];
}

int SimplePipe::verifyResults()
{
    if(sampleArgs->verify)
    {
		int status;

		std::cout << " Kernel Verfication - ";
        // Calculate the reference output
        cpuRefImplementation();

		// Calculate actual output
		actOut = 0;
		void *ptr;
		cl_map_flags mapFlags = CL_MAP_READ;
		cl_event event;

		ptr = (void *)clEnqueueMapBuffer(commandQueue,
										 outputBuffer, 
										 CL_TRUE,
										 mapFlags,
										 0,
										 packetSize*numPackets,
										 0, NULL, NULL,
										 &status);
		CHECK_OPENCL_ERROR(status, "clEnqueueMapBuffer(outputBuffer) Failed.");

		memcpy(ptr, output, packetSize*numPackets);

		status = clEnqueueUnmapMemObject(commandQueue,
										 outputBuffer,
										 (void *)ptr,
										 0, NULL, &event);
		CHECK_OPENCL_ERROR(status, "clEnqueueUnmapMemObject(outputBuffer) Failed.");

		status = clWaitForEvents(1, &event);
		CHECK_OPENCL_ERROR(status, "clWaitForEvents(event) Failed.");

		for(cl_uint i = 0; i < numPackets; ++i)
			actOut += output[i];

        // Compare the results and see if they match
		if(actOut != refOut)
        {
            std::cout << "Failed!\n" << std::endl;
			return SDK_FAILURE;
        }
        else
        {
            std::cout << "Passed\n" << std::endl;
            return SDK_SUCCESS;
        }
    }
    return SDK_SUCCESS;
}

int SimplePipe::runKernels(void)
{
	int status = SDK_SUCCESS;

    // Set arguments for producer kernel
    status = clSetKernelArg(writePipeKernel, 0, sizeof(cl_mem), (void*)&inputBuffer);
    CHECK_OPENCL_ERROR(status, "runKernels::writePipeKernel failed for clSetKernelArg(inputBuffer).");
	if(!isMultiPipe)  /* Single Pipe Use cases */
	{
		status = clSetKernelArg(writePipeKernel, 1, sizeof(cl_mem), (void*)&pipe[0]);
		CHECK_OPENCL_ERROR(status, "runKernels::writePipeKernel failed for clSetKernelArg(pipe[0]).");
	}
	else  /*multiple pipe use cases*/
	{
		for(cl_uint i = 0; i < NUM_PIPES; ++i)
		{
			status |= clSetKernelArg(writePipeKernel, (i+1), sizeof(cl_mem), (void*)&pipe[i]);
		}
		CHECK_OPENCL_ERROR(status, "runKernels::writePipeKernel failed for clSetKernelArg(pipe[]).");

		status = clSetKernelArg(writePipeKernel, 5, sizeof(int), (void*)&numPipes);
		CHECK_OPENCL_ERROR(status, "runKernels::writePipeKernel failed for clSetKernelArg(numPipes).");
	}
	

	// Set arguments for consumer kernel
	if(!isMultiPipe)  /* Single Pipe Use cases */
	{
		status = clSetKernelArg(readPipeKernel, 0, sizeof(cl_mem), (void*)&pipe[0]);
		CHECK_OPENCL_ERROR(status, "runKernels::readPipeKernel failed for clSetKernelArg(pipe[0]).");

		status = clSetKernelArg(readPipeKernel, 1, sizeof(cl_mem), (void*)&outputBuffer);
		CHECK_OPENCL_ERROR(status, "runKernels::readPipeKernel failed for clSetKernelArg(outputBuffer).");
	}
	else  /*multiple pipe use cases*/
	{
		for(cl_uint i = 0; i < NUM_PIPES; ++i)
		{
			status |= clSetKernelArg(readPipeKernel, i, sizeof(cl_mem), (void*)&pipe[i]);
		}
		CHECK_OPENCL_ERROR(status, "runKernels::readPipeKernel failed for clSetKernelArg(pipe[]).");
	
		status = clSetKernelArg(readPipeKernel, 4, sizeof(cl_mem), (void*)&outputBuffer);
		CHECK_OPENCL_ERROR(status, "runKernels::readPipeKernel failed for clSetKernelArg(outputBuffer).");

		status = clSetKernelArg(readPipeKernel, 5, sizeof(int), (void*)&numPipes);
		CHECK_OPENCL_ERROR(status, "runKernels::readPipeKernel failed for clSetKernelArg(numPipes).");
	}
	

    // Clear command queue
	status = clFinish(commandQueue); 
	CHECK_OPENCL_ERROR(status, "runKernels::clFinish() failed.");
    
    
    // Run producer kernel
	status = clEnqueueNDRangeKernel(
					commandQueue,
					writePipeKernel,
					1,
					NULL,
					&globalWorkItems,
					&localWorkItems,
					0,
					NULL,
					NULL); 
	CHECK_OPENCL_ERROR(status, "runKernels::clEnqueueNDRangeKernel(writePipeKernel) failed.");

	// Run consumer kernel
	status = clEnqueueNDRangeKernel(
					commandQueue,
					readPipeKernel,
					1,
					NULL,
					&globalWorkItems,
					&localWorkItems,
					0, 
					NULL,
					NULL); 
	CHECK_OPENCL_ERROR(status, "runKernels::clEnqueueNDRangeKernel(readPipeKernel) failed.");
 
	// Flush all queues
    status = clFlush(commandQueue);
	CHECK_OPENCL_ERROR(status, "runKernels::clFlush failed.");
    // Wait till all kernels to finish
    status = clFinish(commandQueue);
	CHECK_OPENCL_ERROR(status, "runKernels::clFinish failed.");

	std::cout << "\n";

    return SDK_SUCCESS;
}

int
SimplePipe::run()
{
	// Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
        // Arguments are set and execution call is enqueued on command buffer
        if(runKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

	std::cout << "\n Executing kernel for " << iterations
              << " iterations" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

	sampleTimer->resetTimer(Timer);
    sampleTimer->startTimer(Timer);

	for(int i = 0; i < iterations; i++)
    {
		if (runKernels() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
	}

	sampleTimer->stopTimer(Timer);
    seqTime = sampleTimer-> readTimer(Timer) * 1000;
    seqTime = seqTime / iterations;
	seqTime = seqTime/1000;

    return SDK_SUCCESS;
}

void
SimplePipe::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] = {"Number Of Packets", "Packet Size(Bytes)", "Setup Time(ms)", "Average Kernel Time(s)"};
        std::string stats[4];
		stats[0]  = toString(numPackets, std::dec);
		stats[1]  = toString(packetSize, std::dec);
        stats[2]  = toString(setupTime, std::dec);
        stats[3]  = toString(seqTime, std::dec);
        printStatistics(strArray, stats, 4);
    }
}

int
SimplePipe::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;
    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject(inputBuffer) failed.");
    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject(outputBuffer) failed.");

	if(!isMultiPipe)   /* Single Pipe Use cases */
	{
		status = clReleaseMemObject(pipe[0]);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject(pipe[0]) failed.");
	}
	else    /* Mutiple Pipe Use cases */
	{
		for(cl_uint i = 0; i < NUM_PIPES; ++i)
		{
			status |= clReleaseMemObject(pipe[i]);
		}
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject(pipe[]) failed.");
	}
	

    status = clReleaseKernel(writePipeKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel(writePipeKernel) failed.");
    status = clReleaseKernel(readPipeKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel(readPipeKernel) failed.");
    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram(program) failed.");
	status = clReleaseCommandQueue(commandQueue);
	CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue(commandQueue) failed.");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext(context) failed.");

	// freeing input memory
        if(input)
        {
          #if defined WIN32
          _aligned_free(input);
          #elif defined __linux__
          free(input);
          #endif
        }

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
    SimplePipe clSimplePipe;
    if(clSimplePipe.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    if(clSimplePipe.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    if(clSimplePipe.sampleArgs->isDumpBinaryEnabled())
    {
        return clSimplePipe.genBinaryImage();
    }
    status = clSimplePipe.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }
    if(clSimplePipe.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if (clSimplePipe.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    if(clSimplePipe.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    clSimplePipe.printStats();
    return SDK_SUCCESS;
}
