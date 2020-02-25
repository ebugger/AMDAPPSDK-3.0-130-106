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

#include "DeviceEnqueueBFS.hpp"

int
DeviceEnqueueBFS::underflow()
{
	if( (front == NULL) && (rear == NULL) )
	{
		//std::cout << "\n Queue is Empty \n";
		return 0;
	}
	else
		return 1;
}

void 
DeviceEnqueueBFS::enqueue(cl_uint val)
{
	newNode = (queue *)malloc(sizeof(queue));
	newNode->info = val;
	if((front == NULL) && (rear == NULL))
	{
		front = newNode;
		rear = newNode;
		newNode->next = NULL;
	}
	else
	{
		rear->next = newNode;
		newNode->next = NULL;
		rear = newNode;
	}
}

cl_uint
DeviceEnqueueBFS::dequeue()
{
	cl_uint val = 0;
	val = front->info;

	if(front == rear)
	{	
		front = NULL;
		rear = NULL;
	}
	else
	{
		front = front->next;
	}

	return val;
}

int
DeviceEnqueueBFS::matToCSR()
{
	int k = 0, startRowIndex = 0, nZRCountPerRow = 0, nZRCountPreviousRow = 0;
	colIndex[nZRCountPerRow++] = rootNode;  // store root node as first element
	nZRCountPreviousRow = 1;

	for(int i = 0; i < numNodes; i++)
	{
		// initializing the colIndex Array
		for(int j = 0; j < numNodes; j++)
		{
			if(adjMat[i][j] != 0)
			{
				colIndex[nZRCountPerRow++] = j; // store column index of non-zero element
			}
		}

		// initializing the rowPtr Array
		rowPtr[startRowIndex++] = nZRCountPreviousRow;
		nZRCountPreviousRow = nZRCountPerRow;
	}
	rowPtr[startRowIndex] = nZRCountPerRow;
	
	return SDK_SUCCESS;
}

int
DeviceEnqueueBFS::setupBFS()
{
	int retValue = 0;

	if(numNodes == 0)
    {
        numNodes = NUM_OF_NODES;
    }

	// initializing the adjacency matrix
	adjMat = (cl_uint **)malloc(numNodes*sizeof(int *));
	CHECK_ALLOCATION(adjMat, "Allocation failed(adjMat)");
	for(int i = 0; i < numNodes; i++)
	{
		adjMat[i] = (cl_uint *)calloc(numNodes, sizeof(int));
		CHECK_ALLOCATION(adjMat, "Allocation failed(adjMat[i])");
	}
	for(int i = 0; i < numNodes; i++)
	{
		for(int j = i; j < numNodes; j++)
		{
			if(i != j)
				adjMat[i][j] = adjMat[j][i] = rand() & 1 ; // 0 or 1
		}
	}

	// allocate memory for rowPtr array
	rowPtr = (cl_uint *)calloc((numNodes+1), sizeof(int));
	CHECK_ALLOCATION(rowPtr, "Allocation failed(rowPtr)");

	// counting non-zero elements
	for(int i = 0; i < numNodes; i++)
	{
		for(int j = 0; j < numNodes; j++)
		{
			if(adjMat[i][j] != 0)
			{
				nZRCount++;
			}
		}
	}

	// allocate memory for colIndex array
	colIndex = (cl_uint *)malloc((nZRCount+1)*sizeof(int)); // adding one extra element to include root node
	CHECK_ALLOCATION(colIndex, "Allocation failed(colIndex)");

	// Converting adjacency matrix into CSR format
	retValue = matToCSR();
	CHECK_ERROR(retValue, SDK_SUCCESS, "matToCSR() failed.");

	// allocate memory for refDist array
	refDist = (cl_uint *)malloc(numNodes*sizeof(int));
	CHECK_ALLOCATION(refDist, "Allocation failed(refDist)");

	// initialize the dist array with infinity
	for(int m = 0; m < numNodes; m++)
		refDist[m] = INIFINITY;

    return SDK_SUCCESS;
}

int
DeviceEnqueueBFS::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("SimpleDeviceEnqueue_Kernels.cl");
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
DeviceEnqueueBFS::setupCL(void)
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

	// Check localSize and it should be power of 2.
    if((localSize > deviceInfo.maxWorkGroupSize) || (localSize < 2))
    {
        localSize = (cl_uint)deviceInfo.maxWorkGroupSize;
    }
    if(!isPowerOf2(localSize))
    {
        localSize = roundToPowerOf2(localSize);
    }	

	globalWorkItems = 1;  // start with one node (root node)
	
	if(!sampleArgs->quiet)
    {
        std::cout << "\n Sample parameters :" << std::endl;
        std::cout << "\t Local Size : " << localSize << std::endl;
        std::cout << "\t Number of WGs Per Kernel : " << numWGsPerKernel << std::endl;
        std::cout << "\t Available Compute Units : " << numComputeUnits << std::endl;
		std::cout << "\t Dimension of Adjacency Matrix : " << numNodes << std::endl;
        std::cout << "\t Total Number of edges : " << nZRCount << std::endl <<
                  std::endl;
    }

	// Check 2.x compatibility
	bool check2_x = deviceInfo.checkOpenCL2_XCompatibility();
	if(!check2_x)
	{
		OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
	}

	// Create host command queue
	cl_queue_properties *props = NULL;
	commandQueue = clCreateCommandQueueWithProperties(context, devices[sampleArgs->deviceId],
                                        props, &status);
	CHECK_OPENCL_ERROR(status, "clCreateCommandQueueWithProperties failed(commandQueue)");
	
	// Create device command queue
    {
        // The block is to move the declaration of prop closer to its use
        cl_queue_properties prop[] = {
									CL_QUEUE_PROPERTIES, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE | CL_QUEUE_ON_DEVICE|CL_QUEUE_ON_DEVICE_DEFAULT,
									CL_QUEUE_SIZE, deviceInfo.maxQueueSize, 0 };
        
        deviceCommandQueue = clCreateCommandQueueWithProperties(
                           context,
                           devices[sampleArgs->deviceId],
                           prop,
                           &status);
        CHECK_OPENCL_ERROR( status, "clCreateCommandQueueWithProperties failed(deviceCommandQueue).");

    }

	// create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("DeviceEnqueueBFS_Kernels.cl");
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
	writePipeKernel = clCreateKernel(program, "pipeWrite", &status);
	CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(writePipeKernel).");
	deviceEnqueueKernel = clCreateKernel(program, "deviceEnqueueBFSKernel", &status);
	CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(deviceEnqueueKernel).");

	// Create device buffers for input array: rowPtr
	inputRowPtrBuffer = clCreateBuffer(context,  CL_MEM_COPY_HOST_PTR, (numNodes+1)*sizeof(int), (void *)rowPtr, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(inputRowPtrBuffer)");

	// Create device buffers for input array: colIndex
	inputColIndexBuffer = clCreateBuffer(context,  CL_MEM_COPY_HOST_PTR, (nZRCount+1)*sizeof(int), (void *)colIndex, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(inputColIndexBuffer)");

	// Create SVM buffer for output array: outputBFS
    outputDistSVMBuffer = (cl_uint *)clSVMAlloc(context, CL_MEM_READ_WRITE, (numNodes)*sizeof(int), 0);
	if(outputDistSVMBuffer == NULL)
	{
		std::cout << "clSVMAlloc failed. (outputDistSVMBuffer)" << std::endl;
		return SDK_FAILURE;
	}

	// create pipe memory object for reading nodes
	vertexQueueReadPipe = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, sizeof(int), nZRCount+1, NULL, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(vertexQueueReadPipe)");

	// create pipe memory object for writing neighbour nodes of visited nodes
	edgeQueueWritePipe = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, sizeof(int), nZRCount+1, NULL, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(edgeQueueWritePipe)");

    return SDK_SUCCESS;
}

int
DeviceEnqueueBFS::initialize()
{
    // Call base class Initialize to get default configuration
    CHECK_ERROR(sampleArgs->initialize(), SDK_SUCCESS,
                "OpenCL Resources Initialization failed");

    Option* numLoops = new Option;
    CHECK_ALLOCATION(numLoops, "Allocation failed(numLoops)");
    numLoops->_sVersion = "i";
    numLoops->_lVersion = "iterations";
    numLoops->_description = "Number of iterations of kernel execution";
    numLoops->_type = CA_ARG_INT;
    numLoops->_value = &iterations;
    sampleArgs->AddOption(numLoops);
    delete numLoops;

	Option* numOfNodes = new Option;
    CHECK_ALLOCATION(numOfNodes, "Allocation failed(numOfNodes)");
    numOfNodes->_sVersion = "n";
    numOfNodes->_lVersion = "Matrix Size";
    numOfNodes->_description = "Dimension of an Adjacency Matrix";
    numOfNodes->_type = CA_ARG_INT;
    numOfNodes->_value = &numNodes;
    sampleArgs->AddOption(numOfNodes);
    delete numOfNodes;

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
DeviceEnqueueBFS::setup()
{
	int status = 0;
	if (iterations == 0)
    {
        iterations = 1;
    }

	//Setup application data for input array
    if(setupBFS() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	Timer = sampleTimer->createTimer();

    sampleTimer->resetTimer(Timer);
    sampleTimer->startTimer(Timer);

	status = setupCL();
    if(status != SDK_SUCCESS)
    {
        return status;
    }
	sampleTimer-> stopTimer(Timer);
    setupTime = sampleTimer-> readTimer(Timer) * 1000;

	return SDK_SUCCESS;
}

void
DeviceEnqueueBFS::cpuRefImplementation()
{
	rootNode = colIndex[0];
	cl_uint i, j, temp;

	refDist[rootNode] = 0;
	enqueue(rootNode);

	while(underflow() == 1)
	{
		i = dequeue();
		offset = rowPtr[i];
		temp = (rowPtr[i+1] - offset);
		for(cl_uint itr = 0; itr < temp; itr++)
		{
			j = colIndex[offset+itr];
			if(refDist[j] == INIFINITY) /* visit the node, if it is not visited */
			{
				refDist[j] = refDist[i] + 1;
				enqueue(j);
			}
		}
	}
}

int 
DeviceEnqueueBFS::verifyResults()
{
    if(sampleArgs->verify)
    {
		std::cout << " Kernel Verfication - ";

        // Calculate the reference output
        cpuRefImplementation();

		// Calculate actual output
		cl_event svmUnmapEvent;
		int status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_READ, outputDistSVMBuffer, (numNodes)*sizeof(int), 0, NULL, NULL);
		CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clEnqueueSVMMap(outputDistSVMBuffer).");

		for(int i = 0; i < numNodes; ++i)
		{	
			// Compare the results and see if they match
			if(refDist[i] != outputDistSVMBuffer[i])
			{
				std::cout << "Failed!\n" << std::endl;
				return SDK_FAILURE;
			}
		}
		std::cout << "Passed\n" << std::endl;

		status = clEnqueueSVMUnmap(commandQueue, outputDistSVMBuffer, 0, NULL, &svmUnmapEvent);
		CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clEnqueueSVMUnmap(outputDistSVMBuffer).");
		status = waitForEventAndRelease(&svmUnmapEvent);
		CHECK_ERROR(status, SDK_SUCCESS, "waitForEventAndRelease(svmUnmapEvent) Failed");
    }
    return SDK_SUCCESS;
}

int 
DeviceEnqueueBFS::InitializePipe(void)
{
	int status = SDK_SUCCESS;

	//clean-up the pipe memory objects for next iteration
	status = clReleaseMemObject(vertexQueueReadPipe);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject(vertexQueueReadPipe) failed.");
	status = clReleaseMemObject(edgeQueueWritePipe);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject(edgeQueueWritePipe) failed.");

	// create read-pipe memory object for next iteration
	vertexQueueReadPipe = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, sizeof(int), nZRCount+1, NULL, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(vertexQueueReadPipe)");

	// create write-pipe memory object for next iteration
	edgeQueueWritePipe = clCreatePipe(context, CL_MEM_HOST_NO_ACCESS, sizeof(int), nZRCount+1, NULL, &status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(edgeQueueWritePipe)");

	return SDK_SUCCESS;
}

int 
DeviceEnqueueBFS::runWritePipeKernel(void)
{
	int status = SDK_SUCCESS;

	// Set arguments for WritePipe kernel 
	status = clSetKernelArg(writePipeKernel, 0, sizeof(cl_mem), (void*)&vertexQueueReadPipe);
	CHECK_OPENCL_ERROR(status, "runKernels::writePipeKernel failed for clSetKernelArg(vertexQueueReadPipe).");

	// Run write-pipe kernel, to initialize the write-pipe memory object with only root node (it should be called before BFS kernel)
	status = clEnqueueNDRangeKernel(
					commandQueue,
					writePipeKernel,
					1,
					NULL,
					&globalWorkItems,
					NULL,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status, "runKernels::clEnqueueNDRangeKernel(writePipeKernel) failed.");
	// Wait till kernel finishes
	status = clFinish(commandQueue);
	CHECK_OPENCL_ERROR(status, "runKernels::clFinish failed.");

	return SDK_SUCCESS;
}

int 
DeviceEnqueueBFS::runDeviceEnqueueBFSKernels(void)
{
	int status = SDK_SUCCESS;

	// Updating SVM buffer for output dist array
	cl_event svmUnmapEvent;
	status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_WRITE, outputDistSVMBuffer, (numNodes)*sizeof(int), 0, NULL, NULL);
	CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clEnqueueSVMMap(outputDistSVMBuffer).");

	for(int i = 0; i < numNodes; i++)
	{
		outputDistSVMBuffer[i] = INIFINITY;
	}

	status = clEnqueueSVMUnmap(commandQueue, outputDistSVMBuffer, 0, NULL, &svmUnmapEvent);
	CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clEnqueueSVMUnmap(outputDistSVMBuffer).");
	status = waitForEventAndRelease(&svmUnmapEvent);
	CHECK_ERROR(status, SDK_SUCCESS, "waitForEventAndRelease(svmUnmapEvent) Failed");
	
    // Set arguments for device-enqueue BFS kernel
    status = clSetKernelArg(deviceEnqueueKernel, 0, sizeof(cl_mem), (void*)&inputRowPtrBuffer);
    CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArg(inputRowPtrBuffer).");

	status = clSetKernelArg(deviceEnqueueKernel, 1, sizeof(cl_mem), (void*)&inputColIndexBuffer);
    CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArg(inputColIndexBuffer).");

    status = clSetKernelArgSVMPointer(deviceEnqueueKernel, 2, (void*)outputDistSVMBuffer);
    CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArgSVMPointer(outputDistSVMBuffer).");

	status = clSetKernelArg(deviceEnqueueKernel, 3, sizeof(cl_mem), (void*)&vertexQueueReadPipe);
	CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArg(vertexQueueReadPipe).");

	status = clSetKernelArg(deviceEnqueueKernel, 4, sizeof(cl_mem), (void*)&edgeQueueWritePipe);
	CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArg(edgeQueueWritePipe).");

	status = clSetKernelArg(deviceEnqueueKernel, 5, sizeof(cl_int), &rootNode);
    CHECK_OPENCL_ERROR(status, "runKernels::deviceEnqueueKernel failed for clSetKernelArg(rootNode).");

    // Run device-enqueue BFS kernel
	status = clEnqueueNDRangeKernel(
					commandQueue,
					deviceEnqueueKernel,
					1,
					NULL,
					&globalWorkItems,
					NULL,
					0,
					NULL,
					NULL); 
	CHECK_OPENCL_ERROR(status, "runKernels::clEnqueueNDRangeKernel(deviceEnqueueKernel) failed.");

	// Flush all queues
    status = clFlush(commandQueue);
	CHECK_OPENCL_ERROR(status, "runKernels::clFlush failed.");
    // Wait till all kernels to finish
    status = clFinish(commandQueue);
	CHECK_OPENCL_ERROR(status, "runKernels::clFinish failed.");

    return SDK_SUCCESS;
}

int
DeviceEnqueueBFS::run()
{
	// Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
        // run write-pipe kernel at very beginning to initialize the write-pipe object with root node.
        if(runWritePipeKernel() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

		// Arguments are set and execution call is enqueued on command buffer
        if(runDeviceEnqueueBFSKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

		// Clean-up and Initialize the pipe memory objects for next iteration
        if(InitializePipe() != SDK_SUCCESS)
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
		if(runWritePipeKernel() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}

		if (runDeviceEnqueueBFSKernels() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}

		// Clean-up and Initialize the pipe memory objects for next iteration
        if(InitializePipe() != SDK_SUCCESS)
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
DeviceEnqueueBFS::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[3] = {"Number of Edges to be traversed", "Setup Time(ms)", "Average Kernel Time(s)"};
        std::string stats[3];
		stats[0]  = toString(nZRCount, std::dec);
        stats[1]  = toString(setupTime, std::dec);
        stats[2]  = toString(seqTime, std::dec);
        printStatistics(strArray, stats, 3);
    }
}

int
DeviceEnqueueBFS::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;
    status = clReleaseMemObject(inputRowPtrBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject(inputRowPtrBuffer) failed.");
	status = clReleaseMemObject(inputColIndexBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject(inputColIndexBuffer) failed.");
	status = clReleaseMemObject(vertexQueueReadPipe);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject(vertexQueueReadPipe) failed.");
	status = clReleaseMemObject(edgeQueueWritePipe);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject(edgeQueueWritePipe) failed.");

	status = clReleaseKernel(writePipeKernel);
	CHECK_OPENCL_ERROR(status, "clReleaseKernel(writePipeKernel) failed.");
    status = clReleaseKernel(deviceEnqueueKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel(deviceEnqueueKernel) failed.");
    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram(program) failed.");
	status = clReleaseCommandQueue(commandQueue);
	CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue(commandQueue) failed.");
	status = clReleaseCommandQueue(deviceCommandQueue);
	CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue(deviceCommandQueue) failed.");

	/* Freeing SVM Output Dist Buffer */
	clSVMFree(context, outputDistSVMBuffer);
    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext(context) failed.");

	// freeing input memory
	free(*adjMat);
	free(adjMat);
	free(rowPtr);
	free(colIndex);

    // freeing output memory
	free(refDist);

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    int status = 0;
    DeviceEnqueueBFS clDeviceEnqueueBFS;
	
    if(clDeviceEnqueueBFS.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	
    if(clDeviceEnqueueBFS.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    if(clDeviceEnqueueBFS.sampleArgs->isDumpBinaryEnabled())
    {
        return clDeviceEnqueueBFS.genBinaryImage();
    }
	
    status = clDeviceEnqueueBFS.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }
	
    if(clDeviceEnqueueBFS.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	
	if (clDeviceEnqueueBFS.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    if(clDeviceEnqueueBFS.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    clDeviceEnqueueBFS.printStats();
    return SDK_SUCCESS;
}
