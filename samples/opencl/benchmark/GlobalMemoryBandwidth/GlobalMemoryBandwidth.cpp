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


#include "GlobalMemoryBandwidth.hpp"


//Separator
std::string sep = "-----------------------------------------";

template<typename T>
int GlobalMemoryBandwidth::mapBuffer(cl_mem deviceBuffer, T* &hostPointer,
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

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::unmapBuffer(cl_mem deviceBuffer, void* hostPointer)
{
    cl_int status;
    status = clEnqueueUnmapMemObject(commandQueue,
                                     deviceBuffer,
                                     hostPointer,
                                     0,
                                     NULL,
                                     NULL);
    CHECK_OPENCL_ERROR(status, "clEnqueueUnmapMemObject failed");
	
    return SDK_SUCCESS;
}


int
GlobalMemoryBandwidth::setupGlobalMemoryBandwidth()
{
    /**
     * Allocate memory required for global buffer
     * This includes both single and linear(cached and uncached) reads
     */
    cl_uint sizeElement = vectorSize * sizeof(cl_float);
    cl_uint readLength = length + (NUM_READS * 1024 / sizeElement) + EXTRA_ELEMENTS;

    /*
     * Map cl_mem inputBuffer to host for writing
     * Note the usage of CL_MAP_WRITE_INVALIDATE_REGION flag
     * This flag indicates the runtime that whole buffer is mapped for writing and
     * there is no need of device->host transfer. Hence map call will be faster
     */
    int status = mapBuffer( inputBuffer, input,
                            (readLength * sizeElement),
                            CL_MAP_WRITE_INVALIDATE_REGION );
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(inputBuffer)");

    // random initialisation of input
    fillRandom<cl_float>( input,
                          readLength * vectorSize,
                          1,
                          0,
                          (cl_float)(readLength-1));

    /* Unmaps cl_mem inputBuffer from host
     * host->device transfer happens if device exists in different address-space
     */
    status = unmapBuffer(inputBuffer, input);
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to unmap device buffer.(inputBuffer)");

    cl_uint lengthExtra = (length * NUM_READS) + NUM_READS;

    // Map cl_mem inputBufferExtra to host for writing
    status = mapBuffer( inputBufferExtra, inputExtra,
                        (lengthExtra * sizeElement),
                        CL_MAP_WRITE_INVALIDATE_REGION );
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(inputBufferExtra)");

    fillRandom<cl_float>( inputExtra,
                          lengthExtra * vectorSize,
                          1,
                          0,
                          10);

    // Unmaps cl_mem inputBufferExtra from host
    status = unmapBuffer(inputBufferExtra, inputExtra);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(inputBufferExtra)");

#ifdef CL_VERSION_2_0
	if (svmSupport)
	{
		//Data for SVM buffers
		status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_WRITE, 
								 inputSVMBuffer, (readLength * sizeElement),
								 0, NULL, NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (inputSVMBuffer) failed!");

		 // random initialisation of input
		fillRandom<cl_float>( inputSVMBuffer,
							  readLength * vectorSize,
							  1,
							  0,
							  (cl_float)(readLength-1));

		status = clEnqueueSVMUnmap(commandQueue, inputSVMBuffer, 0, NULL, NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (inputSVMBuffer) failed!");

		status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_WRITE, 
								 inputSVMBufferExtra, (lengthExtra * sizeElement),
								 0, NULL, NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (inputSVMBuffer) failed!");

		 // random initialisation of input
		fillRandom<cl_float>( inputSVMBufferExtra,
							  lengthExtra * vectorSize,
							  1,
							  0,
							  10);

		status = clEnqueueSVMUnmap(commandQueue, inputSVMBufferExtra, 0, NULL, NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (inputSVMBufferExtra) failed!");
	}
#endif

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("GlobalMemoryBandwidth_Kernels.cl");

    // Always using vector-width of 1 to dump kernels
    if(vectorSize != 0)
    {
        std::cout <<
                  "Ignoring specified vector-width. Always using vector-width of 1 to dump kernels"
                  << std::endl;
    }
    vectorSize = 1;

    // Pass vectorSize as DATATYPE to kernel
    char buildOption[128];

    sprintf(buildOption, "-D DATATYPE=float -D DATATYPE2=float4 -D OFFSET=%d ",
            OFFSET);

    binaryData.flagsStr = std::string(buildOption);
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    CHECK_ERROR(status, SDK_SUCCESS, "OpenCL Generate Binary Image Failed");
    return status;
}

int
GlobalMemoryBandwidth::createSVMBuffers(cl_uint bufferSize, cl_uint inputExtraBufferSize)
{
#ifdef CL_VERSION_2_0
	// Create input buffer
    inputSVMBuffer = (cl_float*) clSVMAlloc(context,
                                 CL_MEM_READ_ONLY,
                                 bufferSize,
                                 0);
    if (inputSVMBuffer == NULL)
	{
		std::cout << "clSVMAlloc failed. (inputSVMBuffer)" << std::endl;
		return SDK_FAILURE; 
	}

	inputSVMBufferExtra = (cl_float*) clSVMAlloc(context,
                                      CL_MEM_READ_ONLY,
                                      inputExtraBufferSize,
                                      0);
    if (inputSVMBufferExtra == NULL)
	{
		std::cout << "clSVMAlloc failed. (inputSVMBufferExtra)" << std::endl;
		return SDK_FAILURE; 
	}

	outputSVMBufferWriteLinear = (cl_float*) clSVMAlloc(context,
                              CL_MEM_WRITE_ONLY,
                              bufferSize,
                              0);
    if (outputSVMBufferWriteLinear == NULL)
	{
		std::cout << "clSVMAlloc failed. (outputSVMBufferWriteLinear)" << std::endl;
		return SDK_FAILURE; 
	}
#endif
	return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::setupCL(void)
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

    /*
     * Have a look at the available platforms and pick either
     * the AMD one if available or a reasonable default.
     */
    cl_platform_id platform = NULL;
    int retValue = getPlatform(platform, sampleArgs->platformId,
                               sampleArgs->isPlatformEnabled());
    CHECK_ERROR(retValue, SDK_SUCCESS, "getPlatform() failed");

    // Display available devices.
    retValue = displayDevices(platform, dType);
    CHECK_ERROR(retValue, SDK_SUCCESS, "displayDevices() failed");

    /*
     * If we could find our platform, use it. Otherwise use just available platform.
     */
    cl_context_properties cps[3] =
    {
        CL_CONTEXT_PLATFORM,
        (cl_context_properties)platform,
        0
    };

    context = clCreateContextFromType(cps,
                                      dType,
                                      NULL,
                                      NULL,
                                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateContextFromType failed.");

    // getting device on which to run the sample
    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");

    //Set device info of given cl_device_id
    retValue = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
    CHECK_ERROR(retValue, SDK_SUCCESS, "SDKDeviceInfo::setDeviceInfo() failed");

    std::string deviceStr(deviceInfo.deviceVersion);
    size_t vStart = deviceStr.find(" ", 0);
    size_t vEnd = deviceStr.find(" ", vStart + 1);
    std::string vStrVal = deviceStr.substr(vStart + 1, vEnd - vStart - 1);

#ifdef CL_VERSION_1_1
    if(vStrVal.compare("1.0") > 0)
    {
        char openclVersion[1024];
        status = clGetDeviceInfo(devices[sampleArgs->deviceId],
                                 CL_DEVICE_OPENCL_C_VERSION,
                                 sizeof(openclVersion),
                                 openclVersion,
                                 0);
        CHECK_OPENCL_ERROR(status, "clGetDeviceInfo failed.");

        std::string tempStr(openclVersion);
        size_t dotPos = tempStr.find_first_of(".");
        size_t spacePos = tempStr.find_last_of(" ");
        tempStr = tempStr.substr(dotPos + 1, spacePos - dotPos);
        int minorVersion = atoi(tempStr.c_str());
        // OpenCL 1.1 has inbuilt support for vec3 data types
        if(minorVersion < 1 && vec3 == true)
        {
            OPENCL_EXPECTED_ERROR("Device doesn't support built-in 3 component vectors!");
        }
    }
    else
    {
        // OpenCL 1.1 has inbuilt support for vec3 data types
        if(vec3 == true)
        {
            OPENCL_EXPECTED_ERROR("Device doesn't support built-in 3 component vectors!");
        }
    }
#else
    // OpenCL 1.1 has inbuilt support for vec3 data types
    if(vec3 == true)
    {
        OPENCL_EXPECTED_ERROR("Device doesn't support built-in 3 component vectors!");
    }
#endif
	
	//If OpenCL 2.x available, confirm SVM support
	if (deviceInfo.checkOpenCL2_XCompatibility())
	{
		svmSupport = deviceInfo.detectSVM();

		if (svmSupport)
		{
			std::cout << "Device supports SVM" << std::endl;
		}
	}

    {
        
#ifdef CL_VERSION_2_0
		// The block is to move the declaration of prop closer to its use
  	cl_queue_properties prop[] = { CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0 };

		commandQueue = clCreateCommandQueueWithProperties(context,
                                            devices[sampleArgs->deviceId],
                                            prop,
                                            &status);
        CHECK_OPENCL_ERROR(status, "clCreateCommandQueueWithProperties failed.");
#else
		// The block is to move the declaration of prop closer to its use
		/* Note: Using deprecated clCreateCommandQueue as CL_QUEUE_PROFILING_ENABLE flag not currently working 
		***with clCreateCommandQueueWithProperties*/
        cl_command_queue_properties prop = 0;
        prop |= CL_QUEUE_PROFILING_ENABLE;

        commandQueue = clCreateCommandQueue(context,
                                            devices[sampleArgs->deviceId],
                                            prop,
                                            &status);
        CHECK_OPENCL_ERROR(status, "clCreateCommandQueue failed.");
#endif
    }

    if(sampleArgs->isLoadBinaryEnabled())
    {
        // Always assuming kernel was dumped for vector-width 1
        if(vectorSize != 0)
        {
            std::cout <<
                      "Ignoring specified vector-width. Assuming kernel was dumped for vector-width 1"
                      << std::endl;
        }
        vectorSize = 1;
    }
    else
    {
        // If vector-size is not specified in the command-line, choose the preferred size for the device
        if(vectorSize == 0)
        {
            vectorSize = deviceInfo.preferredFloatVecWidth;
        }
        else if(vectorSize == 3)
        {
            //Make vectorSize as 4 if -v option is 3.
            //This memory alignment is required as per OpenCL for type3 vectors
            vec3 = true;
            vectorSize = 4;
        }
        else if((1 != vectorSize) && (2 != vectorSize) && (4 != vectorSize) &&
                (8 != vectorSize) && (16 != vectorSize))
        {
            std::cout << "The vectorsize can only be one of 1,2,3(4),4,8,16!" << std::endl;
            return SDK_FAILURE;
        }
    }

	// inputBufferExtra does the highest single allocation of all
    // Check if this is allocatable, else reduce 'length'
    cl_ulong maxAllocation = sizeof(cl_float) * vectorSize * ((length * NUM_READS) + NUM_READS);
    while(maxAllocation > deviceInfo.maxMemAllocSize)
    {
        length /= 2;
        maxAllocation = sizeof(cl_float) * vectorSize * ((length * NUM_READS) + NUM_READS);
    }

	globalThreads = length;

    cl_uint sizeElement = vectorSize * sizeof(cl_float);
    cl_uint readLength = length + (NUM_READS * 1024 / sizeElement) + EXTRA_ELEMENTS;
    readRange = readLength;
    cl_uint size = readLength * vectorSize * sizeof(cl_float);
	cl_uint extraBufferSize = sizeof(cl_float) * vectorSize *  ((length * NUM_READS) + NUM_READS);

    // Create input buffer
    inputBuffer = clCreateBuffer(context,
                                 CL_MEM_READ_ONLY,
                                 size,
                                 0,
                                 &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer)");

    outputBufferReadSingle = clCreateBuffer(context,
                                            CL_MEM_WRITE_ONLY,
                                            sizeof(cl_float) * vectorSize * length,
                                            0,
                                            &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBufferReadSingle)");

    outputBufferReadLinear = clCreateBuffer(context,
                                            CL_MEM_WRITE_ONLY,
                                            sizeof(cl_float) * vectorSize * length,
                                            0,
                                            &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBufferReadLinear)");

    outputBufferReadLU = clCreateBuffer(context,
                                        CL_MEM_WRITE_ONLY,
                                        sizeof(cl_float) * vectorSize * length,
                                        0,
                                        &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBufferReadLU)");

    outputBufferWriteLinear = clCreateBuffer(context,
                              CL_MEM_WRITE_ONLY,
                              size,
                              0,
                              &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBufferWriteLinear)");

    inputBufferExtra = clCreateBuffer(context,
                                      CL_MEM_READ_ONLY,
                                      extraBufferSize,
                                      0,
                                      &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateBuffer failed due to inavailability of memory on this platform. (inputBufferExtra)");

    outputBufferReadRandom = clCreateBuffer(context,
                                            CL_MEM_WRITE_ONLY,
                                            sizeof(cl_float) * vectorSize * length,
                                            0,
                                            &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBufferReadCombine)");

    outputBufferReadunCombine = clCreateBuffer(context,
                                CL_MEM_WRITE_ONLY,
                                sizeof(cl_float) * vectorSize * length,
                                0,
                                &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateBuffer failed. (outputBufferReadunCombine)");

    constValue = clCreateBuffer(context,
                                CL_MEM_READ_ONLY,
                                vectorSize * sizeof(cl_float),
                                0,
                                &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(constValue)");

	// create a CL program using the kernel source
    char buildOption[512];
    if(vectorSize == 1)
    {
        sprintf(buildOption, "-D DATATYPE=float -D DATATYPE2=float4 -D OFFSET=%d ",
                OFFSET);
    }
    else
    {
        sprintf(buildOption, "-D DATATYPE=float%d -D DATATYPE2=float%d -D OFFSET=%d ",
                (vec3 == true) ? 3 : vectorSize,(vec3 == true) ? 3 : vectorSize, OFFSET);
    }

	if (svmSupport)
		status = createSVMBuffers(size, extraBufferSize);
	CHECK_ERROR(status, SDK_SUCCESS, "createSVMBuffers() failed");

	if (svmSupport)
	{
		//Here IDXTYPE is set to siz_t which uses int64 for 64-bit apps and int for 32bit apps
		//This is because, in OpenCL 2.0, addressing happens in 64-bits for 64-bit apps. 
		//Hence the datatype of address offsets also has to be of 64-bits for better performance
		//OpenCL compiler can better optimize address calculation because static offsets can be 
		//pre-computed into 64-bit offsets
		strcat(buildOption, "-cl-std=CL2.0 -D IDXTYPE=size_t ");
	}
	else
	{
		strcat(buildOption, "-D IDXTYPE=uint ");
	}
	
    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("GlobalMemoryBandwidth_Kernels.cl");
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

    // Global memory bandwidth from read-single access
    kernel[0] = clCreateKernel(program, "read_single", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(read_single)");

    // Global memory  bandwidth from read-linear access
    kernel[1] = clCreateKernel(program, "read_linear", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(read_linear)");

    // Global memory  bandwidth from read-linear access
    kernel[2] = clCreateKernel(program, "read_linear_uncached", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(read_linear_uncached)");

    // Global memory  bandwidth from write-linear access
    kernel[3] = clCreateKernel(program, "write_linear", &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateKernel failed.(GlobalBandwidth_write_linear)");


    if(vectorSize != 1)
    {
        kernel[4] = clCreateKernel(program, "read_random", &status);
        CHECK_OPENCL_ERROR(status,
                           "clCreateKernel failed.(GlobalBandwidth_read_random)");
    }
    else
    {
        kernel[4] = clCreateKernel(program, "read_random1", &status);
        CHECK_OPENCL_ERROR(status,
                           "clCreateKernel failed.(GlobalBandwidth_read_random1)");

    }

    kernel[5] = clCreateKernel(program, "read_uncoalescing", &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateKernel failed.(GlobalBandwidth_read_random1)");

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::bandwidth(cl_kernel &kernel,
                                 cl_mem outputBuffer,
								 cl_float *outputSVMBuffer,
                                 double *timeTaken,
                                 double *gbps,
								 bool useSVM = false)
{
    cl_int status;

    // Check group size against kernelWorkGroupSize
    status = clGetKernelWorkGroupInfo(kernel,
                                      devices[sampleArgs->deviceId],
                                      CL_KERNEL_WORK_GROUP_SIZE,
                                      sizeof(size_t),
                                      &kernelWorkGroupSize,
                                      0);
    CHECK_OPENCL_ERROR(status, "clGetKernelWorkGroupInfo failed.");

    if(localThreads > kernelWorkGroupSize)
    {
        localThreads = kernelWorkGroupSize;
    }

    //Set appropriate arguments to the kernel
    int argIndex = 0;
    if(!writeFlag && !extraFlag)
    {
#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			status = clSetKernelArgSVMPointer(kernel, 
											  argIndex++, 
											  inputSVMBuffer);
			CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(inputSVMBuffer)");
		}
		else
#endif
		{
			status = clSetKernelArg(kernel,
									argIndex++,
									sizeof(cl_mem),
									(void *)&inputBuffer);
			CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBuffer)");
		}
    }
    else if(extraFlag)
    {
#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			status = clSetKernelArgSVMPointer(kernel, 
											  argIndex++, 
											  inputSVMBufferExtra);
			CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(inputSVMBufferExtra)");
		}
		else
#endif
		{
			status = clSetKernelArg(kernel,
									argIndex++,
									sizeof(cl_mem),
									(void *)&inputBufferExtra);
			CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(inputBufferExtra)");
		}
    }
    else
    {
        // Pass a single constant value to kernel of type - float<vectorSize>
        cl_float *temp;

        // Map cl_mem constValue to host for writing
        status = mapBuffer( constValue, temp,
                            (vectorSize * sizeof(cl_float)),
                            CL_MAP_WRITE_INVALIDATE_REGION );
        CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(constValue)");

        memset(temp, 0, vectorSize * sizeof(cl_float));

        /* Unmaps cl_mem constValue from host
         * host->device transfer happens if device exists in different address-space
         */
        status = unmapBuffer(constValue, temp);
        CHECK_ERROR(status, SDK_SUCCESS, "Failed to unmap device buffer.(constValue)");

        status = clSetKernelArg(kernel,
                                argIndex++,
                                sizeof(cl_mem),
                                (void *)&constValue);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(constValue)");
    }

#ifdef CL_VERSION_2_0
		if (useSVM && outputSVMBuffer != NULL)
		{
			status = clSetKernelArgSVMPointer(kernel, 
											  argIndex++, 
											  outputSVMBuffer);
			CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer failed.(outputSVMBuffer)");
		}
		else
#endif
		{
			status = clSetKernelArg(kernel,
                            argIndex++,
                            sizeof(cl_mem),
                            (void *)&outputBuffer);
			CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");
		}

    double sec = 0;
    int iter = iterations;

    // Run the kernel for a number of iterations
    for(int i = 0; i < iter; i++)
    {
        // Enqueue a kernel run call
        cl_event ndrEvt;
        status = clEnqueueNDRangeKernel(commandQueue,
                                        kernel,
                                        1,
                                        NULL,
                                        &globalThreads,
                                        &localThreads,
                                        0,
                                        NULL,
                                        &ndrEvt);
        CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");

        // wait for the kernel call to finish execution
        status = clWaitForEvents(1, &ndrEvt);
        CHECK_OPENCL_ERROR(status, "clWaitForEvents failed.");

        // Calculate performance
        cl_ulong startTime;
        cl_ulong endTime;

        // Get kernel profiling info
        status = clGetEventProfilingInfo(ndrEvt,
                                         CL_PROFILING_COMMAND_START,
                                         sizeof(cl_ulong),
                                         &startTime,
                                         0);
        CHECK_OPENCL_ERROR(status, "clGetEventProfilingInfo failed.(startTime)");

        status = clGetEventProfilingInfo(ndrEvt,
                                         CL_PROFILING_COMMAND_END,
                                         sizeof(cl_ulong),
                                         &endTime,
                                         0);
        CHECK_OPENCL_ERROR(status, "clGetEventProfilingInfo failed.(endTime)");

        // Cumulate time for each iteration
        sec += 1e-9 * (endTime - startTime);

        status = clReleaseEvent(ndrEvt);
        CHECK_OPENCL_ERROR(status, "clGetEventProfilingInfo failed.(endTime)");

		status = clFinish(commandQueue);
		CHECK_OPENCL_ERROR(status, "clFinish failed");
    }

    // Copy bytes
    int bytesPerThread = 0;
    if(vec3 == true)
    {
        bytesPerThread = NUM_READS * 3 * sizeof(cl_float);
    }
    else
    {
        bytesPerThread = NUM_READS * vectorSize * sizeof(cl_float);
    }
    double bytes = (double)(iter * bytesPerThread);
    double perf = (bytes / sec) * 1e-9;
    perf *= globalThreads;

    *gbps = perf;
    *timeTaken = sec / iter;

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::runCLKernels(void)
{
    std::cout << "Executing kernel for " << iterations <<" iterations" << std::endl;
    std::cout<<"-------------------------------------------"<<std::endl;

    // Measure bandwidth of uncached linear reads from global buffer
    int status = bandwidth(kernel[2], outputBufferReadLU, NULL, &readLinearUncachedTime,
                           &readLinearUncachedGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Measure bandwidth of linear reads from global buffer
    status = bandwidth(kernel[1], outputBufferReadLinear, NULL, &readLinearTime,
                       &readLinearGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    writeFlag = true;
    // Measure bandwidth of linear writes to global buffer
    status = bandwidth(kernel[3], outputBufferWriteLinear, NULL, &writeLinearTime,
                       &writeLinearGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    writeFlag = false;

    // Measure bandwidth of single reads from global buffer
    status = bandwidth(kernel[0], outputBufferReadSingle,NULL, &readSingleTime,
                       &readSingleGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Measure bandwidth of random reads from global buffer
    status = bandwidth(kernel[4], outputBufferReadRandom, NULL, &readRandomTime,
                       &readRandomGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    extraFlag = true;
    status = bandwidth(kernel[5], outputBufferReadunCombine, NULL, &readUncombineTime,
                       &readUncombineGbps);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    extraFlag = false;

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::runCLKernels_SVM(void)
{
#ifdef CL_VERSION_2_0
    std::cout << "Executing kernel for " << iterations <<" iterations" << std::endl;
    std::cout<<"-------------------------------------------"<<std::endl;

    // Measure bandwidth of uncached linear reads from global buffer
    int status = bandwidth(kernel[2], outputBufferReadLU, NULL, &readLinearUncachedTime,
                           &readLinearUncachedGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Measure bandwidth of linear reads from global buffer
    status = bandwidth(kernel[1], outputBufferReadLinear, NULL, &readLinearTime,
                       &readLinearGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    writeFlag = true;
    // Measure bandwidth of linear writes to global buffer
    status = bandwidth(kernel[3], NULL, outputSVMBufferWriteLinear, &writeLinearTime,
                       &writeLinearGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    writeFlag = false;

    // Measure bandwidth of single reads from global buffer
    status = bandwidth(kernel[0], outputBufferReadSingle, NULL, &readSingleTime,
                       &readSingleGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Measure bandwidth of random reads from global buffer
    status = bandwidth(kernel[4], outputBufferReadRandom, NULL, &readRandomTime,
                       &readRandomGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    extraFlag = true;
    status = bandwidth(kernel[5], outputBufferReadunCombine, NULL, &readUncombineTime,
                       &readUncombineGbps, true);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    extraFlag = false;
#endif
    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize())
    {
        return SDK_FAILURE;
    }

    Option* num_iterations = new Option;
    CHECK_ALLOCATION(num_iterations,"num_iterators memory allocation failed");

    num_iterations->_sVersion = "i";
    num_iterations->_lVersion = "iterations";
    num_iterations->_description = "Number of iterations for kernel execution";
    num_iterations->_type = CA_ARG_INT;
    num_iterations->_value = &iterations;

    sampleArgs->AddOption(num_iterations);
    delete num_iterations;

    Option* num_arguments = new Option;
    CHECK_ALLOCATION(num_arguments,"num_arguments memory allocation failed");

    num_arguments->_sVersion = "c";
    num_arguments->_lVersion = "components";
    num_arguments->_description =
        "Number of vector components to be used. Can be either 1,2,3(4),4,8,16";
    num_arguments->_type = CA_ARG_INT;
    num_arguments->_value = &vectorSize;

    sampleArgs->AddOption(num_arguments);
    delete num_arguments;

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::setup()
{
    if(iterations < 1)
    {
        std::cout<<"Error, iterations cannot be 0 or negative. Exiting..\n";
        exit(0);
    }

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    int status = setupCL();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    if(setupGlobalMemoryBandwidth()!=SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    return SDK_SUCCESS;
}


int
GlobalMemoryBandwidth::run()
{
	bool useSVM = false;

    // Arguments are set and execution call is enqueued on command buffer
    if(runCLKernels() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	if(sampleArgs->verify && verifyResults(useSVM) != SDK_SUCCESS)
    {
		return SDK_FAILURE;
	}

	printStats();

	//Run SVM buffer kernels
	if (svmSupport)
	{
		std::cout << std::endl;
		std::cout<<"-------------------------------------------"<<std::endl;
		std::cout << "Results with SVM buffer" << std::endl;
		std::cout<<"-------------------------------------------"<<std::endl;

		useSVM = true;
		if(runCLKernels_SVM() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
		if(sampleArgs->verify && verifyResults(useSVM) != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}

		printStats();
	}

    return SDK_SUCCESS;
}

int
GlobalMemoryBandwidth::verifyResults(bool useSVM = false)
{
    if(sampleArgs->verify)
    {
        int vecElements = (vec3 == true) ? 3 : vectorSize;
        int sizeElement = vectorSize * sizeof(cl_float);
        int readLength = length + (NUM_READS * 1024 / sizeElement) + EXTRA_ELEMENTS;
        int status, passStatus;

        // Verify result for single access
        verificationOutput = (cl_float*)malloc(length * vectorSize * sizeof(cl_float));
        CHECK_ALLOCATION(verificationOutput,
                         "verificationOutput memory allocation failed");

#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			//Data for SVM buffers
			status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_READ, 
									 inputSVMBuffer, (readLength * sizeElement),
									 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (inputSVMBuffer) failed!");

			input = inputSVMBuffer;
		}
		else
#endif
		{
			/*
			 * Map cl_mem inputBuffer to host for reading
			 * device->host transfer happens if device exists in different address-space
			 */
			status = mapBuffer( inputBuffer, input,
								(readLength * sizeElement),
								CL_MAP_READ );
			CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(inputBuffer)");
		}
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "\nVerifying results for Read-Linear(uncached) : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

        // Verify result for Linear access
        for(int i = 0; i < (int)length; i++)
        {
            int readPos = i;
            for(int j = 0; j < NUM_READS; j++)
            {
                readPos += OFFSET;
                for(int k = 0; k < vecElements; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[readPos * vectorSize + k];
                }
            }
        }

        // Map cl_mem outputBufferReadLU to host for reading
        status = mapBuffer( outputBufferReadLU, outputReadLU,
                            (length * sizeElement),
                            CL_MAP_READ );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(outputBufferReadLU)");
		
		passStatus = 0;
		float * devBuffer = (float *)outputReadLU;
		float * refBuffer = (float *)verificationOutput;
		for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
        {
			for(int j=0 ; j<vecElements;j++)
			{

            	float fErr = devBuffer[j] -refBuffer[j];
	   			if(fErr < (float)0)
	   				fErr = -fErr;
	   			
				if(fErr>(float)1e-5)
	   			{
          			passStatus = 1;
					break;			
	   			}
			}
			if (passStatus != 0)
				break;

			devBuffer+= vectorSize;
			refBuffer+= vectorSize;
        }

        status = unmapBuffer(outputBufferReadLU, outputReadLU);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBufferReadLU)");

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "\nVerifying results for Read-Linear : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

        // Verify result for Linear access
        int index = 0;
        for(int i = 0; i < (int)length; i++)
        {
            index = i;
            for(int j = 0; j < NUM_READS; j++)
            {
                for(int k = 0; k < vecElements; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[(index + j) * vectorSize + k];
                }
            }
        }

        // Map cl_mem outputBufferReadLU to host for reading
        status = mapBuffer( outputBufferReadLinear, outputReadLinear,
                            (length * sizeElement),
                            CL_MAP_READ );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(outputBufferReadLinear)");

		passStatus = 0;
		devBuffer = (float *)outputReadLinear;
		refBuffer = (float *)verificationOutput;
		for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
        {
			for(int j=0 ; j<vecElements;j++)
			{

            	float fErr = devBuffer[j] -refBuffer[j];
	   			
				if(fErr < (float)0)
	   				fErr = -fErr;
	   			
				if(fErr>(float)1e-5)
	   			{
          			passStatus = 1;
					break;			
	   			}
			}
			if (passStatus != 0)
					break;

			devBuffer+= vectorSize;
			refBuffer+= vectorSize;
        }

        status = unmapBuffer(outputBufferReadLinear, outputReadLinear);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBufferReadLinear)");

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "\nVerifying results for Write-Linear : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			//Data for SVM buffers
			status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_READ, 
									 outputSVMBufferWriteLinear, (length * sizeElement),
									 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (outputSVMBufferWriteLinear) failed!");

			passStatus = 0;
			devBuffer = (float *)outputSVMBufferWriteLinear;
			refBuffer = (float *)verificationOutput;
			for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
			{
				for(int j=0 ; j<vecElements;j++)
				{

            		float fErr = devBuffer[j] -refBuffer[j];
	   			
					if(fErr < (float)0)
	   					fErr = -fErr;
	   			
					if(fErr>(float)1e-5)
	   				{
          				passStatus = 1;
						break;			
	   				}
				}
				if (passStatus != 0)
					break;

				devBuffer+= vectorSize;
				refBuffer+= vectorSize;
			}        

			status = clEnqueueSVMUnmap(commandQueue, outputSVMBufferWriteLinear, 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (outputSVMBufferWriteLinear) failed!");
		}
		else
#endif
		{
			// Map cl_mem outputBufferWriteLinear to host for reading
			status = mapBuffer( outputBufferWriteLinear, outputWriteLinear,
								(length * sizeElement),
								CL_MAP_READ );
			CHECK_ERROR(status, SDK_SUCCESS,
						"Failed to map device buffer.(outputBufferWriteLinear)");
		
			passStatus = 0;
			devBuffer = (float *)outputWriteLinear;
			refBuffer = (float *)verificationOutput;
			for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
			{
				for(int j=0 ; j<vecElements;j++)
				{

            		float fErr = devBuffer[j] -refBuffer[j];
	   			
					if(fErr < (float)0)
	   					fErr = -fErr;
	   			
					if(fErr>(float)1e-5)
	   				{
          				passStatus = 1;
						break;			
	   				}
				}
				if (passStatus != 0)
					break;

				devBuffer+= vectorSize;
				refBuffer+= vectorSize;
			}        

			status = unmapBuffer(outputBufferWriteLinear, outputWriteLinear);
			CHECK_ERROR(status, SDK_SUCCESS,
						"Failed to unmap device buffer.(outputBufferWriteLinear)");
		}

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "\nVerifying results for Read-Single : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

        for(int i = 0; i < (int)length; i++)
        {
            for(int j = 0; j < NUM_READS; j++)
            {
                for(int k = 0; k < vecElements; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[(j * vectorSize) + k];
                }
            }
        }

        // Map cl_mem outputBufferReadSingle to host for reading
        status = mapBuffer( outputBufferReadSingle, outputReadSingle,
                            (length * sizeElement),
                            CL_MAP_READ );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(outputBufferReadSingle)");

		passStatus = 0;
		devBuffer = (float *)outputReadSingle;
		refBuffer = (float *)verificationOutput;
		for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
        {
			for(int j=0 ; j<vecElements;j++)
			{

            	float fErr = devBuffer[j] -refBuffer[j];
	   			if(fErr < (float)0)
	   			fErr = -fErr;
	   			if(fErr>(float)1e-5)
	   			{
          			passStatus = 1;
					break;			
	   			}
			}
			if (passStatus != 0)
				break;

			devBuffer+= vectorSize;
			refBuffer+= vectorSize;
        }        
	
		status = unmapBuffer(outputBufferReadSingle, outputReadSingle);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBufferReadSingle)");

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        cl_uint index1,readpos;
        std::cout << "\nVerifying results for Read-Random : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

		

        // Verify result for Random access
        for(int i = 0; i < (int)length; i++)
        {
            index1 = i % localThreads;
            //cl_uint index = 0;
            for(int j = 0; j < NUM_READS; j++)
            {
                for(int k =0 ; k<vecElements ; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[index1 * vectorSize + k];
                }
                index1 = (cl_uint)(input[index1 * vectorSize + 0]);
            }
        }

        // Map cl_mem outputBufferReadRandom to host for reading
        status = mapBuffer( outputBufferReadRandom, outputReadRandom,
                            (length * sizeElement),
                            CL_MAP_READ );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(outputBufferReadRandom)");
	
		passStatus = 0;
		devBuffer = (float *)outputReadRandom;
		refBuffer = (float *)verificationOutput;
		for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
        {
			for(int j=0 ; j<vecElements;j++)
			{

            	float fErr = devBuffer[j] -refBuffer[j];
	   			if(fErr < (float)0)
	   			fErr = -fErr;
	   			if(fErr>(float)1e-5)
	   			{
          			passStatus = 1;
					break;			
	   			}
			}
			if (passStatus != 0)
				break;

			devBuffer+= vectorSize;
			refBuffer+= vectorSize;
        }
       
        status = unmapBuffer(outputBufferReadRandom, outputReadRandom);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBufferReadRandom)");

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "\nVerifying results for Read-unCombine : ";
        memset(verificationOutput, 0, length * vectorSize * sizeof(cl_float));

        cl_uint lengthExtra = (length * NUM_READS) + NUM_READS;

#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			//Data for SVM buffers
			status = clEnqueueSVMMap(commandQueue, CL_TRUE, CL_MAP_READ, 
									 inputSVMBufferExtra, (lengthExtra * sizeElement),
									 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMMap (inputSVMBufferExtra) failed!");

			inputExtra = inputSVMBufferExtra;
		}
		else
#endif
		{
			// Map cl_mem inputBufferExtra to host for reading
			status = mapBuffer( inputBufferExtra, inputExtra,
								(lengthExtra * sizeElement),
								CL_MAP_READ );
			CHECK_ERROR(status, SDK_SUCCESS,
						"Failed to map device buffer.(inputBufferExtra)");
		}

        for(int i =0; i<(int)length; i++)
        {
            for(int j=0; j<NUM_READS; j++)
            {
                readpos = i*NUM_READS + j;
                for(int k=0; k<vecElements; k++)
                {
                    verificationOutput[i * vectorSize + k] +=
                        inputExtra[(readpos) * vectorSize + k];

                }
            }
        }

#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			inputExtra = NULL;
			status = clEnqueueSVMUnmap(commandQueue, inputSVMBufferExtra, 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (inputSVMBufferExtra) failed!");
		}
		else
#endif
		{
			// Unmap cl_mem inputBufferExtra from host
			status = unmapBuffer(inputBufferExtra, inputExtra);
			CHECK_ERROR(status, SDK_SUCCESS,
						"Failed to unmap device buffer.(inputBufferExtra)");
		}

        // Map cl_mem outputBufferReadunCombine to host for reading
        status = mapBuffer( outputBufferReadunCombine, outputReadunCombine,
                            (length * sizeElement),
                            CL_MAP_READ );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(outputBufferReadunCombine)");
	
		passStatus = 0;
		devBuffer = (float *)outputReadunCombine;
		refBuffer = (float *)verificationOutput;
		for(int i = 0; i < (int)(length * vectorSize); i+=vectorSize)
        {
			for(int j=0 ; j<vecElements;j++)
			{

            			float fErr = devBuffer[j] -refBuffer[j];
	   			if(fErr < (float)0)
	   			fErr = -fErr;
	   			if(fErr>(float)1e-5)
	   			{
          			passStatus = 1;
					break;			
	   			}
			}
			if (passStatus != 0)
						break;

			devBuffer+= vectorSize;
			refBuffer+= vectorSize;
        }
        
        status = unmapBuffer(outputBufferReadunCombine, outputReadunCombine);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBufferReadunCombine)");

        if(passStatus == 0)
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CL_VERSION_2_0
		if (useSVM)
		{
			input = NULL;
			status = clEnqueueSVMUnmap(commandQueue, inputSVMBuffer, 0, NULL, NULL);
			CHECK_OPENCL_ERROR(status, "clEnqueueSVMUnmap (inputSVMBuffer) failed!");
		}
		else
#endif
		{
			// Unmap cl_mem inputBuffer from host
			status = unmapBuffer(inputBuffer, input);
			CHECK_ERROR(status, SDK_SUCCESS, "Failed to unmap device buffer.(inputBuffer)");
		}
    }

    return SDK_SUCCESS;
}

void
GlobalMemoryBandwidth::printStats()
{
    std::string strArray[3];
    std::string stats[3];
    sampleArgs->timing = true;

    int sizeInBytesPerIter = (int) (NUM_READS * vectorSize * sizeof(
                                        cl_float) * globalThreads);
    std::cout << std::endl << std::setw(18) << std::left
              << "Vector width used " << ": " << ((vec3)? 3: vectorSize) << std::endl;
    std::cout << std::setw(18) << std::left
              << "Setup Time " << ": " << setupTime << " secs" << std::endl << std::endl;

    std::cout << "\n1. Global Memory Read: Linear Uncached" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(readLinearUncachedTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(readLinearUncachedGbps, std::dec);
    printStatistics(strArray, stats, 3);

    std::cout << "\n\n2. Global Memory Read: Linear Cached" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(readLinearTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(readLinearGbps, std::dec);
    printStatistics(strArray, stats, 3);

    std::cout << "\n\n3. Global Memory Read: Single" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(readSingleTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(readSingleGbps, std::dec);
    printStatistics(strArray, stats, 3);

    std::cout << "\n\n4. Global Memory Read: Random" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(readRandomTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(readRandomGbps, std::dec);
    printStatistics(strArray, stats, 3);

    std::cout << "\n\n5. Global Memory Read: UnCombine_unCache" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(readUncombineTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(readUncombineGbps, std::dec);
    printStatistics(strArray, stats, 3);

    std::cout << "\n\n6. Global Memory Write: Linear" << std::endl;
    strArray[0] = "Size (Bytes)";
    stats[0] = toString(sizeInBytesPerIter, std::dec);
    strArray[1] = "Avg. Kernel Time (sec)";
    stats[1] = toString(writeLinearTime, std::dec);
    strArray[2] = "Avg Bandwidth (GBPS)";
    stats[2] = toString(writeLinearGbps, std::dec);
    printStatistics(strArray, stats, 3);

}


int
GlobalMemoryBandwidth::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer)");

    status = clReleaseMemObject(outputBufferReadSingle);
    CHECK_OPENCL_ERROR(status,
                       "clReleaseMemObject failed.(outputBufferReadSingle)");

    status = clReleaseMemObject(outputBufferReadLinear);
    CHECK_OPENCL_ERROR(status,
                       "clReleaseMemObject failed.(outputBufferReadLinear)");

    status = clReleaseMemObject(outputBufferReadLU);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBufferReadLU)");

    status = clReleaseMemObject(outputBufferWriteLinear);
    CHECK_OPENCL_ERROR(status,
                       "clReleaseMemObject failed.(outputBufferWriteLinear)");


    status = clReleaseMemObject(outputBufferReadRandom);
    CHECK_OPENCL_ERROR(status,
                       "clReleaseMemObject failed.(outputBufferWriteLinear)");

    status = clReleaseMemObject(outputBufferReadunCombine);
    CHECK_OPENCL_ERROR(status,
                       "clReleaseMemObject failed.(outputBufferWriteLinear)");

    status = clReleaseMemObject(inputBufferExtra);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBufferExtra)");

	if (svmSupport)
	{
#ifdef CL_VERSION_2_0
		//release svm buffer
		clSVMFree(context,inputSVMBuffer);
		clSVMFree(context,inputSVMBufferExtra);
		clSVMFree(context,outputSVMBufferWriteLinear);
#endif
	}

	if(constValue)
    {
        status = clReleaseMemObject(constValue);
        CHECK_OPENCL_ERROR(status, "clReleaseMemOnject failed.");
    }

	for (int i = 0; i < NUM_KERNELS; i++)
	{
		status = clReleaseKernel(kernel[i]);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");
	}

	status = clReleaseCommandQueue(commandQueue);
	CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

	status = clReleaseProgram(program);
	CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed. (context)");

    FREE(verificationOutput);
    FREE(devices);

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    cl_int status = 0;
    GlobalMemoryBandwidth clGlobalMemoryBandwidth;

    if(clGlobalMemoryBandwidth.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clGlobalMemoryBandwidth.sampleArgs->parseCommandLine(argc,
            argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clGlobalMemoryBandwidth.sampleArgs->isDumpBinaryEnabled())
    {
        return clGlobalMemoryBandwidth.genBinaryImage();
    }

    status = clGlobalMemoryBandwidth.setup();
    if(status != SDK_SUCCESS)
    {
        if(status == SDK_EXPECTED_FAILURE)
        {
            return SDK_SUCCESS;
        }

        return SDK_FAILURE;
    }

    if(clGlobalMemoryBandwidth.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clGlobalMemoryBandwidth.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	
    return SDK_SUCCESS;
}
