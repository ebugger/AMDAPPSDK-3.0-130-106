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


#include "BinarySearchDeviceSideEnqueue.hpp"
#include <malloc.h>

/*
 * \brief set up program input data
 */
int BinarySearch::setupBinarySearch()
{
    // allocate and init memory used by host
    cl_uint max = length * 20;
    cl_uint inputSizeBytes = length *  sizeof(cl_uint);

    int status = mapBuffer( inputBuffer, input, inputSizeBytes,
                            CL_MAP_WRITE_INVALIDATE_REGION);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(inputBuffer in setupBinarySearch)");

	
	status = mapBuffer( keysBuffer, keys_array, nKeys * sizeof(cl_uint),
                            CL_MAP_WRITE_INVALIDATE_REGION);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(keysBuffer in setupBinarySearch)");


	
	cl_uint n=0;
	cl_uint power_by_2 = 0;
	
	/**** calculating the value of N  for 2^N=length ******/
	int length_temp = length;
	for(int count=0 ; 1; count++)
	{
		length_temp >>=1 ;
		power_by_2++;
		if(length_temp == 0)
			break;
	}
	--power_by_2;

	/***** Fixing the Left shift according to the Power of 2 . Such that to prevent Overflows ****/
	n = power_by_2 - 3 ;
	n = (n>=16)?16:n;

	/****** random initialisation of input ***********/
    input[0] = 0;

	srand(seed);
    for(cl_uint i = 1; i < length; i++)
    {
		
		input[i] = input[i - 1] + ( (cl_uint) ((max *(unsigned int)rand()) / (unsigned int)(RAND_MAX<<n)) + 1);		
		

		/****** Checking the Overflow for the input buffer values ******/
		u64_t Overflow = 0;
		Overflow = (u64_t)((u64_t)input[i - 1] + (u64_t)(((max * (unsigned int)rand()) / (unsigned int)(RAND_MAX<<n)) + 1));
		if(Overflow > (u64_t)MAXUINT)
		{
		status = 1;
		CHECK_ERROR(status, SDK_SUCCESS,
                "Overflow occurred in input buffer's value : Lower the length of the array");
		}

		

	}

	/***** Initializations of keys based on one key input *********/;
	for(cl_uint i=0;i<nKeys;i++)
		keys_array[i] = (i*i) + findMe + (cl_uint)pow((double)2,(double)n) ;
	
	globalLowerBound = (cl_uint *) calloc(nKeys ,sizeof(cl_uint));
        CHECK_ALLOCATION(globalLowerBound,
                         "Failed to allocate host memory. (globalLowerBound)");

	globalUpperBound = (cl_uint *) calloc(nKeys ,sizeof(cl_uint));
        CHECK_ALLOCATION(globalUpperBound,
                         "Failed to allocate host memory. (globalUpperBound)");

	elementIndex = (cl_uint *) calloc(nKeys ,sizeof(cl_uint));
        CHECK_ALLOCATION(elementIndex,
                         "Failed to allocate host memory. (elementIndex)");

	isElementFound = (cl_uint *) calloc(nKeys ,sizeof(cl_uint));
        CHECK_ALLOCATION(isElementFound,
                         "Failed to allocate host memory. (isElementFound)");


    /*
     * Unless sampleArgs->quiet mode has been enabled, print the INPUT array.
     */
    if(!sampleArgs->quiet)
    {
        printArray<cl_uint>(
            "Sorted Input",
            input,
            length,
            1);
    }

    status = unmapBuffer( inputBuffer, input);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(inputBuffer in setupBinarySearch)");

	status = unmapBuffer( keysBuffer, keys_array);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(keysBuffer in setupBinarySearch)");


    return SDK_SUCCESS;
}

template<typename T>
int BinarySearch::mapBuffer(cl_mem deviceBuffer, T* &hostPointer,
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
BinarySearch::unmapBuffer(cl_mem deviceBuffer, void* hostPointer)
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
BinarySearch::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("BinarySearchDeviceSideEnqueue_Kernels.cl");
    binaryData.flagsStr = std::string("");
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }
    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    return status;
}


/*
 * \brief OpenCL related initialisations are done here.
 *        Context, Device list, Command Queue for host and devices are set up.
 *        Calls are made to set up OpenCL memory buffers that this program uses
 *        and to load the programs into memory and get kernel handles.
 *          Load and build OpenCL program and get kernel handles.
 *        Set up OpenCL memory buffers used by this program.
 */

int
BinarySearch::setupCL(void)
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
    CHECK_OPENCL_ERROR( status, "clCreateContextFromType failed.");

    // getting device on which to run the sample
    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");

	//Set device info of given cl_device_id
	status = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
	CHECK_OPENCL_ERROR(status, "deviceInfo.setDeviceInfo failed");

	// Check of OPENCL_C_VERSION if device version is 2.0 or higher
	isOpenCL2_XSupported = deviceInfo.checkOpenCL2_XCompatibility();
    if(!isOpenCL2_XSupported)
	{
		if(devEnqueue)
		{
			std::cout << std::endl << std::endl << "Device-Side Kernel Enqueue Feature not supported in 1.x platform , falling back to iterative enqueue on host" << std::endl;
			devEnqueue = false;
		}
		sampleArgs->flags = "";
	}

	printf("\n\n***********************************************************************************\n");
	if(devEnqueue)
		printf("Binary Search Using OpenCL 2.x Device-Side Enqueue Feature \n");
	else
		printf("Binary Search Using OpenCL 1.x \n");
	printf("***********************************************************************************\n\n");

    {
        // The block is to move the declaration of prop closer to its use
        const cl_queue_properties prop = 0 ;
        commandQueue = clCreateCommandQueueWithProperties(
                           context,
                           devices[sampleArgs->deviceId],
                           prop,
                           &status);
        CHECK_OPENCL_ERROR( status, "clCreateCommandQueueWithProperties failed.");
    }

	// Create Device command Queue for en-queuing commands inside the kernel  :
	if(devEnqueue)
	{
		{
		// The block is to move the declaration of prop closer to its use
			cl_queue_properties prop[] = {
			CL_QUEUE_PROPERTIES, CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE|CL_QUEUE_ON_DEVICE|CL_QUEUE_ON_DEVICE_DEFAULT,
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

	/***** Set the work group size ******/
    localThreads[0] = (NO_OF_PEs * NO_OF_WAVEFRONTS);
    numSubdivisions = length / (cl_uint)localThreads[0];

    if(numSubdivisions < localThreads[0])
    {
        numSubdivisions = (cl_uint)localThreads[0];
    }

	
	// This Below code snippets ensure the program launches the kernel for two or more times .
	if(numSubdivisions > (deviceInfo.maxComputeUnits * (NO_OF_PEs * NO_OF_WAVEFRONTS) ))
		numSubdivisions = (deviceInfo.maxComputeUnits * (NO_OF_PEs * NO_OF_WAVEFRONTS) );
	

    inputBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY,
                      sizeof(cl_uint) * length,
                      NULL,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer)");

    outputBuffer = clCreateBuffer(
                       context,
                       CL_MEM_WRITE_ONLY,
                       sizeof(cl_uint4) * nKeys,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBuffer)");

	keysBuffer = clCreateBuffer(
                       context,
                       CL_MEM_READ_ONLY,
                       sizeof(cl_uint) * nKeys ,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (keysBuffer)");


	if(devEnqueue)
	{
		parentGlobalids = clCreateBuffer(
						   context,
						   CL_MEM_READ_WRITE,
						   sizeof(cl_uint) * nKeys ,
						   NULL,
						   &status);
		CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (parentGlobalids)");
			
	}
	
		globalLowerBoundbuf = clCreateBuffer(
						   context,
						   CL_MEM_READ_WRITE,
						   sizeof(cl_uint) * nKeys ,
						   NULL,
						   &status);
		CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (globalLowerBoundbuf)");

	

    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("BinarySearchDeviceSideEnqueue_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
	if(!isOpenCL2_XSupported)
    buildData.flagsStr = std::string("-cl-std=CL1.2");
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
    kernel = clCreateKernel(program, "binarySearch_device_enqueue_multiKeys", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.");
	}
	else
	{
	// get a kernel object handle for a kernel with the given name
    kernel = clCreateKernel(program, "binarySearch", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.");

	}

    return SDK_SUCCESS;
}

int
BinarySearch::runCLKernels(void)
{
    cl_int   status;

    size_t globalThreads[1];
    globalThreads[0] = numSubdivisions;

    // Check group size against kernelWorkGroupSize
    status = kernelInfo.setKernelWorkGroupInfo(kernel,
             devices[sampleArgs->deviceId]);
    CHECK_ERROR(status, SDK_SUCCESS, "clGetKernelWorkGroupInfo failed.");

    if((cl_uint)(localThreads[0]) > kernelInfo.kernelWorkGroupSize)
    {
        if(!sampleArgs->quiet)
        {
            std::cout << "Out of Resources!" << std::endl;
            std::cout << "Group Size specified : " << localThreads[0] << std::endl;
            std::cout << "Max Group Size supported on the kernel : "
                      << kernelInfo.kernelWorkGroupSize << std::endl;
            std::cout << "Changing the group size to "
                      << kernelInfo.kernelWorkGroupSize << std::endl;
        }

        localThreads[0] = kernelInfo.kernelWorkGroupSize;
        numSubdivisions = length / (cl_uint)localThreads[0];
        if(numSubdivisions < localThreads[0])
        {
            numSubdivisions = (cl_uint)localThreads[0];
        }
        globalThreads[0] = numSubdivisions;
    }


    /**
     * Since a plain binary search on the GPU would not achieve much benefit over the GPU
     * we are doing an N'ary search. We split the array into N segments every pass and therefore
     * get log (base N) passes instead of log (base 2) passes.
     *
     * In every pass, only the thread that can potentially have the element we are looking for
     * writes to the output array. For ex: if we are looking to find 4567 in the array and every
     * thread is searching over a segment of 1000 values and the input array is 1, 2, 3, 4,...
     * then the first thread is searching in 1 to 1000, the second one from 1001 to 2000, etc.
     * The first one does not write to the output. The second one doesn't either. The fifth one however is from
     * 4001 to 5000. So it can potentially have the element 4567 which lies between them.
     *
     * This particular thread writes to the output the lower bound, upper bound and whether the element equals the lower bound element.
     * So, it would be 4001, 5000, 0
     *
     * The next pass would subdivide 4001 to 5000 into smaller segments and continue the same process from there.
     *
     * When a pass returns 1 in the third element, it means the element has been found and we can stop executing the kernel.
     * If the element is not found, then the execution stops after looking at segment of size 1.
     */


    iglobalLowerBound = 0;
    iglobalUpperBound = length - 1;
    cl_uint subdivSize = (iglobalUpperBound - iglobalLowerBound + 1) /
                         numSubdivisions;
	
    isElementFoundi = 0;
    cl_uint inputSizeBytes = length *  sizeof(cl_uint);

    status = mapBuffer( inputBuffer, input, inputSizeBytes, CL_MAP_READ);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(inputBuffer in setupBinarySearch");
	
	/***** Note: Needed Only for Single Key Mode **********/
	if(nKeys == 1)
	{
		if((input[0] > findMe) || (input[length - 1] < findMe))
		{
			status = unmapBuffer( inputBuffer, input);
			CHECK_ERROR(status, SDK_SUCCESS, "Failed to unmap device buffer.(inputBuffer)");

			return SDK_SUCCESS;
		}
	}

	/****  Initialize the elementIndex,globalLowerBound and isElementFound CPU buffers **********/

	memset(elementIndex,0,nKeys * sizeof(cl_uint));
	memset(isElementFound,0,nKeys * sizeof(cl_uint));
	memset(globalLowerBound,0,nKeys * sizeof(cl_uint));
	
	


    status = mapBuffer( outputBuffer, output, sizeof(cl_uint4) * nKeys, CL_MAP_WRITE );
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(outputBuffer)");

    output[3] = 1;
    status = unmapBuffer( outputBuffer, output);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(outputBuffer)");

    status = unmapBuffer( inputBuffer, input);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(inputBuffer in setupBinarySearch)");


    // Set appropriate arguments to the kernel

    /*
     * First argument of the kernel is the output buffer
     */
    status = clSetKernelArg(
                 kernel,
                 0,
                 sizeof(cl_mem),
                 (void *)&outputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg 0(OutputBuffer) failed.");

    status = mapBuffer( outputBuffer, output, sizeof(cl_uint4) * nKeys, CL_MAP_WRITE );
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(outputBuffer)");

	/******* Initialising the values for Lower-bound for each keys ********/
	status = mapBuffer( globalLowerBoundbuf, clglobalLowerBound, sizeof(cl_uint) * nKeys, CL_MAP_WRITE );
				CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(globalLowerBound_buf)");

	memset(clglobalLowerBound,0,nKeys * sizeof(cl_uint));
	

    if(devEnqueue)
	{ 
		cl_uint *parent_globalids_ptr = NULL;
		/***  Clearing the parent_globalids openCL global buffer ***/ 
		status = mapBuffer( parentGlobalids, parent_globalids_ptr, sizeof(cl_uint) * nKeys,
                            CL_MAP_WRITE);
		CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(parentGlobalids)");

		memset(parent_globalids_ptr,0,nKeys * sizeof(cl_uint));
			

		status = unmapBuffer( parentGlobalids, parent_globalids_ptr);
		CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(parentGlobalids)");

		/**** Unmap the globalLowerBound_buf global buffer ******/
		status = unmapBuffer( globalLowerBoundbuf, clglobalLowerBound );
				CHECK_ERROR(status, SDK_SUCCESS,
							"Failed to unmap device buffer.(globalLowerBoundbuf)");

       	/*
		 *  Setting subdivSize as argument 3
		 */
		status = clSetKernelArg(
						kernel,
						2,
						sizeof(cl_uint),
						(void*)&subdivSize);
		CHECK_OPENCL_ERROR(status, "clSetKernelArg 2(subdivSize) failed");

		/*
		 *  Setting subdivSize as argument 4
		 */
		status = clSetKernelArg(
						kernel,
						3,
						sizeof(cl_mem),
						(void*)&globalLowerBoundbuf);
		CHECK_OPENCL_ERROR(status, "clSetKernelArg 3(globalLowerBoundbuf) failed");
		

        /*
        * Second argument is input buffer
        */
        status = clSetKernelArg(
                     kernel,
                     1,
                     sizeof(cl_mem),
                     (void *)&inputBuffer);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg 1(inputBuffer) failed.");

        /*
        * Fifth argument is the elements we are looking for
        */
        status = clSetKernelArg(
                     kernel,
                     4,
                     sizeof(cl_mem),
                     (void *)&keysBuffer);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg 4(keysBuffer) failed.");

		/*
        * Pass No of Keys to kernel .
        */

	
        status = clSetKernelArg(
                     kernel,
                     5,
                     sizeof(cl_uint),
                     (void *)&nKeys);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg 5(nKeys) failed.");


		/*
        * Pass parent_globalids global buffer to kernel .
        */

	
        status = clSetKernelArg(
                     kernel,
                     6,
                     sizeof(cl_mem),
                     (void *)&parentGlobalids);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg 6(parentGlobalids) failed.");


		/*
        * Pass Global threads to kernel .
        */

		cl_uint globalThreadsKernel = (cl_uint)globalThreads[0] ;
        status = clSetKernelArg(
                     kernel,
                     7,
                     sizeof(cl_uint),
                     (void *)&globalThreadsKernel);
        CHECK_OPENCL_ERROR(status, "clSetKernelArg 7(globalThreadsKernel) failed.");

		/****** Initializing the outputBuffer of the openCL device *******/
		memset(output,0,nKeys * sizeof(cl_uint4));
		
        status = unmapBuffer( outputBuffer, output );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(outputBuffer)");

        /*
        * Enqueue a kernel run call
        */
        cl_event ndrEvt;
        status = clEnqueueNDRangeKernel(commandQueue,
                                        kernel,
                                        1,
                                        NULL,
                                        globalThreads,
                                        localThreads,
                                        0,
                                        NULL,
                                        &ndrEvt);
        CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");

        status = clFlush(commandQueue);
        CHECK_OPENCL_ERROR(status, "clFlush failed.");

        status = waitForEventAndRelease(&ndrEvt);
        CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

        status = mapBuffer( outputBuffer, output, sizeof(cl_uint4) * nKeys, CL_MAP_WRITE );
        CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(outputBuffer)");

		/**** Calculating Lower and Upper Bounds for each keys and also handling Error conditions from the 
		 output of the openCL device *****/
		for(cl_uint x=0; x<nKeys; x++)
		{
			cl_uint errorCheck		= CL_SUCCESS;
			cl_int errorCodes		= CL_SUCCESS;
			errorCheck				= output[3]; 
			errorCodes				= (cl_int)output[2]; 
			subdivSize				= output[1];
			globalLowerBound[x]		= output[0];
			globalUpperBound[x]		= globalLowerBound[x]+subdivSize-1;
			if(subdivSize == 0)
				globalUpperBound[x] = globalLowerBound[x] + 1;
			
			if(errorCheck == 5)   //TODO  Need to update the error codes for OpenCL 2.0 in CLUtil.hpp file .
			{
				CHECK_OPENCL_ERROR(errorCodes, "En-queueing Kernel in the openCL device Failed)");
			}
				
			output += 4;
		}

	}
	else
	{
		
			cl_uint subdivSizekeys = subdivSize;
			/*** checks whether at-least one key is found or not *****/
			bool    keys_hit = true;
		
			while(subdivSizekeys > 1 && keys_hit == true)
			{
				status = unmapBuffer( globalLowerBoundbuf, clglobalLowerBound );
				CHECK_ERROR(status, SDK_SUCCESS,
							"Failed to unmap device buffer.(globalLowerBoundbuf)");

       			/*
				 *  Setting subdivSize_keys as argument 3
				 */
				status = clSetKernelArg(
								kernel,
								2,
								sizeof(cl_uint),
								(void*)&subdivSizekeys);
				CHECK_OPENCL_ERROR(status, "clSetKernelArg 2(subdivSizekeys) failed");

				/*
				 *  Setting globalLowerBound_buf as argument 4
				 */
				status = clSetKernelArg(
								kernel,
								3,
								sizeof(cl_mem),
								(void*)&globalLowerBoundbuf);
				CHECK_OPENCL_ERROR(status, "clSetKernelArg 3(globalLowerBound) failed");
		

				/*
				* Second argument is input buffer
				*/
				status = clSetKernelArg(
							 kernel,
							 1,
							 sizeof(cl_mem),
							 (void *)&inputBuffer);
				CHECK_OPENCL_ERROR(status, "clSetKernelArg 1(inputBuffer) failed.");

				/*
				* Fifth argument are the elementS we are looking for
				*/
				status = clSetKernelArg(
                     kernel,
                     4,
                     sizeof(cl_mem),
                     (void *)&keysBuffer);
				CHECK_OPENCL_ERROR(status, "clSetKernelArg 4(keysBuffer) failed.");

				/*
				* Pass No of Keys to kernel .
				*/
				  status = clSetKernelArg(
                     kernel,
                     5,
                     sizeof(cl_uint),
                     (void *)&nKeys);
				 CHECK_OPENCL_ERROR(status, "clSetKernelArg 5(nKeys) failed.");


				/****** Initializing the outputBuffer of the openCL device *******/
				memset(output,0,nKeys * sizeof(cl_uint4));

				status = unmapBuffer( outputBuffer, output );
				CHECK_ERROR(status, SDK_SUCCESS,
							"Failed to unmap device buffer.(outputBuffer)");


				
				/*
				* Enqueue a kernel run call
				*/
				cl_event ndrEvt;
				status = clEnqueueNDRangeKernel(commandQueue,
												kernel,
												1,
												NULL,
												globalThreads,
												localThreads,
												0,
												NULL,
												&ndrEvt);
				CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");

				status = clFlush(commandQueue);
				CHECK_OPENCL_ERROR(status, "clFlush failed.");

				status = waitForEventAndRelease(&ndrEvt);
				CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

				status = mapBuffer( outputBuffer, output, sizeof(cl_uint4) * nKeys, CL_MAP_WRITE );
				CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(outputBuffer)");

				status = mapBuffer( globalLowerBoundbuf, clglobalLowerBound, sizeof(cl_uint) * nKeys, CL_MAP_WRITE );
				CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(globalLowerBoundbuf)");

				cl_uint *outputPtr = (cl_uint *)output;
				keys_hit = false;
				cl_uint subdivSizeKeysTemp = 0;

				/**** Calculating Lower and Upper Bounds for each keys from the 
				output of the openCL device *****/
				for(cl_uint x=0; x<nKeys; x++)
				{
					globalLowerBound[x] += outputPtr[0]*subdivSizekeys;
					clglobalLowerBound[x]	=  globalLowerBound[x];
					globalUpperBound[x]	   = globalLowerBound[x]+subdivSizekeys-1;
					if(keys_hit == true)
					{
						keys_hit = (bool)outputPtr[3] ;
						subdivSizeKeysTemp = (globalUpperBound[x] - globalLowerBound[x] + 1)/numSubdivisions;
					}
					outputPtr+=4;

				}

				subdivSizekeys = subdivSizeKeysTemp;
				
			}

			status = unmapBuffer( globalLowerBoundbuf, clglobalLowerBound );
			CHECK_ERROR(status, SDK_SUCCESS,
							"Failed to unmap device buffer.(globalLowerBoundbuf)");
	

	}
       
	status = mapBuffer( keysBuffer, keys_array, nKeys * sizeof(cl_uint),CL_MAP_READ);
		CHECK_ERROR(status, SDK_SUCCESS,
					"Failed to map device buffer.(keysBuffer in setupBinarySearch)");

    status = mapBuffer( inputBuffer, input, inputSizeBytes, CL_MAP_READ);
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to map device buffer.(inputBuffer)");

	/*** Check whether the Keys are found in the Narrowed SearchSpace provided from the openCL device ****/
	for(cl_uint x=0; x<nKeys; x++)
	{
		for(cl_uint i=globalLowerBound[x]; i<= globalUpperBound[x]; i++)
		{
			if(input[i] == keys_array[x])
			{
				elementIndex[x] = i;
				globalLowerBound[x] = i;
				globalUpperBound[x] = i+1;
				isElementFound[x] = 1;
				break;
			}
		}
	}

    status = unmapBuffer( inputBuffer, input);
    CHECK_ERROR(status, SDK_SUCCESS, "Failed to unmap device buffer.(inputBuffer)");

    status = unmapBuffer( outputBuffer, output );
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(outputBuffer)");

	status = unmapBuffer( keysBuffer, keys_array);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(keysBuffer)");

    return SDK_SUCCESS;
}


/**
 * CPU verification for the BinarySearch algorithm
 */
int
BinarySearch::binarySearchCPUReference()
{
    cl_uint inputSizeBytes = length *  sizeof(cl_uint);

	int status = mapBuffer( keysBuffer, keys_array, nKeys * sizeof(cl_uint),
                            CL_MAP_READ);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to map device buffer.(keysBuffer in binarySearchCPUReference)");

	for(cl_uint key_count=0;key_count<nKeys;key_count++)
	{
		if(isElementFound[key_count])
		{
			if(verificationInput[globalLowerBound[key_count]] == keys_array[key_count])
			{
				continue;
			}
			else
			{
				return SDK_FAILURE;
			}
		}
		else
		{
			for(cl_uint i = 0; i < length; i++)
			{
				if(verificationInput[i] == keys_array[key_count])
				{
					return SDK_FAILURE;
				}
			}
		}
	}
	status = unmapBuffer( keysBuffer, keys_array);
    CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(keysBuffer in binarySearchCPUReference)");

	return SDK_SUCCESS;
}

int BinarySearch::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize())
    {
        return SDK_FAILURE;
    }

    // Now add customized options
    Option* array_length = new Option;
    CHECK_ALLOCATION(array_length, "Memory allocation error.\n");

    array_length->_sVersion = "x";
    array_length->_lVersion = "length";
    array_length->_description = "Length of the input array";
    array_length->_type = CA_ARG_INT;
    array_length->_value = &length;
	sampleArgs->AddOption(array_length);
	delete array_length;
	
	
	// Note : This option is used only during single-key search mode 
    Option* find_me = new Option;
    CHECK_ALLOCATION(find_me, "Memory allocation error.\n");

    find_me->_sVersion = "f";
    find_me->_lVersion = "find";
    find_me->_description = "element to be found";
    find_me->_type = CA_ARG_INT;
    find_me->_value = &findMe;
    sampleArgs->AddOption(find_me);

    delete find_me;

    Option* sub_div = new Option;
    CHECK_ALLOCATION(sub_div, "Memory allocation error.\n");

    sub_div->_sVersion = "s";
    sub_div->_lVersion = "subdivisions";
    sub_div->_description = "number of subdivisions";
    sub_div->_type = CA_ARG_INT;
    sub_div->_value = &numSubdivisions;
    sampleArgs->AddOption(sub_div);

    delete sub_div;
	
	// This option used to the enter the number of keys 
	Option* numKeys= new Option;
    CHECK_ALLOCATION(numKeys, "Memory allocation error.\n");

    numKeys->_sVersion = "k";
    numKeys->_lVersion = "nKeys";
    numKeys->_description = "number of input keys";
    numKeys->_type = CA_ARG_INT;
    numKeys->_value = &nKeys;
    sampleArgs->AddOption(numKeys);

	 delete numKeys;

	/** This option enables the openCL 1.2 implementation by (eq 0) : By default 
	.   the device-side kernel enqueue option is enabled 
	*/
	Option* usingKernelEnqueueOption= new Option;
    CHECK_ALLOCATION(usingKernelEnqueueOption, "Memory allocation error.\n");

    usingKernelEnqueueOption->_sVersion = "eq";
    usingKernelEnqueueOption->_lVersion = "devEnqueue";
    usingKernelEnqueueOption->_description = "device-side kernel enqueue : To run the sample using openCL 1.2 choose <eq 0> ";
    usingKernelEnqueueOption->_type = CA_ARG_INT;
    usingKernelEnqueueOption->_value = &devEnqueue;
    sampleArgs->AddOption(usingKernelEnqueueOption);

	delete usingKernelEnqueueOption;


	/** This option is used to generate different sets of random numbers for the 
	.   input buffer.
	*/
	Option* randomSeed= new Option;
    CHECK_ALLOCATION(randomSeed, "Memory allocation error.\n");

    randomSeed->_sVersion = "rs";
    randomSeed->_lVersion = "randSeed";
    randomSeed->_description = "<value> : To generate different sets of random values";
    randomSeed->_type = CA_ARG_INT;
    randomSeed->_value = &seed;
    sampleArgs->AddOption(randomSeed);

     delete randomSeed;

    Option* num_iterations = new Option;
    CHECK_ALLOCATION(num_iterations, "Memory allocation error.\n");

    num_iterations->_sVersion = "i";
    num_iterations->_lVersion = "iterations";
    num_iterations->_description = "Number of iterations for kernel execution";
    num_iterations->_type = CA_ARG_INT;
    num_iterations->_value = &iterations;

    sampleArgs->AddOption(num_iterations);

    delete num_iterations;

    return SDK_SUCCESS;
}

int BinarySearch::setup()
{
    if(iterations < 1)
    {
        std::cout<<"Error, iterations cannot be 0 or negative. Exiting..\n";
        exit(0);
    }
    //length should bigger then 0 and  divisible by 256
    if(length<=512)
    {
        length = 512;
    }
  
    if(isPowerOf2(length))
    {
        length = roundToPowerOf2(length);
    }

	
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    if(setupCL() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	sampleTimer->stopTimer(timer);
    setupTime = (cl_double)(sampleTimer->readTimer(timer));

    if(setupBinarySearch() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
	    

    return SDK_SUCCESS;
}


int BinarySearch::run()
{
    // Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
        // Arguments are set and execution call is enqueued on command buffer
        if(runCLKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

    std::cout << "Executing kernel for " << iterations <<
              " iterations" << std::endl;
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
    totalKernelTime = (double)(sampleTimer->readTimer(timer));


    if(!sampleArgs->quiet)
    {
		int status = mapBuffer( keysBuffer, keys_array, nKeys * sizeof(cl_uint),
                            CL_MAP_READ);
		CHECK_ERROR(status, SDK_SUCCESS,
					"Failed to map device buffer.(keysBuffer in setupBinarySearch)");

		for(cl_uint x=0; x<nKeys; x++)
		{
			std::cout << "For Key No: "<< x << std::endl;
			std::cout << "Lower bound = " << globalLowerBound[x] << ", Upper bound = " <<
					  globalUpperBound[x] <<
					  ", Element to be searched = " << keys_array[x] << std::endl;
			if(isElementFound[x] == 1)
			{
				std::cout<<"Element found at index "<<elementIndex[x];
			}
			else
			{
				std::cout<<"Element not found\n";
			}
		}

		status = unmapBuffer( keysBuffer, keys_array);
		CHECK_ERROR(status, SDK_SUCCESS,
                "Failed to unmap device buffer.(Keysbuffer)");

    }

    return SDK_SUCCESS;
}

int BinarySearch::verifyResults()
{
    if(sampleArgs->verify)
    {
        verificationInput = (cl_uint *) malloc(length * sizeof(cl_int));
        CHECK_ALLOCATION(verificationInput,
                         "Failed to allocate host memory. (verificationInput)");

        int status = mapBuffer( inputBuffer, input, length * sizeof(cl_int),
                                CL_MAP_READ);
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to map device buffer.(inputBuffer in sampleArgs->verify)");

        memcpy(verificationInput, input, length*sizeof(cl_int));

        status = unmapBuffer( inputBuffer, input );
        CHECK_ERROR(status, SDK_SUCCESS,
                    "Failed to unmap device buffer.(inputBuffer in sampleArgs->verify)");

        /* reference implementation
         * it overwrites the input array with the output
         */
        int refTimer = sampleTimer->createTimer();
        sampleTimer->resetTimer(refTimer);
        sampleTimer->startTimer(refTimer);
        cl_int verified = binarySearchCPUReference();
        sampleTimer->stopTimer(refTimer);
        referenceKernelTime =sampleTimer->readTimer(refTimer);

        // compare the results and see if they match
        if(verified == SDK_SUCCESS)
        {
            std::cout << "\nPassed!" << std::endl;
            return SDK_SUCCESS;
        }
        else
        {
            std::cout << "Failed\n" << std::endl;
            return SDK_FAILURE;
        }
    }

    return SDK_SUCCESS;
}

void BinarySearch::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] = {"Elements", "Set-up Time (sec)", "Avg. Kernel Time (sec)", "Elements/sec"};
        std::string stats[4];

        sampleTimer->totalTime = ( totalKernelTime / iterations );

        stats[0] = toString(length   , std::dec);
        stats[1] = toString(setupTime, std::dec);
        stats[2] = toString( sampleTimer->totalTime, std::dec);
        stats[3] = toString(length/sampleTimer->totalTime, std::dec);

        printStatistics(strArray, stats, 4);
    }
}

int BinarySearch::cleanup()
{
    /* Releases OpenCL resources (Context, Memory etc.) */
    cl_int status;

    status = clReleaseKernel(kernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.");

    status = clReleaseMemObject(inputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

	status = clReleaseMemObject(keysBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");
	
	if(devEnqueue)
	{
		status = clReleaseMemObject(parentGlobalids);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");
	}
	
	status = clReleaseMemObject(globalLowerBoundbuf);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.");

	if(devEnqueue)
	{
	status = clReleaseCommandQueue(devcommandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.");
	}
	
    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.");

    // release program resources (input memory etc.)

    FREE(devices);

    FREE(verificationInput);

	FREE(globalLowerBound);

	FREE(globalUpperBound);

	FREE(elementIndex);

	FREE(isElementFound);


    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    BinarySearch clBinarySearch;

    if(clBinarySearch.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBinarySearch.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBinarySearch.sampleArgs->isDumpBinaryEnabled())
    {
        return clBinarySearch.genBinaryImage();
    }
    else
    {
        if(clBinarySearch.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(clBinarySearch.run() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(clBinarySearch.verifyResults() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(clBinarySearch.cleanup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        clBinarySearch.printStats();
    }

    return SDK_SUCCESS;
}
