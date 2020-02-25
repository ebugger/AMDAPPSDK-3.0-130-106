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


#include "ConstantBandwidth.hpp"
#include <string>

//Separator
std::string sep = "-----------------------------------------";

int
ConstantBandwidth::setupConstantBandwidth()
{

    //Make vectorSize as 4 if -v option is 3.

    //This memory alignment is required as per OpenCL for type3 vectors
    if(vectorSize == 3)
    {
        vec3 = true;
        vectorSize = 4;
    }
    else if((1 != vectorSize) && (2 != vectorSize) && (4 != vectorSize) &&
            (8 != vectorSize) && (16 != vectorSize))
    {
        std::cout << "The vectorsize can only be one of 1,2,3(4),4,8,16!" << std::endl;
        return SDK_FAILURE;
    }

    // Allocate memory
    cl_uint size = (WAVEFRONT + NUM_READS) * vectorSize * sizeof(cl_float);

    input = (cl_float*)malloc(size);
    CHECK_ALLOCATION(input, "Failed to allocate host memory. (input)");

    // Allocate memory for output buffer
    output = (cl_float*)malloc(length * vectorSize * sizeof(cl_float));
    CHECK_ALLOCATION(output, "Failed to allocate host memory. (output)");

    if(sampleArgs->verify)
    {
        cl_uint size = length * vectorSize * sizeof(cl_float);
        verificationOutput = (cl_float*)malloc(size);
        CHECK_ALLOCATION(verificationOutput,
                         "Failed to allocate host memory. (verificationOutput)");
    }

    // random initialisation of input
    return fillRandom<cl_float>(input,
                                (WAVEFRONT + NUM_READS) * vectorSize,
                                1,
                                0,
                                1000);

}

int
ConstantBandwidth::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("ConstantBandwidth_Kernels.cl");

    // Pass vectorSize as DATATYPE to kernel
    cl_uint size = (WAVEFRONT + NUM_READS) * vectorSize * sizeof(cl_float);
    char buildOption[64];
    if(vectorSize == 1)
    {
        sprintf(buildOption, "-D DATATYPE=float -D DATATYPE2=float4 -D SIZE=%d ", size);
    }
    else
    {
        sprintf(buildOption, "-D DATATYPE=float%d -D DATATYPE2=float%d -D SIZE=%d ",
                vec3 == true ? 3 : vectorSize, vec3 == true ? 3 : vectorSize,size);
    }

    binaryData.flagsStr = std::string(buildOption);
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    CHECK_ERROR(status, SDK_SUCCESS, "OpenCL Generate Binary Image Failed");
    return SDK_SUCCESS;
}


int
ConstantBandwidth::setupCL(void)
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
    if(vStrVal.compare("1.0") >  0)
    {
        char openclVersion[1024];
        status = clGetDeviceInfo(devices[sampleArgs->deviceId],
                                 CL_DEVICE_OPENCL_C_VERSION,
                                 sizeof(openclVersion),
                                 openclVersion,
                                 0);
        CHECK_OPENCL_ERROR(status, "clGetDeviceInfo failed.");

        std::string tempStr(deviceInfo.openclCVersion);
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

    {
        // The block is to move the declaration of prop closer to its use
        cl_command_queue_properties prop = 0;
        prop |= CL_QUEUE_PROFILING_ENABLE;

        commandQueue = clCreateCommandQueue(context,
                                            devices[sampleArgs->deviceId],
                                            prop,
                                            &status);
        CHECK_OPENCL_ERROR(status, "clCreateCommandQueue failed.");
    }

    cl_uint size = (WAVEFRONT + NUM_READS) * vectorSize * sizeof(cl_float);

    // Create constant buffer
    constantBuffer = clCreateBuffer(context,
                                    CL_MEM_READ_ONLY,
                                    size,
                                    0,
                                    &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (constantBuffer)");

    // Write data to buffer
    status = clEnqueueWriteBuffer(commandQueue,
                                  constantBuffer,
                                  1,
                                  0,
                                  size,
                                  input,
                                  0,
                                  0,
                                  0);
    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (constantBuffer)");


    // Create output buffer
    outputBuffer = clCreateBuffer(context,
                                  CL_MEM_WRITE_ONLY,
                                  sizeof(cl_float) * vectorSize * length,
                                  0,
                                  &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (outputBuffer)");



    // create a CL program using the kernel source

    // Pass vectorSize as DATATYPE to kernel
    char buildOption[128];
    if(vectorSize == 1)
    {
        sprintf(buildOption, "-D DATATYPE=float -D DATATYPE2=float4 -D SIZE=%d ", size);

    }
    else
    {
        sprintf(buildOption, "-D DATATYPE=float%d -D DATATYPE2=float%d -D SIZE=%d ",
                vec3 == true ? 3 : vectorSize, vec3 == true ? 3 : vectorSize,size);

    }

	//Set the data type based on OS configuration 64/32
	//compiler can better optimize the address calculations because the static offsets can be precomputed into 64-bit offsets
	if (is64Bit())
	{
		strcat(buildOption, "-D IDXTYPE=ulong ");
	}
	else
	{
		strcat(buildOption, "-D IDXTYPE=uint ");
	}

    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("ConstantBandwidth_Kernels.cl");
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

    // ConstantBuffer bandwidth from single access
    kernel[0] = clCreateKernel(program, "constantBandwidth_single_static", &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateKernel failed.(constantBandwidth_single_static)");

    kernel[1] = clCreateKernel(program, "constantBandwidth_single_dynamic",
                               &status);
    CHECK_OPENCL_ERROR(status,
                       "clCreateKernel failed.(constantBandwidth_single_dynamic)");

    // ConstantBuffer bandwidth from linear access
    kernel[2] = clCreateKernel(program, "constantBandwidth_linear", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(constantBandwidth_linear)");

    //ConstantBuffer bandwidth from random access
    if(vectorSize != 1)
    {
        kernel[3] = clCreateKernel(program,"constantBandwidth_random1",&status);
        CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(constantBandwidth_random1)");
    }
    else
    {
        kernel[3] = clCreateKernel(program,"constantBandwidth_random2",&status);
        CHECK_OPENCL_ERROR(status, "clCreateKernel failed.(constantBandwidth_random2)");
    }

    return SDK_SUCCESS;
}

int
ConstantBandwidth::bandwidth(cl_kernel &kernel)
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

    // Set appropriate arguments to the kernel
    int argIndex = 0;

    if(dynamiArgFlag)
    { 
		if (is64Bit())
		{
			cl_ulong index = 0;
			status = clSetKernelArg(kernel,
									argIndex++,
									sizeof(cl_long),
									(void *)&index);
		}
		else
		{
			cl_uint index = 0;
			status = clSetKernelArg(kernel,
								argIndex++,
								sizeof(cl_uint),
								(void *)&index);
		}

        CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(index)");
    }


    status = clSetKernelArg(kernel,
                            argIndex++,
                            sizeof(cl_mem),
                            (void *)&constantBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(constantBuffer)");


    status = clSetKernelArg(kernel,
                            argIndex++,
                            sizeof(cl_mem),
                            (void *)&outputBuffer);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed.(outputBuffer)");




    double sec = 0;
    int iter = iterations;
    if(sampleArgs->verify)
    {
        iter = 1;
    }


    // Run the kernel for a number of iterations
    for(int i = 0; i < iter; i++)
    {
        //Enqueue a kernel run call
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
        CHECK_OPENCL_ERROR(status, "clRelease Event Failed");
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

    std::cout << ": " << perf << " GB/s" << std::endl;

    // Enqueue readBuffer

    status = clEnqueueReadBuffer(commandQueue,
                                 outputBuffer,
                                 CL_TRUE,
                                 0,
                                 length * sizeof(cl_float) * vectorSize,
                                 output,
                                 0,
                                 NULL,
                                 0);
    CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer failed.");

    return SDK_SUCCESS;
}

int
ConstantBandwidth::runCLKernels(void)
{
    int status;

    if(vec3 == true)
    {
        std::cout << "\nAccessType\t: single(static index)\nVectorElements\t: 3" <<
                  std::endl;
    }
    else
    {
        std::cout << "\nAccessType\t: single(static index)\nVectorElements\t: " <<
                  vectorSize << std::endl;
    }

    std::cout << "Bandwidth\t";

    // Measure bandwidth of single reads(static index) from constant buffer
    status = bandwidth(kernel[0]);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(sampleArgs->verify)
    {
        int size = length * vectorSize * sizeof(cl_float);
        // Verify result for single access
        memset(verificationOutput, 0, size);

        int index = 0;
        for(int i = 0; i < (int)length; i++)
        {
            for(int j = 0; j < NUM_READS; j++)
            {
                for(int k = 0; k < vectorSize; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[(index + j) * vectorSize + k];
                }
            }
        }

        if(!memcmp(output, verificationOutput, size))
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }
    }

    if(vec3 == true)
    {
        std::cout << "\nAccessType\t: single(dynamic index)\nVectorElements\t: 3" <<
                  std::endl;
    }
    else
    {
        std::cout << "\nAccessType\t: single(dynamic index)\nVectorElements\t: " <<
                  vectorSize << std::endl;
    }

    std::cout << "Bandwidth\t";
    dynamiArgFlag = true;

    // Measure bandwidth of single(dynamic index) reads from constant buffer
    status = bandwidth(kernel[1]);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(sampleArgs->verify)
    {
        int size = length * vectorSize * sizeof(cl_float);
        // Verify result for single access
        if(!memcmp(output, verificationOutput, size))
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }
    }

    if(vec3 == true)
    {
        std::cout << "\nAccessType\t: linear\nVectorElements\t: 3" << std::endl;
    }
    else
    {
        std::cout << "\nAccessType\t: linear\nVectorElements\t: " << vectorSize <<
                  std::endl;
    }

    std::cout << "Bandwidth\t";

    dynamiArgFlag = false;
    // Measure bandwidth of linear reads from constant buffer
    status = bandwidth(kernel[2]);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(sampleArgs->verify)
    {
        int size = length * vectorSize * sizeof(cl_float);
        memset(verificationOutput, 0, size);
        // Verify result for Linear access
        for(int i = 0; i < (int)length; i++)
        {
            int index = i % 64;
            for(int j = 0; j < NUM_READS; j++)
            {
                for(int k = 0; k < vectorSize; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[(index + j) * vectorSize + k];
                }
            }
        }

        if(!memcmp(output, verificationOutput, size))
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }
    }

    std::cout << "\nAccessType\t: random\nVectorElements\t: " << vectorSize <<
              std::endl;
    std::cout << "Bandwidth\t";

    // Measure bandwidth of linear reads from constant buffer

    status = bandwidth(kernel[3]);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(sampleArgs->verify)
    {
        int size = length  * sizeof(cl_float) *vectorSize;

        memset(verificationOutput, 0, size);

        // Verify result for Random access
        for(int i = 0; i < (int)length; i++)
        {
            cl_uint index = i % GROUP_SIZE;
            //cl_uint index = 0;
            for(int j = 0; j < NUM_READS; j++)
            {

                for(int k =0 ; k<vectorSize ; k++)
                {
                    verificationOutput[i * vectorSize + k] += input[index * vectorSize + k];

                }
                index = (cl_uint)(verificationOutput[i * vectorSize + 0]) % 320;


            }
        }

		float verSumHost = 0.0f;
		float verSumDev = 0.0f;
		for(int i = 0; i < (int)length; i++)
        {
			for(int k =0 ; k<vectorSize ; k++)
            {
               verSumHost += verificationOutput[i * vectorSize + k];
			   verSumDev += output[i * vectorSize + k];
            }
		}

        if (compare(&verSumHost, &verSumDev, 1))
        {
            std::cout << "Passed!\n" << std::endl;
            return SDK_SUCCESS;
        }
        else
        {
            std::cout << "Failed!\n" << std::endl;
            return SDK_FAILURE;
        }
    }

    return SDK_SUCCESS;
}


int
ConstantBandwidth::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    Option* num_iterations = new Option;
    CHECK_ALLOCATION(num_iterations, "Memory Allocation failed (num_iterations)");

    num_iterations->_sVersion = "i";
    num_iterations->_lVersion = "iterations";
    num_iterations->_description = "Number of iterations for kernel execution";
    num_iterations->_type = CA_ARG_INT;
    num_iterations->_value = &iterations;

    sampleArgs->AddOption(num_iterations);
    delete num_iterations;

    Option* num_arguments = new Option;
    CHECK_ALLOCATION(num_arguments, "Memory Allocation failed (num_arguments)");

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
ConstantBandwidth::setup()
{
    if(iterations < 1)
    {
        std::cout<<"Error, iterations cannot be 0 or negative. Exiting..\n";
        return SDK_FAILURE;
    }
    if(setupConstantBandwidth() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    int status = setupCL();
    if(status != SDK_SUCCESS)
    {
        if(status == SDK_EXPECTED_FAILURE)
        {
            return SDK_EXPECTED_FAILURE;
        }

        return SDK_FAILURE;
    }

    sampleTimer->stopTimer(timer);
    setupTime = (cl_double)sampleTimer->readTimer(timer);

    return SDK_SUCCESS;
}


int
ConstantBandwidth::run()
{
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    // Arguments are set and execution call is enqueued on command buffer
    if(runCLKernels() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    sampleTimer->stopTimer(timer);
    totalKernelTime = (double)(sampleTimer->readTimer(timer));

    return SDK_SUCCESS;
}

int
ConstantBandwidth::verifyResults()
{
    return SDK_SUCCESS;
}


int
ConstantBandwidth::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

    for(int i = 0; i < NUM_KERNELS; i++)
    {
        status = clReleaseKernel(kernel[i]);
        CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");
    }

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.");

    status = clReleaseMemObject(constantBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseMemObject(outputBuffer);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.");

    // release program resources (input memory etc.)
    FREE(input);

    FREE(output);

    FREE(verificationOutput);

    // release device list
    FREE(devices);

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    cl_int status = 0;
    ConstantBandwidth clConstantBandwidth;

    if(clConstantBandwidth.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clConstantBandwidth.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clConstantBandwidth.sampleArgs->isDumpBinaryEnabled())
    {
        return clConstantBandwidth.genBinaryImage();
    }

    status = clConstantBandwidth.setup();
    if(status != SDK_SUCCESS)
    {
        if(status == SDK_EXPECTED_FAILURE)
        {
            return SDK_SUCCESS;
        }

        return SDK_FAILURE;
    }

    if(clConstantBandwidth.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clConstantBandwidth.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clConstantBandwidth.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}
