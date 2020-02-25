/**********************************************************************
Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.

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


#include "SimpleGenericAddressSpace.hpp"
#include <cmath>

int
SimpleGenericAddressSpace::roundUp(int value, unsigned int multiple)
{
	int remainder = value%(int)multiple;
	if(remainder)
	{
		value += ((int)multiple-remainder);
	}
	return value;
}

int
SimpleGenericAddressSpace::readInputImage(std::string inputImageName)
{
	// load input bitmap image
    inputBitmap.load(inputImageName.c_str());

    // error if image did not load
    if(!inputBitmap.isLoaded())
    {
        std::cout << "Failed to load input image!";
        return SDK_FAILURE;
    }

    // get width and height of input image
    height = inputBitmap.getHeight();
    width = inputBitmap.getWidth();

    // allocate memory for input image data to host
    inputImageData2D = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(inputImageData2D,"Failed to allocate memory! (inputImageData2D)");

	// get the pointer to pixel data
    pixelData = inputBitmap.getPixels();
    if(pixelData == NULL)
    {
        std::cout << "Failed to read pixel Data!";
        return SDK_FAILURE;
    }

    // Copy pixel data into inputImageData2D
    memcpy(inputImageData2D, pixelData, width * height * pixelSize);

	// allocate memory for output image data for convolutionGlobalKernel to host
    outputImageData2DGlobal = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(outputImageData2DGlobal,"Failed to allocate memory! (outputImageData2DGlobal)");
	memset(outputImageData2DGlobal, 0, width * height * pixelSize);
	
	// allocate memory for output image data for sepiaToningLocalKernel to host
    outputImageData2DLocal = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(outputImageData2DLocal,"Failed to allocate memory! (outputImageData2DLocal)");
	memset(outputImageData2DLocal, 0, width * height * pixelSize);

	// allocate memory for verification output for 1st kernel
    verificationConvolutionOutput = (cl_uchar*)malloc(width * height * pixelSize);
    CHECK_ALLOCATION(verificationConvolutionOutput,"Failed to allocate memory! (verificationConvolutionOutput)");
    memset(verificationConvolutionOutput, 0, width * height * pixelSize);

	// allocate memory for verification output for 2nd kernel
    verificationSepiaToningOutput = (cl_uchar*)malloc(width * height * pixelSize);
    CHECK_ALLOCATION(verificationSepiaToningOutput,"Failed to allocate memory! (verificationSepiaToningOutput)");
    memset(verificationSepiaToningOutput, 0, width * height * pixelSize);

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::writeOutputImage(std::string outputImageName, cl_uchar4 *outputImageData)
{
    // copy output image data back to original pixel data
    memcpy(pixelData, outputImageData, width * height * pixelSize);

    // write the output bmp file
    if(!inputBitmap.write(outputImageName.c_str()))
    {
        std::cout << "Failed to write output image!";
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}


int
SimpleGenericAddressSpace::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("SimpleGenericAddressSpace_Kernels.cl");
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
SimpleGenericAddressSpace::setupCL()
{
    cl_int status = CL_SUCCESS;
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
    CHECK_OPENCL_ERROR(status, "clCreateContextFromType failed.");

    // getting device on which to run the sample
    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");

    status = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
    CHECK_OPENCL_ERROR(status, "deviceInfo.setDeviceInfo failed");

    if(!deviceInfo.imageSupport)
    {
        OPENCL_EXPECTED_ERROR(" Expected Error: Device does not support Images");
    }

	// Check of OPENCL_C_VERSION if device version is 2.0	
    int majorRev, minorRev;
    if (sscanf(deviceInfo.deviceVersion, "OpenCL %d.%d", &majorRev, &minorRev) == 2) 
    {
      if (majorRev < 2) {
	    OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
      }
    }

	// Create command queue
    cl_queue_properties *props = NULL;
    commandQueue = clCreateCommandQueueWithProperties(
                       context,
                       devices[sampleArgs->deviceId],
                       props,
                       &status);
    CHECK_OPENCL_ERROR(status,"clCreateCommandQueueWithProperties failed.");

	// Initialize CL input buffer for image data
	deviceInputBuffer = clCreateBuffer(context,
						CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
						width * height * pixelSize,  
						inputImageData2D,
						&status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(deviceInputBuffer).");

	// Create CL filter Buffer
	devicefilterBuffer = clCreateBuffer(context,
							CL_MEM_READ_ONLY,
							sizeof(float) * filterWidth * filterHeight,
							NULL,
							&status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(devicefilterBuffer).");
	
	// Create CL output Buffer for output image
	deviceOutputBuffer = clCreateBuffer(context,
							CL_MEM_WRITE_ONLY,
							width * height * pixelSize,
							NULL,
							&status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(deviceOutputBuffer).");

    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("SimpleGenericAddressSpace_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
	buildData.flagsStr = std::string("-cl-std=CL2.0");  /*check it*/
    //buildData.flagsStr = std::string("");
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

	// get a kernel object handle for a convolution2DUsingGlobal kernel
    convolutionGlobalKernel = clCreateKernel(program, "convolution2DUsingGlobal", &status);
    CHECK_OPENCL_ERROR(status,"clCreateKernel failed.(convolutionGlobalKernel)");

    // get a kernel object handle for a sepiaToning2DUsingLocal kernel
    sepiaToningLocalKernel = clCreateKernel(program, "sepiaToning2DUsingLocal", &status);
    CHECK_OPENCL_ERROR(status,"clCreateKernel failed.(sepiaToningLocalKernel)");

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::runConvolutionGlobalKernels()
{
    cl_int status;

	// Set appropriate arguments to the convolutionGlobalKernel kernel
    // input data buffer 
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 0,
                 sizeof(cl_mem),
                 &deviceInputBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceInputBuffer)");

	// output data buffer 
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 1,
                 sizeof(cl_mem),
                 &deviceOutputBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceOutputBuffer)");

	// image height
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 2,
                 sizeof(int),
                 &height);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (height)");

	// image width
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 3,
                 sizeof(int),
                 &width);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (width)");

	// filter buffer
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 4,
                 sizeof(cl_mem),
				 &devicefilterBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (devicefilterBuffer)");

	// filter height
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 5,
                 sizeof(int),
                 &filterHeight);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (filterHeight)");

	// filter width
    status = clSetKernelArg(
                 convolutionGlobalKernel,
                 6,
                 sizeof(int),
                 &filterWidth);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (filterWidth)");
	
	// calculate the number of padding pixels
	filterDim.s[0] = filterWidth;  filterDim.s[1] = filterHeight;
	filterRadius.s[0] = filterDim.s[0]/2; filterRadius.s[1] = filterDim.s[1]/2;
	padding.s[0] = filterRadius.s[0]*2; padding.s[1] = filterRadius.s[1]*2;
	
	// set local work-group size (check the group size retuned by kernel)
	localSize[0] = blockSizeX; 
	localSize[1] = blockSizeY;

	// set global work-group size, padding work-items do not need to be considered
	globalSize[0] = roundUp(width-padding.s[0], (unsigned int)blockSizeX);
	globalSize[1] = roundUp(height-padding.s[1], (unsigned int)blockSizeY);

	cl_event ndrEvt;
	// Enqueue a convolutionGlobalKernel run call
	status = clEnqueueNDRangeKernel(
                 commandQueue,
                 convolutionGlobalKernel,
                 2,
                 NULL,
				 globalSize,
				 localSize,
                 0,
                 NULL,
				 &ndrEvt);
    CHECK_OPENCL_ERROR(status,"clEnqueueNDRangeKernel(convolutionGlobalKernel)failed.");

	status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.");

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::runSepiaToningLocalKernels()
{

	int status;

	localWidth = blockSizeX+padding.s[0]; 
	localHeight = blockSizeY+padding.s[1];

    // input data buffer 
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 0,
                 sizeof(cl_mem),
                 &deviceInputBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceInputBuffer)");

	// output data buffer 
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 1,
                 sizeof(cl_mem),
                 &deviceOutputBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceOutputBuffer)");

	// image height
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 2,
                 sizeof(int),
                 &height);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (height)");

	// image width
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 3,
                 sizeof(int),
                 &width);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (width)");

	// filter buffer
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 4,
                 sizeof(cl_mem),
				 &devicefilterBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (devicefilterBuffer)");

	// filter height
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 5,
                 sizeof(int),
                 &filterHeight);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (filterHeight)");

	// filter width
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 6,
                 sizeof(int),
                 &filterWidth);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (filterWidth)");
	
	// shared memory buffer 
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 7,
                 localHeight*localWidth*pixelSize,
                 NULL);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (localImage)");
	
	// shared memory buffer height
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 8,
                 sizeof(int),
                 &localHeight);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (localHeight)");
	
	// shared memory buffer width
    status = clSetKernelArg(
                 sepiaToningLocalKernel,
                 9,
                 sizeof(int),
                 &localWidth);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (localWidth)");
	
	// Enqueue a sepiaToningLocalKernel run call
	cl_event ndrEvt;
	status = clEnqueueNDRangeKernel(
                 commandQueue,
                 sepiaToningLocalKernel,
                 2,
                 NULL,
				 globalSize,
				 localSize,
                 0,
                 NULL,
				 &ndrEvt);
    CHECK_OPENCL_ERROR(status,"clEnqueueNDRangeKernel(sepiaToningLocalKernel)failed.");

	status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status, "clFlush failed.");

    status = waitForEventAndRelease(&ndrEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");

	return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::initialize()
{

    // Call base class Initialize to get default configuration
    if (sampleArgs->initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    Option* iteration_option = new Option;
    CHECK_ALLOCATION(iteration_option,
                     "Memory Allocation error. (iteration_option)");

    iteration_option->_sVersion = "i";
    iteration_option->_lVersion = "iterations";
    iteration_option->_description = "Number of iterations to execute kernel";
    iteration_option->_type = CA_ARG_INT;
    iteration_option->_value = &iterations;

    sampleArgs->AddOption(iteration_option);

    delete iteration_option;

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::setup()
{
    int status = 0;
    // Allocate host memory and read input image
	std::string filePath = getPath() + std::string(INPUT_IMAGE);
    status = readInputImage(filePath);
    CHECK_ERROR(status, SDK_SUCCESS, "Read Input Image failed");

    // create and initialize timers
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    status = setupCL();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    sampleTimer->stopTimer(timer);
    // Compute setup time
    setupTime = (double)(sampleTimer->readTimer(timer));

    return SDK_SUCCESS;

}

int
SimpleGenericAddressSpace::run()
{
    int status;

	// initialize filter buffer
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 devicefilterBuffer,
                 CL_TRUE,
                 0,
                 sizeof(float) * filterWidth * filterHeight,
                 filterData,
                 0,
                 NULL,
                 NULL);
    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (devicefilterBuffer)");
	
	// Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
		// initialize output buffer for ConvolutionGlobalKernels
		status = clEnqueueWriteBuffer(
                 commandQueue,
                 deviceOutputBuffer,
                 CL_TRUE,
                 0,
                 width * height * pixelSize,
                 outputImageData2DGlobal,
                 0,
                 NULL,
                 NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceOutputBuffer)");
		
		// Arguments are set and execution call is enqueued on command buffer
        if(runConvolutionGlobalKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

		// Enqueue readBuffer for ConvolutionGlobalKernels
		status = clEnqueueReadBuffer(
					commandQueue,
					deviceOutputBuffer,
					CL_TRUE,
					0,
					width*height*pixelSize,
					outputImageData2DGlobal,
					0,
					NULL,
					NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer(outputImageData2DGlobal) Failed");
	
		// initialize output buffer for SepiaToningLocalKernels
		status = clEnqueueWriteBuffer(
                 commandQueue,
                 deviceOutputBuffer,
                 CL_TRUE,
                 0,
                 width * height * pixelSize,
                 outputImageData2DLocal,
                 0,
                 NULL,
                 NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceOutputBuffer)");
	
		// Arguments are set and execution call is enqueued on command buffer
        if(runSepiaToningLocalKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
		
		// Enqueue readBuffer for SepiaToningLocalKernels
		status = clEnqueueReadBuffer(
					commandQueue,
					deviceOutputBuffer,
					CL_TRUE,
					0,
					width*height*pixelSize,
					outputImageData2DLocal,
					0,
					NULL,
					NULL);
		CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer(outputImageData2DLocal) Failed");
    }
 
	std::cout << "Executing kernel for " << iterations <<
              " iterations" <<std::endl;
    std::cout << "-------------------------------------------" << std::endl;

	// initialize output buffer ConvolutionGlobalKernels
    status = clEnqueueWriteBuffer(
                 commandQueue,
                 deviceOutputBuffer,
                 CL_TRUE,
                 0,
                 width * height * pixelSize,
                 outputImageData2DGlobal,
                 0,
                 NULL,
                 NULL);
    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceOutputBuffer)");
	
	// create and initialize timers
    int timer = sampleTimer->createTimer();
	sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	// Running ConvolutionGlobal kernel
	for(int i = 0; i < iterations; i++)
    {
        // Set kernel arguments and run kernel
        status = runConvolutionGlobalKernels();
        CHECK_ERROR(status, SDK_SUCCESS, "OpenCL run Kernel failed");
    }

    sampleTimer->stopTimer(timer);
    ConvolutionGlobalKernelTime = (double)(sampleTimer->readTimer(timer)) / iterations;

	// Enqueue readBuffer
	status = clEnqueueReadBuffer(
					commandQueue,
					deviceOutputBuffer,
					CL_TRUE,
					0,
					width*height*pixelSize,
					outputImageData2DGlobal,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer(outputImageData2DGlobal) Failed");
	
	// write the global convolution output image to bitmap file
    status = writeOutputImage(OUTPUT_IMAGE_GLOBAL, outputImageData2DGlobal);
    CHECK_ERROR(status, SDK_SUCCESS, "write Global Convolution Output Image Failed");
	
	// initialize output buffer for SepiaToningLocalKernels
	status = clEnqueueWriteBuffer(
                 commandQueue,
                 deviceOutputBuffer,
                 CL_TRUE,
                 0,
                 width * height * pixelSize,
                 outputImageData2DLocal,
                 0,
                 NULL,
                 NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceOutputBuffer)");
		
	sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	// Running SepiaToningLocal kernel
	for(int i = 0; i < iterations; i++)
    {
        // Set kernel arguments and run kernel
        status = runSepiaToningLocalKernels();
        CHECK_ERROR(status, SDK_SUCCESS, "OpenCL run Kernel failed");
    }

    sampleTimer->stopTimer(timer);
    sepiaToningLocalKernelTime = (double)(sampleTimer->readTimer(timer)) / iterations;

	// Enqueue readBuffer for SepiaToningLocalKernels
	status = clEnqueueReadBuffer(
					commandQueue,
					deviceOutputBuffer,
					CL_TRUE,
					0,
					width*height*pixelSize,
					outputImageData2DLocal,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer(outputImageData2DLocal) Failed");

	// write the local SepiaToning output image to bitmap file
    status = writeOutputImage(OUTPUT_IMAGE_LOCAL, outputImageData2DLocal);
	CHECK_ERROR(status, SDK_SUCCESS, "write Local SepiaToning Output Image Failed");

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

	status = clReleaseKernel(convolutionGlobalKernel);
    CHECK_OPENCL_ERROR(status,"clReleaseKernel failed.(convolutionGlobalKernel)");

    status = clReleaseKernel(sepiaToningLocalKernel);
    CHECK_OPENCL_ERROR(status,"clReleaseKernel failed.(sepiaToningLocalKernel)");

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status,"clReleaseProgram failed.(program)");

	status = clReleaseMemObject(deviceInputBuffer);
    CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(deviceInputBuffer)");

	status = clReleaseMemObject(deviceOutputBuffer);
    CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(deviceOutputBuffer)");
	
	status = clReleaseMemObject(devicefilterBuffer);
    CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(devicefilterBuffer)");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status,"clReleaseCommandQueue failed.(commandQueue)");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status,"clReleaseContext failed.(context)");

    FREE(inputImageData2D);
    FREE(outputImageData2DGlobal);
	FREE(outputImageData2DLocal);
	FREE(verificationConvolutionOutput);
	FREE(verificationSepiaToningOutput);
    FREE(devices);

    return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::CPUReference()
{
	int red, green, blue;
	int id;
	double grayscale;
	double depth;
	
    for(int i = 0; i < height-padding.s[1]; ++i)
    {
        for(int j = 0; j < width-padding.s[0]; ++j)
        {
			cl_float4 sum = {0.0f, 0.0f, 0.0f, 0.0f};
			for(int m = 0; m < filterDim.s[1]; m++)
			{
				for(int n = 0; n < filterDim.s[0]; n++)
				{
					// copy uchar4 data to float4
					sum.s[0] += (cl_float)(inputImageData2D[(i+m)*width+(j+n)].s[0]) * (filterData[m*filterDim.s[0]+n]);
					sum.s[1] += (cl_float)(inputImageData2D[(i+m)*width+(j+n)].s[1]) * (filterData[m*filterDim.s[0]+n]);
					sum.s[2] += (cl_float)(inputImageData2D[(i+m)*width+(j+n)].s[2]) * (filterData[m*filterDim.s[0]+n]);
					sum.s[3] += (cl_float)(inputImageData2D[(i+m)*width+(j+n)].s[3]) * (filterData[m*filterDim.s[0]+n]);
				}
			}

			// calculating cpu reference for Ist kernel
			verificationConvolutionOutput[(((i+filterRadius.s[0])*width + (j+filterRadius.s[1]))*4) + 0] = (cl_uchar)((sum.s[0] < 0) ? 0 : ((sum.s[0] > 255.0) ? 255 : sum.s[0]) );
			verificationConvolutionOutput[(((i + filterRadius.s[0])*width + (j + filterRadius.s[1])) * 4) + 1] = (cl_uchar)((sum.s[1] < 0) ? 0 : ((sum.s[1] > 255.0) ? 255 : sum.s[1]));
			verificationConvolutionOutput[(((i + filterRadius.s[0])*width + (j + filterRadius.s[1])) * 4) + 2] = (cl_uchar)((sum.s[2] < 0) ? 0 : ((sum.s[2] > 255.0) ? 255 : sum.s[2]));
			verificationConvolutionOutput[(((i + filterRadius.s[0])*width + (j + filterRadius.s[1])) * 4) + 3] = (cl_uchar)((sum.s[3] < 0) ? 0 : ((sum.s[3] > 255.0) ? 255 : sum.s[3]));

			// calculating cpu reference for 2 nd kernel
			id = (((i+filterRadius.s[0])*width + (j+filterRadius.s[1]))*4);
			red = verificationConvolutionOutput[id + 0];
			green = verificationConvolutionOutput[id + 1];
			blue = verificationConvolutionOutput[id + 2];

			grayscale = (0.3 * red + 0.59 * green + 0.11 * blue);
			depth = 1.8;

			red = (int)(grayscale + depth * 56.6 );
			if (red > 255)
		    {
			    red = 255;
		    }

		    green = (int)(grayscale + depth * 33.3 );
		    if (green > 255)
		    {
		        green = 255;
		    }

		    blue = (int)(grayscale + depth * 10.1);
			if (blue > 255)
			{
			    blue = 255;
			}
			verificationSepiaToningOutput[id + 0] = red;
			verificationSepiaToningOutput[id + 1] = green;
			verificationSepiaToningOutput[id + 2] = blue;
        }
    }

	return SDK_SUCCESS;
}

int
SimpleGenericAddressSpace::verifyResults()
{
    if(sampleArgs->verify)
    {
		/**
		* Reference implementation on host device
		*/
		CPUReference();

		float *outputDevice = new float[width * height * pixelSize];
        CHECK_ALLOCATION(outputDevice,"Failed to allocate host memory! (outputDevice)");
	
		float *outputReference = new float[width * height * pixelSize];
		CHECK_ALLOCATION(outputReference, "Failed to allocate host memory! (outputReference)");

		std::cout << "Verifying convolutionGlobal Kernel result - ";

		for(int i = 0; i < (int)(width * height); i++)
		{
			// copy uchar data to float array from verificationConvolutionOutput
			outputReference[i * 4 + 0] = verificationConvolutionOutput[i * 4 + 0];
			outputReference[i * 4 + 1] = verificationConvolutionOutput[i * 4 + 1];
			outputReference[i * 4 + 2] = verificationConvolutionOutput[i * 4 + 2];
			outputReference[i * 4 + 3] = verificationConvolutionOutput[i * 4 + 3];

			// copy uchar data to float array from global kernel
			outputDevice[i * 4 + 0] = outputImageData2DGlobal[i].s[0];
			outputDevice[i * 4 + 1] = outputImageData2DGlobal[i].s[1];
			outputDevice[i * 4 + 2] = outputImageData2DGlobal[i].s[2];
			outputDevice[i * 4 + 3] = outputImageData2DGlobal[i].s[3];
		}		

		// compare the results and see if they match
        if(compare(outputDevice, outputReference, width * height * 4))
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
			delete[] outputDevice;
			delete[] outputReference;
            std::cout << "Failed\n" << std::endl;
            return SDK_FAILURE;
        }

        std::cout << "Verifying sepiaToningLocal Kernel result - ";

		memset(outputDevice, 0, width*height*4);
		memset(outputReference, 0, width*height*4);
        for(int i = 0; i < (int)(width * height); i++)
        {
			// copy uchar data to float array from verificationSepiaToningOutput
			outputReference[i * 4 + 0] = verificationSepiaToningOutput[i * 4 + 0];
			outputReference[i * 4 + 1] = verificationSepiaToningOutput[i * 4 + 1];
			outputReference[i * 4 + 2] = verificationSepiaToningOutput[i * 4 + 2];
			outputReference[i * 4 + 3] = verificationSepiaToningOutput[i * 4 + 3];

			// copy uchar data to float array from global kernel
            outputDevice[i * 4 + 0] = outputImageData2DLocal[i].s[0];
            outputDevice[i * 4 + 1] = outputImageData2DLocal[i].s[1];
            outputDevice[i * 4 + 2] = outputImageData2DLocal[i].s[2];
            outputDevice[i * 4 + 3] = outputImageData2DLocal[i].s[3];
        }

        // compare the results and see if they match
        if(compare(outputDevice, outputReference, width * height * 4))
        {
			delete[] outputDevice;
			delete[] outputReference;
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
			delete[] outputDevice;
			delete[] outputReference;
            std::cout << "Failed\n" << std::endl;
            return SDK_FAILURE;
        }
    }
    return SDK_SUCCESS;
}


void
SimpleGenericAddressSpace::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] =
        {
            "Width",
            "Height",
            "TotalTime(sec)",
            "kernelTime(sec)"
        };
        std::string stats[4];

		std::cout << "\nconvolutionGlobalKernel Timing Measurement!" << std::endl;
        sampleTimer->totalTime = setupTime + ConvolutionGlobalKernelTime;
        stats[0] = toString(width, std::dec);
        stats[1] = toString(height, std::dec);
        stats[2] = toString(sampleTimer->totalTime, std::dec);
        stats[3] = toString(ConvolutionGlobalKernelTime, std::dec);
        printStatistics(strArray, stats, 4);

		std::cout << "\n\nsepiaToningLocalKernel Timing Measurement. Note: this kernel time includes (Convolution operation using Local Address Space + SepiaToning filtering operation)!" << std::endl;
		sampleTimer->totalTime = setupTime + sepiaToningLocalKernelTime;
		stats[0] = toString(width, std::dec);
        stats[1] = toString(height, std::dec);
        stats[2] = toString(sampleTimer->totalTime, std::dec);
        stats[3] = toString(sepiaToningLocalKernelTime, std::dec);
        printStatistics(strArray, stats, 4);
    }
}


int
main(int argc, char * argv[])
{
    int status = 0;
    SimpleGenericAddressSpace clSimpleGenericAddressSpace;

    if (clSimpleGenericAddressSpace.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clSimpleGenericAddressSpace.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clSimpleGenericAddressSpace.sampleArgs->isDumpBinaryEnabled())
    {
        return clSimpleGenericAddressSpace.genBinaryImage();
    }

    status = clSimpleGenericAddressSpace.setup();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    if (clSimpleGenericAddressSpace.run() !=SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clSimpleGenericAddressSpace.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clSimpleGenericAddressSpace.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clSimpleGenericAddressSpace.printStats();
    return SDK_SUCCESS;
}
