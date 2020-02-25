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


#include "ImageBinarization.hpp"
#include <cmath>

int
ImageBinarization::calculateThresholdValueOnHost()
{
	double prob[BIN_SIZE], omega[BIN_SIZE];   /* Prob of graylevels */
	double mu[BIN_SIZE];                      /* Mean value for separation */
	double max_sigma, sigma[BIN_SIZE];        /* Inter-class variance */

	// Calculation of probability density
	for (int i = 0; i < binSize; ++i)
	{
		prob[i] = (double)deviceBin[i] / (width * height);
	}

	// omega and mu generation
	omega[0] = prob[0];
	mu[0] = 0.0;
	for (int i = 1; i < binSize; ++i)
	{
		omega[i] = omega[i - 1] + prob[i];
		mu[i] = mu[i - 1] + i*prob[i];
	}

	// sigma maximization, sigma stands for inter-class variance and determines optimal threshold value
	threshold = 0;
	max_sigma = 0.0;
	for (int i = 0; i < binSize; ++i)
	{
		if (omega[i] != 0.0 && omega[i] != 1.0)
		{
			sigma[i] = pow((mu[BIN_SIZE - 1]*omega[i] - mu[i]), 2) / (omega[i] * (1.0 - omega[i]));
		}
		else
			sigma[i] = 0.0;

		if (sigma[i] > max_sigma)
		{
			max_sigma = sigma[i];
			threshold = i;
		}
	}

	return SDK_SUCCESS;
}

int
ImageBinarization::readInputImage(std::string inputImageName)
{

    // load input bitmap image
    inputBitmap.load(inputImageName.c_str());

    // error if image did not load
    if(!inputBitmap.isLoaded())
    {
        error("Failed to load input image!");
        return SDK_FAILURE;
    }

    // get width and height of input image
    height = inputBitmap.getHeight();
    width = inputBitmap.getWidth();
	globalWorkSizeHist = (width/nPixelsPerThread)*height;
	subHistgCnt = (cl_int)(globalWorkSizeHist/localThreadsHistogram); 

	// allocate memory for input image data (only Gray component) to host
    inputImageDataGrayComponent = (cl_uchar*)malloc(width * height * sizeof(cl_uchar));
    CHECK_ALLOCATION(inputImageDataGrayComponent,"Failed to allocate memory! (inputImageDataGrayComponent)");

    // get the pointer to pixel data
    pixelData = inputBitmap.getPixels();
    CHECK_ALLOCATION(pixelData,"Failed to read pixel Data!");

	// calculating Y component from pixel data, using RGB to YUV conversion
	unsigned int R, G, B;
	for(int i = 0; i <  width*height; i++)
	{
		R = pixelData[i].x;
		G = pixelData[i].y;
		B = pixelData[i].z;
		inputImageDataGrayComponent[i] = ( (  66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
	}

	// Allocate host memory for histgram bins
	hostBin = (cl_uint*)malloc(binSize * sizeof(cl_uint));
	CHECK_ALLOCATION(hostBin, "Failed to allocate host memory. (hostBin)");
	memset(hostBin, 0, binSize * sizeof(cl_uint));

	// Allocate device memory for histogram bins
	deviceBin = (cl_uint*)malloc(binSize * sizeof(cl_uint));
	CHECK_ALLOCATION(deviceBin, "Failed to allocate host memory. (deviceBin)");
	memset(deviceBin, 0, binSize * sizeof(cl_uint));

	// Allocate intermittent device memory for sub-histogram bins
	intermittentdeviceBinResult = (cl_uint*)malloc(binSize * subHistgCnt * sizeof(cl_uint));
	CHECK_ALLOCATION(intermittentdeviceBinResult, "Failed to allocate host memory. (intermittentdeviceBinResult)");
	memset(intermittentdeviceBinResult, 0, binSize * subHistgCnt * sizeof(cl_uint));

	// allocate memory for output image data (only Gray compnent) to host
    outputImageDataGrayComponent = (cl_uchar*)malloc(width * height * sizeof(cl_uchar));
    CHECK_ALLOCATION(outputImageDataGrayComponent,"Failed to allocate memory! (outputImageDataGrayComponent)");
	memset((void *)outputImageDataGrayComponent, 0, width * height);

    // allocate memory for verification of ImageBinarization Kernel to host
    refOutputBinarizationData = (cl_uchar*)malloc(width * height * sizeof(cl_uchar));
    CHECK_ALLOCATION(refOutputBinarizationData,"refOutputBinarizationData heap allocation failed!");
    memset((void *)refOutputBinarizationData, 0, width * height);

    return SDK_SUCCESS;

}

int
ImageBinarization::writeOutputImage(std::string outputImageName)
{
    // copy output image data back to original pixel data
	memset(pixelData, 0xff, width * height * pixelSize);
	for(int i = 0; i <  width*height; i++)
	{	
		pixelData[i].x = outputImageDataGrayComponent[i];
		pixelData[i].y = outputImageDataGrayComponent[i];
		pixelData[i].z = outputImageDataGrayComponent[i];
	}

    // write the output bmp file
    if(!inputBitmap.write(outputImageName.c_str()))
    {
        std::cout << "Failed to write output image!";
        return SDK_FAILURE;
    }
    return SDK_SUCCESS;
}


int
ImageBinarization::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("ImageBinarization_Kernels.cl");
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
ImageBinarization::setupCL()
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

    // Create and initialize image objects
    cl_image_desc imageDesc;
    memset(&imageDesc, '\0', sizeof(cl_image_desc));
    imageDesc.image_type = CL_MEM_OBJECT_IMAGE2D;
    imageDesc.image_width = width;
    imageDesc.image_height = height;

    // Create 2D image, which will be used as input as well as output image
    image2D = clCreateImage(context,
                        CL_MEM_READ_WRITE,
                        &imageFormat,
                        &imageDesc,
                        NULL,
                        &status);
    CHECK_OPENCL_ERROR(status,"clCreateImage failed. (image2D)");

	// Initialize CL buffer for Histogram Kernel, it stores image data in 1D form
	deviceImageData1D = clCreateBuffer(context,
						CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
						sizeof(cl_uchar) * width * height,  
						(void *)inputImageDataGrayComponent,
						&status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(deviceImageData1D).");

	// Create CL Buffer for Histogram Kernel to store histogram bins
	deviceBinResultBuffer = clCreateBuffer(context,
							CL_MEM_WRITE_ONLY,
							sizeof(cl_uint) * binSize * subHistgCnt,
							NULL,
							&status);
	CHECK_OPENCL_ERROR(status, "clCreateBuffer failed.(deviceBinResultBuffer).");

    // create a CL program using the kernel source
    buildProgramData buildData;
    buildData.kernelName = std::string("ImageBinarization_Kernels.cl");
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

	// get a kernel object handle for a imageHistogram256_scalar kernel
    imageHistogramKernel = clCreateKernel(program, "imageHistogram256", &status);
    CHECK_OPENCL_ERROR(status,"clCreateKernel failed.(imageHistogramKernel)");

    // get a kernel object handle for a imageBinarization kernel
    imageBinarizationKernel = clCreateKernel(program, "imageBinarization", &status);
    CHECK_OPENCL_ERROR(status,"clCreateKernel failed.(imageBinarizationKernel)");

	// Check binarization local work group size against group size returned by kernel
    status = clGetKernelWorkGroupInfo(imageBinarizationKernel,
                                      devices[sampleArgs->deviceId],
                                      CL_KERNEL_WORK_GROUP_SIZE,
                                      sizeof(size_t),
                                      &binarizationkernelWorkGroupSize,
                                      0);
    CHECK_OPENCL_ERROR(status,"clGetKernelWorkGroupInfo  failed.");

    cl_uint temp = (cl_uint)binarizationkernelWorkGroupSize;

    if((blockSizeX * blockSizeY) > temp)
    {
        if(!sampleArgs->quiet)
        {
            std::cout << "Out of Resources!" << std::endl;
            std::cout << "Group Size specified : "
                      << blockSizeX * blockSizeY << std::endl;
            std::cout << "Max Group Size supported on the kernel(s) : "
                      << temp << std::endl;
            std::cout << "Falling back to " << temp << std::endl;
        }

        if(blockSizeX > temp)
        {
            blockSizeX = temp;
            blockSizeY = 1;
        }
    }
    return SDK_SUCCESS;
}

int
ImageBinarization::runCLKernels()
{
    cl_int status;

	// Set appropriate arguments to the ImageHistogram kernel
    // image1D data input buffer 
    status = clSetKernelArg(
                 imageHistogramKernel,
                 0,
                 sizeof(cl_mem),
                 (void *)&deviceImageData1D);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceImageData1D)");

	// shared array for storing subhistogram
	status = clSetKernelArg(
				imageHistogramKernel, 
				1, 
				binSize * nBanks * sizeof(cl_uint),
                NULL);
    CHECK_OPENCL_ERROR(status, "clSetKernelArg failed. (local memory)");

	// output histogram buffer
    status = clSetKernelArg(
                 imageHistogramKernel,
                 2,
                 sizeof(cl_mem),
                 (void *)&deviceBinResultBuffer);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (deviceBinResultBuffer)");

	// number of pixels per thread
    status = clSetKernelArg(
                 imageHistogramKernel,
                 3,
                 sizeof(cl_int),
				 &nPixelsPerThread);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (nPixelsPerThread)");
	
	// Enqueue a imageHistogramKernel run call
	status = clEnqueueNDRangeKernel(
                 commandQueue,
                 imageHistogramKernel,
                 1,
                 NULL,
				 &globalWorkSizeHist,
				 &localThreadsHistogram,
                 0,
                 NULL,
				 NULL);
    CHECK_OPENCL_ERROR(status,"clEnqueueNDRangeKernel(imageHistogramKernel)failed.");

	status = clEnqueueReadBuffer(
					commandQueue,
					deviceBinResultBuffer,
					CL_TRUE,
					0,
					sizeof(cl_uint)*binSize*subHistgCnt,
					intermittentdeviceBinResult,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer(intermittentdeviceBinResult) Failed");

	// Clear deviceBin array
	memset(deviceBin, 0, binSize * sizeof(cl_uint));

	// Calculate final histogram bin
	for (int i = 0; i < subHistgCnt; ++i)
	{
		for (int j = 0; j < binSize; ++j)
		{
			deviceBin[j] += intermittentdeviceBinResult[i * binSize + j];
		}
	}

	// Calculate threshold value usitng Otsu's Algorithm
	if (calculateThresholdValueOnHost())
	{
		return SDK_FAILURE;
	}

	// initialize image for ImageBinarization Kernel
    cl_event writeEvt;
    origin[0] = 0; origin[1] = 0; origin[2] = 0;
	region[0] = width; region[1] = height; region[2] = 1;

    status = clEnqueueWriteImage(
                 commandQueue,
                 image2D,
                 CL_FALSE,
                 origin,
                 region,
				 0,
                 0,
                 inputImageDataGrayComponent,
                 0,
                 NULL,
                 &writeEvt);
    CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceImageData1D)");

    status = waitForEventAndRelease(&writeEvt);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvt) Failed");

	// Set appropriate arguments to the imageBinarization kernel
    // buffer image
    status = clSetKernelArg(
                 imageBinarizationKernel,
                 0,
                 sizeof(cl_mem),
                 &image2D);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (image2D)");

	// dummy threshold
    status = clSetKernelArg(
                 imageBinarizationKernel,
                 1,
                 sizeof(int),
                 &threshold);
    CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (threshold)");

	size_t globalThreads2[] = {width, height};
	size_t localThreads2[] = {blockSizeX, blockSizeY};

	// Enqueue a imageBinarizationKernel run call
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 imageBinarizationKernel,
                 2,
                 NULL,
                 globalThreads2,
                 localThreads2,
                 0,
                 NULL,
                 0);
    CHECK_OPENCL_ERROR(status,"clEnqueueNDRangeKernel(imageBinarizationKernel) failed.");

	status = clFinish(commandQueue);
    CHECK_OPENCL_ERROR(status,"clFinish failed.");

    return SDK_SUCCESS;
}

int
ImageBinarization::initialize()
{

    // Call base class Initialize to get default configuration
    if (sampleArgs->initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    Option* iteration_option = new Option;
    CHECK_ALLOCATION(iteration_option, "Memory Allocation error. (iteration_option)");
    iteration_option->_sVersion = "i";
    iteration_option->_lVersion = "iterations";
    iteration_option->_description = "Number of iterations to execute kernel";
    iteration_option->_type = CA_ARG_INT;
    iteration_option->_value = &iterations;
    sampleArgs->AddOption(iteration_option);
    delete iteration_option;

	Option* global_work_size_hist = new Option;
    CHECK_ALLOCATION(global_work_size_hist, "Memory Allocation error. (global_work_size_hist)");
    global_work_size_hist->_sVersion = "g";
    global_work_size_hist->_lVersion = "globalWorkSizeHist";
	global_work_size_hist->_description = "Global Work Size for Histogram Kernel, it should be power of 2";
    global_work_size_hist->_type = CA_ARG_INT;
    global_work_size_hist->_value = &globalWorkSizeHist;
    sampleArgs->AddOption(global_work_size_hist);
    delete global_work_size_hist;

    return SDK_SUCCESS;
}

int
ImageBinarization::setup()
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
ImageBinarization::run()
{
    int status;

	// Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
        // Arguments are set and execution call is enqueued on command buffer
        if(runCLKernels() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }

    // create and initialize timers
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    std::cout << "Executing kernel for " << iterations <<
              " iterations" <<std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    for(int i = 0; i < iterations; i++)
    {
        // Set kernel arguments and run kernel
        status = runCLKernels();
        CHECK_ERROR(status, SDK_SUCCESS, "OpenCL run Kernel failed");
    }

    sampleTimer->stopTimer(timer);
    // Compute kernel time
    kernelTime = (double)(sampleTimer->readTimer(timer)) / iterations;

	// Set origin and region for ReadImage call
    origin[0] = 0; origin[1] = 0; origin[2] = 0;
	region[0] = width; region[1] = height; region[2] = 1;

    // Read output of 2D copy
    status = clEnqueueReadImage(commandQueue,
                                image2D,
                                1,
                                origin,
                                region,
                                0,
                                0,
                                outputImageDataGrayComponent,
                                0, 0, 0);
    CHECK_OPENCL_ERROR(status,"clEnqueueReadImage(outputImageDataGrayComponent) failed.");

    // Wait for the read buffer to finish execution
    status = clFinish(commandQueue);
    CHECK_OPENCL_ERROR(status,"clFinish failed.(commandQueue)");

    // write the output image to bitmap file
    status = writeOutputImage(OUTPUT_IMAGE);
    CHECK_ERROR(status, SDK_SUCCESS, "write Output Image Failed");

    return SDK_SUCCESS;
}

int
ImageBinarization::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

	if (imageHistogramKernel)
	{
		status = clReleaseKernel(imageHistogramKernel);
		CHECK_OPENCL_ERROR(status,"clReleaseKernel failed.(imageHistogramKernel)");
	}

	if (imageBinarizationKernel)
	{
		status = clReleaseKernel(imageBinarizationKernel);
		CHECK_OPENCL_ERROR(status,"clReleaseKernel failed.(imageBinarizationKernel)");
	}

	if (program)
	{
		status = clReleaseProgram(program);
		CHECK_OPENCL_ERROR(status,"clReleaseProgram failed.(program)");
	}

	if (image2D)
	{
		status = clReleaseMemObject(image2D);
		CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(image2D)");
	}

	if (deviceImageData1D)
	{
		status = clReleaseMemObject(deviceImageData1D);
		CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(deviceImageData1D)");
	}

	if (deviceBinResultBuffer)
	{
		status = clReleaseMemObject(deviceBinResultBuffer);
		CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(deviceBinResultBuffer)");
	}

	if (commandQueue)
	{
		status = clReleaseCommandQueue(commandQueue);
		CHECK_OPENCL_ERROR(status,"clReleaseCommandQueue failed.(commandQueue)");
	}

	if (context)
	{
		status = clReleaseContext(context);
		CHECK_OPENCL_ERROR(status,"clReleaseContext failed.(context)");
	}

    // release program resources (input memory etc.)
	FREE(inputImageDataGrayComponent);
	FREE(outputImageDataGrayComponent);
	FREE(refOutputBinarizationData);
	FREE(hostBin);
	FREE(intermittentdeviceBinResult);
	FREE(deviceBin);
    FREE(devices);

    return SDK_SUCCESS;
}

void
ImageBinarization::calculateHostBin()
{
    int red;
    for(int i = 0; i < width*height; i++)
    {
       red = inputImageDataGrayComponent[i];
       hostBin[red]++;
    }
}


void
ImageBinarization::ImageBinarizationCPUReference()
{
	int red;
	for(int i = 0; i < width*height; i++)
	{
		red = inputImageDataGrayComponent[i];
		if(red < threshold)
			red = 0;
		else
			red = 255;

		refOutputBinarizationData[i] = red;
	}
}


int
ImageBinarization::verifyResults()
{
    if(sampleArgs->verify)
    {
		/**
		* Reference implementation on host device
		* calculates the histogram bin on host
		*/
		calculateHostBin();

		// compare the results and see if they match
		bool result = true;
		for (int i = 0; i < binSize; ++i)
		{
			if (hostBin[i] != deviceBin[i])
			{
				result = false;
				break;
			}
		}

		if (!result)
		{
			std::cout << "Verifying ImageHistogram Kernel result - Failed\n" << std::endl;
			return SDK_FAILURE;
		}

        std::cout << "Verifying ImageBinarization Kernel result - ";
		// Calculate the reference output
		ImageBinarizationCPUReference();

        // compare the results and see if they match
        if(!memcmp(refOutputBinarizationData, outputImageDataGrayComponent, width * height))
        {
            std::cout << "Passed!\n" << std::endl;
        }
        else
        {
            std::cout << "Failed\n" << std::endl;
            return SDK_FAILURE;
        }
    }
    return SDK_SUCCESS;
}


void
ImageBinarization::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] =
        {
            "Width",
            "Height",
            "Time(sec)",
            "kernelTime(sec)"
        };
        std::string stats[4];

        sampleTimer->totalTime = setupTime + kernelTime;

        stats[0] = toString(width, std::dec);
        stats[1] = toString(height, std::dec);
        stats[2] = toString(sampleTimer->totalTime, std::dec);
        stats[3] = toString(kernelTime, std::dec);

        printStatistics(strArray, stats, 4);
    }
}


int
main(int argc, char * argv[])
{
    int status = 0;
    ImageBinarization clImageBinarization;

    if (clImageBinarization.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clImageBinarization.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clImageBinarization.sampleArgs->isDumpBinaryEnabled())
    {
        return clImageBinarization.genBinaryImage();
    }

    status = clImageBinarization.setup();
    if(status != SDK_SUCCESS)
    {
		clImageBinarization.cleanup();
        return status;
    }

    if (clImageBinarization.run() !=SDK_SUCCESS)
    {
		clImageBinarization.cleanup();
        return SDK_FAILURE;
    }

    if (clImageBinarization.verifyResults() != SDK_SUCCESS)
    {
		clImageBinarization.cleanup();
        return SDK_FAILURE;
    }

    if (clImageBinarization.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clImageBinarization.printStats();
    return SDK_SUCCESS;
}
