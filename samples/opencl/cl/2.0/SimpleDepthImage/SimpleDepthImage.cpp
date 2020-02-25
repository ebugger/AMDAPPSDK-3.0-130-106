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


#include "SimpleDepthImage.hpp"
#include <cmath>


int SimpleDepthImage::readInputImage(std::string inputImageName)
{
  //load input bitmap image
  inputBitmap.load(inputImageName.c_str());
  
  //error if image did not load
  if(!inputBitmap.isLoaded())
    {
      error("Failed to load input image!");
      return SDK_FAILURE;
    }
  
  //get width and height of input image
  height = inputBitmap.getHeight();
  width = inputBitmap.getWidth();
  
  //allocate memory for input image data (only Gray component) to host
  inputImageData = (cl_float*)malloc(width * height * sizeof(cl_float));
  CHECK_ALLOCATION(inputImageData,"Failed to allocate memory! (inputImageData)");

  //get the pointer to pixel data
  pixelData = inputBitmap.getPixels();
  CHECK_ALLOCATION(pixelData,"Failed to read pixel Data!");

  //calculating Y component from pixel data, using RGB to YUV conversion
  unsigned int R, G, B, Y;
  for(int i = 0; i <  width*height; i++)
    {
      R = pixelData[i].x;
      G = pixelData[i].y;
      B = pixelData[i].z;

      Y = ((66 * R + 129 * G +  25 * B + 128) >> 8) +  16;

      //make image from 0 to 1
      inputImageData[i] = (float)(Y)/(float)(256.0);
    }

  
  //allocate memory for output image data to host
  outputImageData = (cl_float*)malloc(width * height * sizeof(cl_float));
  CHECK_ALLOCATION(outputImageData,
                   "Failed to allocate memory! (outputImageData)");
  memset((void *)outputImageData, 0, width * height);

  //allocate memory for verification of depth image Kernel to host
  refOutputImageData = (cl_float*)malloc(width * height * sizeof(cl_float));
  CHECK_ALLOCATION(refOutputImageData,
                   "Failed to allocate memory! (outputImageData)");
  memset((void *)refOutputImageData, 0, width * height);

  return SDK_SUCCESS;

}

int SimpleDepthImage::writeOutputImage(cl_float *imageData, 
                                       std::string outputImageName)
{
  // copy output image data back to original pixel data
  memset(pixelData, 0xff, width * height * pixelSize);
  for(int i = 0; i <  width*height; i++)
    {   
      pixelData[i].x = (cl_uchar)(imageData[i]*(MAX_LUMA));
      pixelData[i].y = pixelData[i].x;
      pixelData[i].z = pixelData[i].x;
    }

  // write the output bmp file
  if(!inputBitmap.write(outputImageName.c_str()))
    {
      std::cout << "Failed to write output image!";
      return SDK_FAILURE;
    }
  return SDK_SUCCESS;
}


int SimpleDepthImage::genBinaryImage()
{
  bifData binaryData;
  binaryData.kernelName = std::string("SimpleDepthImage_Kernels.cl");
  binaryData.flagsStr = std::string("");
  if(sampleArgs->isComplierFlagsSpecified())
    {
      binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }
  
  binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
  int status = generateBinaryImage(binaryData);
  return status;
}


int SimpleDepthImage::setupCL()
{
  cl_int         status = CL_SUCCESS;
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
  status                  = getPlatform(platform, sampleArgs->platformId,
                                        sampleArgs->isPlatformEnabled());
  CHECK_ERROR(status, SDK_SUCCESS, "getPlatform() failed");
  
  // Display available devices.
  status = displayDevices(platform, dType);
  CHECK_ERROR(status, SDK_SUCCESS, "displayDevices() failed");
  
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
  bool cl2enabled = deviceInfo.checkOpenCL2_XCompatibility();
  if(!cl2enabled)
    {
      OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher.");
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
  cl_image_desc         imageDesc;
  cl_image_format       imageFormat;        

  // Image is a depth image
  imageFormat.image_channel_data_type  = CL_FLOAT;
  imageFormat.image_channel_order      = CL_DEPTH;

  memset(&imageDesc, '\0', sizeof(cl_image_desc));
  imageDesc.image_type                 = CL_MEM_OBJECT_IMAGE2D;
  imageDesc.image_width                = width;
  imageDesc.image_height               = height;

  // Create 2D image, which will be used as input as well as output image
  oclImage = clCreateImage(context,
                           CL_MEM_READ_WRITE,
                           &imageFormat,
                           &imageDesc,
                           NULL,
                           &status);
  CHECK_OPENCL_ERROR(status,"clCreateImage failed.(oclImage)");

  // create a CL program using the kernel source
  buildProgramData buildData;
  buildData.kernelName  = std::string("SimpleDepthImage_Kernels.cl");
  buildData.devices     = devices;
  buildData.deviceId    = sampleArgs->deviceId;
  buildData.flagsStr    = std::string("-I. -cl-std=CL2.0");  

  if(sampleArgs->isLoadBinaryEnabled())
    {
      buildData.binaryName = std::string(sampleArgs->loadBinary.c_str());
    }
  
  if(sampleArgs->isComplierFlagsSpecified())
    {
      buildData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

  status = buildOpenCLProgram(program, context, buildData);
  CHECK_ERROR(status, SDK_SUCCESS, "buildOpenCLProgram() failed");

  
  // get a kernel object handle for a depth image manipulation kernel
  simpleDepthKernel = clCreateKernel(program, "binarizeDepth", &status);
  CHECK_OPENCL_ERROR(status,"clCreateKernel failed.(binarizeDepth)");
  
  return SDK_SUCCESS;
}

int SimpleDepthImage::runCLKernels()
{
  cl_int    status;
  cl_float  normalizedThreshold;

  // set the threshold for binarization
  normalizedThreshold = (float)(threshold)/(float)(MAX_LUMA +1);

  // initialize image for ImageBinarization Kernel
  cl_event writeEvt;
  origin[0] = origin[1] = origin[2] = 0;
  region[0] = width; 
  region[1] = height; 
  region[2] = 1;
    
  status = clEnqueueWriteImage(commandQueue,
                               oclImage,
                               CL_FALSE,
                               origin,
                               region,
                               0,
                               0,
                               inputImageData,
                               0,
                               NULL,
                               &writeEvt);
  CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (deviceImageData1D)");
    
  status = waitForEventAndRelease(&writeEvt);
  CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(writeEvt) Failed");
    
  // Set appropriate arguments to the imageBinarization kernel
  status = clSetKernelArg(simpleDepthKernel,
                          0,
                          sizeof(cl_mem),
                          &oclImage);
  CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (image2D)");
    
  // dummy threshold
  status = clSetKernelArg(simpleDepthKernel,
                          1,
                          sizeof(cl_float),
                          &normalizedThreshold);
  CHECK_OPENCL_ERROR(status,"clSetKernelArg failed. (threshold)");
    
  size_t globalThreads2[] = {width, height};

  // Enqueue a imageBinarizationKernel run call
  status = clEnqueueNDRangeKernel(commandQueue,
                                  simpleDepthKernel,
                                  2,
                                  NULL,
                                  globalThreads2,
                                  NULL,
                                  0,
                                  NULL,
                                  0);
  CHECK_OPENCL_ERROR(status,"clEnqueueNDRangeKernel(simpleDepthKernel) failed.");
    
  status = clFinish(commandQueue);
  CHECK_OPENCL_ERROR(status,"clFinish failed.");
  
  return SDK_SUCCESS;
}

int SimpleDepthImage::initialize()
{

  // Call base class Initialize to get default configuration
  if (sampleArgs->initialize() != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }

  Option* new_option = new Option;
  CHECK_ALLOCATION(new_option, "Memory Allocation error.(new_option)");
  
  new_option->_sVersion       = "i";
  new_option->_lVersion       = "iterations";
  new_option->_description    = "Number of iterations to execute kernel";
  new_option->_type           = CA_ARG_INT;
  new_option->_value          = &iterations;
  sampleArgs->AddOption(new_option);
  
  new_option->_sVersion       = "z";
  new_option->_lVersion       = "threshold";
  new_option->_description    = "threshold for binarization";
  new_option->_type           = CA_ARG_INT;
  new_option->_value          = &threshold;
  sampleArgs->AddOption(new_option);


  delete new_option;

  return SDK_SUCCESS;
}

int
SimpleDepthImage::setup()
{
  int status = 0;

  //sanity check on input
  if((threshold < MIN_LUMA) || (threshold > MAX_LUMA))
    {
      std::cout << "Error: Threshold should be in the range ";
      std::cout << "[" << MIN_LUMA << ":" << MAX_LUMA << "]";
      std::cout << std::endl;
      return SDK_EXPECTED_FAILURE;
    }

  // Allocate host memory and read input image
  std::string filePath = getPath() + std::string(INPUT_IMAGE);
  status = readInputImage(filePath);
  CHECK_ERROR(status, SDK_SUCCESS, "Read Input Image failed");

  // create and initialize timers
  int timer = sampleTimer->createTimer();
  sampleTimer->resetTimer(timer);
  sampleTimer->startTimer(timer);

  // setup stage for OCL execution
  status = setupCL();
  if(status != SDK_SUCCESS)
    {
      return status;
    }
  
  sampleTimer->stopTimer(timer);

  // compute setup time
  setupTime = (double)(sampleTimer->readTimer(timer));

  return SDK_SUCCESS;

}

int SimpleDepthImage::run()
{
  int status;

  // Warm up run
  for(int i = 0; i < 2 && iterations != 1; i++)
    {
      // Arguments are set and execution call is enqueued on command buffer
      if(runCLKernels() != SDK_SUCCESS)
        {
          return SDK_FAILURE;
        }
    }
  
  // create and initialize timers
  std::cout << "Executing kernel for " << iterations << " iterations" <<std::endl;
  std::cout << "-------------------------------------------" << std::endl;

  int timer = sampleTimer->createTimer();
  sampleTimer->resetTimer(timer);
  sampleTimer->startTimer(timer);

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
                              oclImage,
                              1,
                              origin,
                              region,
                              0,
                              0,
                              outputImageData,
                              0, 0, 0);
  CHECK_OPENCL_ERROR(status,
                     "clEnqueueReadImage(outputImageData) failed.");

  // Wait for the read buffer to finish execution
  status = clFinish(commandQueue);
  CHECK_OPENCL_ERROR(status,"clFinish failed.(commandQueue)");
  
  // write the output image to bitmap file
  status = writeOutputImage(outputImageData, OUTPUT_IMAGE);
  CHECK_ERROR(status, SDK_SUCCESS, "write Output Image Failed");

  return SDK_SUCCESS;
}

int SimpleDepthImage::cleanup()
{
  // Releases OpenCL resources (Context, Memory etc.)
  cl_int status;

  status = clReleaseKernel(simpleDepthKernel);
  CHECK_OPENCL_ERROR(status,"clReleaseKernel failed.(simpleDepthKernel)");

  status = clReleaseProgram(program);
  CHECK_OPENCL_ERROR(status,"clReleaseProgram failed.(program)");

  status = clReleaseMemObject(oclImage);
  CHECK_OPENCL_ERROR(status,"clReleaseMemObject failed.(oclImage)");

  status = clReleaseCommandQueue(commandQueue);
  CHECK_OPENCL_ERROR(status,"clReleaseCommandQueue failed.(commandQueue)");

  status = clReleaseContext(context);
  CHECK_OPENCL_ERROR(status,"clReleaseContext failed.(context)");

  // release program resources (input memory etc.)
  FREE(inputImageData);
  FREE(outputImageData);
  FREE(refOutputImageData);
  FREE(devices);
  
  return SDK_SUCCESS;
}

void SimpleDepthImage::CPUReference()
{
  float pixel;
  float normalizedThreshold;

  normalizedThreshold = (float)(threshold)/(float)(MAX_LUMA +1);

  for(int i = 0; i < width*height; i++)
    {
      pixel = inputImageData[i];


      if(pixel < normalizedThreshold)
        refOutputImageData[i] = (float)(BIN_ONE);
      else
        refOutputImageData[i] = (float)(BIN_TWO);
    }

}


int SimpleDepthImage::verifyResults()
{
  int    status  = 0;
  int    count  = 0;
  float  oclPix,refPix;

  if(sampleArgs->verify)
    {
      // reference implementation on host
      CPUReference();
      
          //write reference image
      status = writeOutputImage(refOutputImageData, REF_IMAGE);
      CHECK_ERROR(status, SDK_SUCCESS, "write Output Image Failed");

      // compare the results and see if they match
          status = 1;
      for(int i = 0; i < width*height; i++)
        {
          oclPix = outputImageData[i];
          refPix = refOutputImageData[i];
          
          if(F_ABS(oclPix - refPix) > F_EPSILON)
            {
              status =  0;
              count  += 1;
            }

        }
      

      if(status)
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


void SimpleDepthImage::printStats()
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


int main(int argc, char * argv[])
{
  int              status              = SDK_SUCCESS;
  SimpleDepthImage clSimpleDepthImage;
  
  if (clSimpleDepthImage.initialize() != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }

  if (clSimpleDepthImage.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }

  if(clSimpleDepthImage.sampleArgs->isDumpBinaryEnabled())
    {
        return clSimpleDepthImage.genBinaryImage();
    }

  status = clSimpleDepthImage.setup();
  if(status != SDK_SUCCESS)
    {
        return status;
    }

  if (clSimpleDepthImage.run() !=SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

  if (clSimpleDepthImage.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

  if (clSimpleDepthImage.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

  clSimpleDepthImage.printStats();
  return SDK_SUCCESS;
}
