/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

1   Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.
2   Redistributions in binary form must reproduce the above copyright notice, th
is list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
#include "RegionGrowingCPU.hpp"
#include "RegionGrowingSegmentation.hpp"

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

int RegionGrowingSegmentation::initialize()
{
    // Call base class Initialize to get default configuration
    if(sampleArgs->initialize())
    {
        return SDK_FAILURE;
    }

    Option* addtionalOptions = new Option;
    CHECK_ALLOCATION(addtionalOptions, "Memory allocation error.\n");

    addtionalOptions->_sVersion    = "o";
    addtionalOptions->_lVersion    = "hostq";
    addtionalOptions->_value       = &hostQ;
    addtionalOptions->_description = "Host side enqueue of kernels";
    addtionalOptions->_type        = CA_NO_ARGUMENT;

    sampleArgs->AddOption(addtionalOptions);

    delete addtionalOptions;

    return SDK_SUCCESS;
}

int RegionGrowingSegmentation::genBinaryImage()
{
    bifData binaryData;
    binaryData.kernelName = std::string("RegionGrowingSegmentation_Kernels.cl");
    binaryData.flagsStr = std::string("");
    if(sampleArgs->isComplierFlagsSpecified())
    {
        binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }
    binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
    int status = generateBinaryImage(binaryData);
    return status;
}

int RegionGrowingSegmentation::readSeedFile(std::string seedFileName)
{
  FILE*     fp;
  cl_uint   val;
  int       status;

  fp = fopen(seedFileName.c_str(),"r");

  if(fp == NULL)
    {
      std::cout << "Error: Seed File is not found..." << std::endl;
      return SDK_FAILURE;
    }

  /* read number of regions */
  status = fscanf_s(fp,"%d", &numRegions);

  if(numRegions < 2)
    {
      std::cout << "Error: Error in seed file..." << std::endl;
      std::cout << "       At least two regions are required." << std::endl;
      return SDK_FAILURE;
    }

  if(numRegions > 200)
    {
      std::cout << "Error: Error in seed file..." << std::endl;
      std::cout << "       More than 200 regions are not supported." ;
      std::cout << std::endl;
      return SDK_FAILURE;
    }

  /* read seed pixels */
  pSeeds = (cl_uint2*)malloc(numRegions*sizeof(cl_uint2));
  CHECK_ALLOCATION(pSeeds,"Failed to allocate memory! (pSeeds)");
  memset(pSeeds,0,numRegions*sizeof(cl_uint2));

  for(int i = 0; i < numRegions; ++i)
    {
      status = fscanf_s(fp,"%d", &val);
      pSeeds[i].x = val;

      status = fscanf_s(fp,"%d", &val);
      pSeeds[i].y = val;
    }
  
  fclose(fp);

  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::readInputImage(std::string inputImageName)
{
  /* load input bitmap image */
  std::string filePath = getPath() + inputImageName;
  inputBitmap.load(filePath.c_str());
  if(!inputBitmap.isLoaded())
    {
      std::cout << "Failed to load input image " << filePath.c_str();
      std::cout << std::endl;
      return SDK_FAILURE;
    }
  
  /* get width and height of input image */
  height = inputBitmap.getHeight();
  width = inputBitmap.getWidth();
  
  if(!sampleArgs->quiet)
    {
      std::cout << "Image Size [Width = " << width;
      std::cout << " , Height = " << height << "]" << std::endl;
    }
  
  /* get the pointer to pixel data */
  pPixelData = inputBitmap.getPixels();
  
  if(pPixelData == NULL)
    {
      std::cout << "Failed to read pixel Data!";
      return SDK_FAILURE;
    }
  
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::writeOutputImage(std::string outputImageName,
                                                cl_uchar4*  pImage)
{
  /* copy output image data back to original pixel data */
  memcpy(pPixelData, pImage, width * height * pixelSize);

  /* write the output bmp file */
  if(!inputBitmap.write(outputImageName.c_str()))
    {
      std::cout << "Failed to write output image!";
      return SDK_FAILURE;
    }
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::setupCL()
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
  
  /* Display available devices. */
  retValue = displayDevices(platform, dType);
  CHECK_ERROR(retValue, SDK_SUCCESS, "displayDevices() failed");
  
  /*  
   * If we could find our platform, use it. Otherwise use just available 
   * platform. 
   */
  
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
  
  retValue = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
  CHECK_ERROR(retValue, 0, "SDKDeviceInfo::setDeviceInfo() failed");
  
  /* check ocl2.0 compatibility */
  bool cl2enabled = deviceInfo.checkOpenCL2_XCompatibility();
  if(!cl2enabled)
    {
      OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher.");
    }

  /* define command queue */
  cl_command_queue_properties prop[] = {0};
  commandQueue = clCreateCommandQueueWithProperties(
                                                    context,
                                                    devices[sampleArgs->deviceId],
                                                    prop,
                                                    &status);
  CHECK_OPENCL_ERROR( status, "clCreateCommandQueue failed(commandQueue).");
  
  /* get the device info */
  status = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
  CHECK_OPENCL_ERROR(status, "setDeviceInfo failed.");
  
  /* define device side command queue */
  cl_command_queue_properties devProp[] 
    = {CL_QUEUE_PROPERTIES, 
       CL_QUEUE_ON_DEVICE|
       CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE|
       CL_QUEUE_ON_DEVICE_DEFAULT, 
       CL_QUEUE_SIZE, 
       deviceInfo.maxQueueSize, 
       0};
  
  devCommandQueue = clCreateCommandQueueWithProperties(
                                                       context,
                                                       devices[sampleArgs->deviceId],
                                                       devProp,
                                                       &status);
  CHECK_OPENCL_ERROR( status, "clCreateCommandQueue failed(devCommandQueue).");
  
  /* allocate a buffer for input image */
  oclImageBuf = clCreateBuffer(context,
                               CL_MEM_READ_WRITE,
                               width*height*pixelSize,
                               NULL,
                               &status);
  CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (oclImageBuf)");
  
  /* create buffer to hold queued pixels on the device */
  oclQueuedPixels = clCreateBuffer(context, 
                                   CL_MEM_READ_WRITE, 
                                   (width)*(height)*sizeof(cl_int2),
                                   NULL,
                                   &status);
  
  CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (oclQueuedPixels)");

  /* create a buffer to pass parameters */
  oclParams  = clCreateBuffer(context, 
                              CL_MEM_READ_WRITE, 
                              sizeof(RGSParamType),
                              NULL,
                              &status);
  CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (oclQueuedPixels)");
  
  /* Build the kernels. */
  buildProgramData buildData;
  buildData.kernelName = std::string("RegionGrowingSegmentation_Kernels.cl");
  buildData.devices = devices;
  buildData.deviceId = 0;
  buildData.flagsStr = std::string("-I. -cl-std=CL2.0");
  
  status = buildOpenCLProgram(program, context, buildData);
  CHECK_ERROR(status, 0, "sampleCommand::buildOpenCLProgram() failed.");
  
  initKernel = clCreateKernel(program, "init_kernel", &status);
  CHECK_OPENCL_ERROR(status, "clCreateKernel failed (init_kernel).");
  
  growRegionKernel = clCreateKernel(program, "grow_region", &status);
  CHECK_OPENCL_ERROR(status, "clCreateKernel failed (grow_region).");
  
  /* create additional kernels if host side iteration is enabled */
  if(hostQ)
    {
      classifyKernel = clCreateKernel(program, "classify", &status);
      CHECK_OPENCL_ERROR(status, "clCreateKernel failed (classify).");
  
      queueNeighKernel = clCreateKernel(program, "queue_neigh", &status);    
      CHECK_OPENCL_ERROR(status, "clCreateKernel failed (queue_neigh).");
    }
  
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::preProcess()
{
  int      i;
  cl_int   status;
  
  /* allocate memory for input and output images */
  pOclImageData = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
  CHECK_ALLOCATION(pOclImageData,
                   "Failed to allocate memory! (pOclImageData)");
  
  memcpy(pOclImageData, pPixelData, width * height * pixelSize);
  
  /* convert RGB input image to YVU */
  rgbToYvu(pOclImageData);
  
  /* set all pixels except boundary pixels as unclassified */
  for(cl_uint r = 0; r < height; ++r)
    {
      for(cl_uint c = 0; c < width; ++c)
        {
          i = TWOD_TO_ONED(c,r,width);

          pOclImageData[i].w = RGS_REGION_UNCLASSIFIED;

          if((r == 0) || (r == height -1))
            pOclImageData[i].w = RGS_REGION_BOUNDARY;

          if((c == 0) || (c == width -1))
            pOclImageData[i].w = RGS_REGION_BOUNDARY;
        }
    }
  
  /* copy input image to OCL buffer */
  status = clEnqueueWriteBuffer(commandQueue,
                                oclImageBuf,
                                CL_TRUE,
                                0,
                                width*height*pixelSize,
                                (void *)(pOclImageData),
                                0,
                                NULL,
                                NULL);
  CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (oclImageBuf)"); 
  

  /* copy the queued pixels */
  pQueuedPixels = (cl_int2 *)malloc(width*height*sizeof(cl_int2));
  for(cl_int i = 0; i < numRegions; ++i)
    {
      pQueuedPixels[i].x = pSeeds[i].x;
      pQueuedPixels[i].y = height -1 - pSeeds[i].y;
    }
  
  for(cl_int i = numRegions; i < (cl_int)((width)*(height)); ++i)
    {
      pQueuedPixels[i].x = 0;
      pQueuedPixels[i].y = 0;
    }
  
  status = clEnqueueWriteBuffer(commandQueue,
                                oclQueuedPixels,
                                CL_TRUE,
                                0,
                                width*height*sizeof(cl_int2),
                                (void *)(pQueuedPixels),
                                0,
                                NULL,
                                NULL);
  CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (oclQueuedPixels)"); 
    
  /* copy parameters */
  /*** 
       note - pQueuedPixel buffer is devided into two buffers that act as
       ping-pong buffers for queued pixels in iterative fashion. eoffset and 
       ooffset give the offsets of these ping-pong buffers in pQueuedPixel  
  ***/

  pParams              = (RGSParamType *)malloc(sizeof(RGSParamType));;
  pParams->width       = width;
  pParams->height      = height;
  pParams->regions     = numRegions;
  pParams->eoffset     = 0;
  pParams->ooffset     = width*height/2;
  pParams->queued      = numRegions;
  pParams->hostq        = hostQ;
 
  status = clEnqueueWriteBuffer(commandQueue,
                                oclParams,
                                CL_TRUE,
                                0,
                                sizeof(RGSParamType),
                                (void *)(pParams),
                                0,
                                NULL,
                                NULL);
  CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (oclParams)"); 
  

  /* initialize the program scope OCL variables */
  size_t        localThreads[]  = {RGS_LOCAL_THREADS};
  size_t        globalThreads[] = {RGS_LOCAL_THREADS};

  
  status = clSetKernelArg(initKernel, 0, sizeof(cl_mem), &oclImageBuf);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (initKernel).");

  status = clSetKernelArg(initKernel, 1, sizeof(cl_mem), &oclQueuedPixels);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (initKernel).");

  status = clSetKernelArg(initKernel, 2, sizeof(cl_mem), &oclParams);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (initKernel).");

  status = clEnqueueNDRangeKernel(commandQueue, 
                                  initKernel, 
                                  1, 
                                  NULL,
                                  globalThreads, 
                                  localThreads, 
                                  0, 
                                  NULL, 
                                  NULL);
  CHECK_OPENCL_ERROR(status,
                     "clEnqueueNDRangeKernel failed (initKernel).");
  
  status = clFinish(commandQueue);
  CHECK_OPENCL_ERROR(status, "clFinish failed - initKernel.");

  /* initialize CPU reference class */
  pVerImageData = (cl_uchar4*)malloc(width * height * pixelSize);
  CHECK_ALLOCATION(pVerImageData,
                   "Failed to allocate memory! (verImageData)");
  
  memcpy(pVerImageData, pOclImageData, width * height * pixelSize);
  
  cpuRgs.init((&pVerImageData),
              pQueuedPixels,
              pParams);
  
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::postProcess()
{
  cl_int   status;

  /* get the OCL image back to host memeory */
  status = clEnqueueReadBuffer(commandQueue,
                               oclImageBuf,
                               CL_TRUE,
                               0,
                               width*height*pixelSize,
                               (void *)(pOclImageData),
                               0,
                               NULL,
                               NULL);
  CHECK_OPENCL_ERROR(status, "clEnqueueReadBuffer failed. (oclImageBuf)"); 

  /* convert images suitable for display */
  convertImage(pOclImageData);
  convertImage(pVerImageData);

  /* format conversion */
  yvuToRgb(pOclImageData);
  yvuToRgb(pVerImageData);

  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::setup()
{
  /* Allocate host memory and read input image */
  if(readInputImage(INPUT_IMAGE) != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }
  
  /* Read the seeds */
  if(readSeedFile(SEED_FILE) != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }
  
  /* Set up OpenCL  and sample related environment. */
  int status = setupCL();
  if (status != SDK_SUCCESS)
  {
	  return status;
  }
  

    return SDK_SUCCESS;
}

int RegionGrowingSegmentation::runCLKernels()
{
  cl_int   status;
  cl_uint  queued, wgn, res;

  size_t    localThreads[]    = {RGS_LOCAL_THREADS};
  size_t    globalThreads[]   = {RGS_LOCAL_THREADS};

  status = clSetKernelArg(growRegionKernel, 0, sizeof(cl_mem), &oclImageBuf);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (growRegionKernel).");
  
  status = clSetKernelArg(growRegionKernel, 1, sizeof(cl_mem), &oclQueuedPixels);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (growRegionKernel).");

  status = clSetKernelArg(growRegionKernel, 2, sizeof(cl_mem), &oclParams);
  CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (growRegionKernel).");
  
  if(hostQ)
    {
      /* host side iteration */

      status = clSetKernelArg(classifyKernel, 0, sizeof(cl_mem), &oclImageBuf);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (classifyKernel).");
      
      status = clSetKernelArg(classifyKernel, 1, sizeof(cl_mem), &oclQueuedPixels);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (classifyKernel).");
      
      status = clSetKernelArg(classifyKernel, 2, sizeof(cl_mem), &oclParams);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (classifyKernel).");
      
      status = clSetKernelArg(queueNeighKernel, 0, sizeof(cl_mem), &oclImageBuf);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (queueNeighKernel).");
      
      status = clSetKernelArg(queueNeighKernel, 1, sizeof(cl_mem), &oclQueuedPixels);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (queueNeighKernel).");
      
      status = clSetKernelArg(queueNeighKernel, 2, sizeof(cl_mem), &oclParams);
      CHECK_OPENCL_ERROR(status, "clSetKernelArg failed (queueNeighKernel).");
      
      queued = numRegions;
      while(queued > 0)
	{
	  globalThreads[0] = localThreads[0];
	  
	  status = clEnqueueNDRangeKernel(commandQueue, 
					  growRegionKernel, 
					  1, 
					  NULL,
					  globalThreads, 
					  localThreads, 
					  0, 
					  NULL, 
					  NULL);
	  CHECK_OPENCL_ERROR(status,
			     "clEnqueueNDRangeKernel failed (growRegionKernel).");
	  
	  globalThreads[0] = queued;
	  wgn              = (cl_uint)(globalThreads[0]/localThreads[0]);
	  res              = (cl_uint)(globalThreads[0] % localThreads[0]);
	  
	  if(res)
	    wgn += 1;
	  
	  globalThreads[0] = wgn*localThreads[0];
	  
	  status = clEnqueueNDRangeKernel(commandQueue, 
					  queueNeighKernel, 
					  1, 
					  NULL,
					  globalThreads, 
					  localThreads, 
					  0, 
					  NULL, 
					  NULL);
	  CHECK_OPENCL_ERROR(status,
			     "clEnqueueNDRangeKernel failed (queueNeighKernel).");
	  
	  globalThreads[0] = RGS_NEIGH*queued;
	  
	  status = clEnqueueNDRangeKernel(commandQueue, 
					  classifyKernel, 
					  1, 
					  NULL,
					  globalThreads, 
					  localThreads, 
					  0, 
					  NULL, 
					  NULL);
	  CHECK_OPENCL_ERROR(status,
			     "clEnqueueNDRangeKernel failed (queueNeighKernel).");
	  
	  
	  status = clFinish(commandQueue);
	  CHECK_OPENCL_ERROR(status, "clFinish failed(growRegionKernel).");
	  
	  status = clEnqueueReadBuffer(commandQueue,
				       oclParams,
				       CL_TRUE,
				       0,
				       sizeof(RGSParamType),
				       (void *)(pParams),
				       0,
				       NULL,
				       NULL);
	  CHECK_OPENCL_ERROR(status, "clEnqueueWriteBuffer failed. (oclParams)"); 
	  
	  queued = pParams->queued;
	}
    }
  else
    {
      /* device side enqueuing. only one kernel invocation is needed here */
      status = clEnqueueNDRangeKernel(commandQueue, 
				      growRegionKernel, 
				      1, 
				      NULL,
				      globalThreads, 
				      localThreads, 
				      0, 
				      NULL, 
				      NULL);
      CHECK_OPENCL_ERROR(status,
			 "clEnqueueNDRangeKernel failed (growRegionKernel).");
      
      status = clFinish(commandQueue);
      CHECK_OPENCL_ERROR(status, "clFinish failed(growRegionKernel).");
    }
 
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::run()
{
  if(!sampleArgs->quiet)
    {
      if(hostQ)
	std::cout << "Host enqueue of kernels... " << std::endl;
      else
	std::cout << "Device enqueue of kernels... " << std::endl;	
    }

  /* actual run and timing information */  
  int timer = sampleTimer->createTimer();
  sampleTimer->resetTimer(timer);
  sampleTimer->startTimer(timer);

   if (preProcess() != SDK_SUCCESS)
	{
	  return SDK_FAILURE;
	}

    if(runCLKernels() != SDK_SUCCESS)
    {
	return SDK_FAILURE;
    }
  
  sampleTimer->stopTimer(timer);
  totalKernelTime = (double)(sampleTimer->readTimer(timer));

  /* native execution for verification */
  if (runRef() != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }

  /* post processing, that is YUV to RGB conversion */
  if (postProcess() != SDK_SUCCESS)
    {
      return SDK_FAILURE;
    }

  return SDK_SUCCESS;
}

void RegionGrowingSegmentation::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[4] = {"Width", 
                                   "Height", 
                                   "Kernel Time (sec)", 
                                   "Pixels/sec"};
        std::string stats[4];

        sampleTimer->totalTime = ( totalKernelTime );

        stats[0] = toString(width   , std::dec);
        stats[1] = toString(height, std::dec);
        stats[2] = toString( sampleTimer->totalTime, std::dec);
        stats[3] = toString((width * height)/sampleTimer->totalTime, std::dec);

        printStatistics(strArray, stats, 4);

    }
}

int RegionGrowingSegmentation::cleanup()
{
    /* Releases OpenCL resources (Context, Memory etc.) */
    cl_int status;

    status = clReleaseKernel(growRegionKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");

    status = clReleaseKernel(initKernel);
    CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");

    if(hostQ)
      {
	status = clReleaseKernel(classifyKernel);
	CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");

	status = clReleaseKernel(queueNeighKernel);
	CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.");
      }

    status = clReleaseProgram(program);
    CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.");

    status = clReleaseMemObject(oclImageBuf);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseMemObject(oclQueuedPixels);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseMemObject(oclParams);
    CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.");

    status = clReleaseCommandQueue(commandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.");

    status = clReleaseCommandQueue(devCommandQueue);
    CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.");

    status = clReleaseContext(context);
    CHECK_OPENCL_ERROR(status, "clReleaseContext failed.");

    /* release program resources (input memory etc.) */
    FREE(devices);
    FREE(pOclImageData);
    FREE(pVerImageData);
    FREE(pQueuedPixels);
    FREE(pParams);

    return SDK_SUCCESS;
}

int RegionGrowingSegmentation::runRef()
{
  cpuRgs.growRegion();
  
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::verifyResults()
{
    if(sampleArgs->verify)
      {
        /* write ocl output image */
        if (writeOutputImage(OCL_OUTPUT_IMAGE, pOclImageData) != SDK_SUCCESS)
          {
            std::cout << "writing image output failed." << std::endl;
            return SDK_FAILURE;
          }

        /* write verification output image */
        if (writeOutputImage(VER_OUTPUT_IMAGE, pVerImageData) != SDK_SUCCESS)
          {
            std::cout << "writing image output failed." << std::endl;
            return SDK_FAILURE;
          }
        
        /* compare the images */
        if (compareImages() == SDK_SUCCESS)
          std::cout << "\nPassed!\n";
        else
          std::cout << "\nFailed!\n";
      }
    return SDK_SUCCESS;
}

int RegionGrowingSegmentation::compareImages()
{
  int i;
  int status = SDK_SUCCESS;
  int count  = 0;

  for(cl_uint r = 1; r < height -1; ++r)
    {
      for(cl_uint c = 1; c < width -1; ++c)
        {
          i = TWOD_TO_ONED(c,r,width);

          if((int)pOclImageData[i].w != (int)pVerImageData[i].w)
            {
              count++;
              status = SDK_FAILURE;
            }
        }
    }

  if(status == SDK_FAILURE)
    {
      float mismatch_ratio = (float)(count)*100.0f/(float)(width*height);

      if(mismatch_ratio < RGS_ACCEPTABLE_MISMATCH)
        {

          status = SDK_SUCCESS;
        }
      else
        {
		  std::cout << "No of mismatches " << count << std::endl;
          std::cout << "[" << (int)(mismatch_ratio) << "%] mismatch found.";
          std::cout << std::endl;
        }
    }

  return status;
}

int RegionGrowingSegmentation::convertImage(cl_uchar4* pImage)
{
  int  region, i;
  int  range = RGS_REGION_STOP - RGS_REGION_START +1;
  int  jmp   = range/numRegions;

  for (cl_uint r = 0; r < height; ++r)
    {
      for (cl_uint c = 0; c < width; ++c)
        {
          i = TWOD_TO_ONED(c,r,width);

          region = (int)(pImage[i].w);

          if((region >= RGS_REGION_START) && 
             (region <= RGS_REGION_STOP))
            {
              pImage[i].x = RGS_REGION_STOP 
                - jmp*(region - RGS_REGION_START);
            }
          else
            {
              pImage[i].x = RGS_REGION_BOUNDARY; 
            }
        }
    }

  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::rgbToYvu(cl_uchar4* pImage)
{
  int   i;
  float r,g,b,y,v,u;
  int   iy,iv,iu;

  for(cl_uint rr = 0; rr < height; ++rr)
    {
      for(cl_uint cc = 0; cc < width; ++cc)
        {
          i = TWOD_TO_ONED(cc,rr,width);

          /* RGB to YUV (or YCbCr) */
          r = (float)pImage[i].x/255.0f;
          g = (float)pImage[i].y/255.0f;
          b = (float)pImage[i].z/255.0f;
          
          y = 0.299f*r + 0.587f*g + 0.114f*b;
          v = (r-y)*0.713f + 0.5f;
          u = (b-y)*0.565f + 0.5f;
          
          iy          = (int)(y*255);
          pImage[i].x  = CLAMP(iy); 
          
          iv          = (int)(v*255);
          pImage[i].y = CLAMP(iv); 
          
          iu          = (int)(u*255);
          pImage[i].z  = CLAMP(iu); 
        }
    }
  
  return SDK_SUCCESS;
}

int RegionGrowingSegmentation::yvuToRgb(cl_uchar4* pImage)
{
  int   i;
  float r,g,b,y,v,u;
  int   ir,ig,ib;

  for(cl_uint rr = 0; rr < height; ++rr)
    {
      for(cl_uint cc = 0; cc < width; ++cc)
        {
          i = TWOD_TO_ONED(cc,rr,width);

          y = (float)pImage[i].x/255.0f;
          v = (float)pImage[i].y/255.0f;
          u = (float)pImage[i].z/255.0f;
          
          v = v - 0.5f;
          u = u - 0.5f;
          
          /* force V and U componenet to be zero */
          v = u = 0.0;
          
          /* YUV (or YCbCr) to RGB */
          r = y + 1.403f*v;
          g = y - 0.344f*u - 0.714f*v;
          b = y + 1.77f*u;
          
          ir = (int)(r*255);
          pImage[i].x = CLAMP(ir); 
          
          ig = (int)(g*255);
          pImage[i].y = CLAMP(ig); 
          
          ib = (int)(b*255);
          pImage[i].z = CLAMP(ib); 
        }
    }
  return SDK_SUCCESS;
}

int main(int argc, char * argv[])
{
	int status = 0;
    RegionGrowingSegmentation clRGS;

    if(clRGS.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clRGS.sampleArgs->parseCommandLine(argc, argv) 
       != SDK_SUCCESS)
    {
        return SDK_EXPECTED_FAILURE;
    }

    if(clRGS.sampleArgs->isDumpBinaryEnabled())
    {
        return clRGS.genBinaryImage();
    }
    else
    {
		status = clRGS.setup();
		if (status != SDK_SUCCESS)
		{
			return status;
		}
      
		status = clRGS.run();
		if (status != SDK_SUCCESS)
        {
          return status;
        }
      
      if(clRGS.verifyResults() != SDK_SUCCESS)
        {
          return SDK_FAILURE;
        }
      
      if(clRGS.cleanup() != SDK_SUCCESS)
        {
          return SDK_FAILURE;
        }
      
      clRGS.printStats();

    }

    return SDK_SUCCESS;
}
