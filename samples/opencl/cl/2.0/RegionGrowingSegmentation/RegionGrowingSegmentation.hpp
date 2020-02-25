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
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIM
ED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INC
LUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef _REGION_GROWING_SEGMENTATION_H_
#define _REGION_GROWING_SEGMENTATION_H_ 

#include "CLUtil.hpp"
#include "SDKBitMap.hpp"
#include "RegionGrowingConst.hpp"

using namespace appsdk;

#define INPUT_IMAGE      "RegionGrowing_Input.bmp"
#define OCL_OUTPUT_IMAGE "RegionGrowing_ocl_output.bmp"
#define VER_OUTPUT_IMAGE "RegionGrowing_ver_output.bmp"
#define SEED_FILE        "RegionGrowing_seeds.txt"

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

class RegionGrowingSegmentation
{
  cl_context          context;
  cl_device_id*       devices;
  cl_command_queue    commandQueue;
  cl_command_queue    devCommandQueue;
  cl_program          program;

  cl_kernel           growRegionKernel; 
  cl_kernel           initKernel; 
  
  cl_kernel           queueNeighKernel; 
  cl_kernel           classifyKernel; 
 
  SDKBitMap           inputBitmap;
  
  cl_int              numRegions;
  cl_uint2*           pSeeds;

  uchar4*             pPixelData;
  cl_uchar4*          pOclImageData;
  cl_uchar4*          pVerImageData;
  cl_int2*            pQueuedPixels;
  RGSParamType*       pParams;

  cl_uint             pixelSize;
  cl_uint             width;
  cl_uint             height;
  
  cl_mem              oclImageBuf;
  cl_mem              oclQueuedPixels;
  cl_mem              oclParams;
  
  SDKTimer*           sampleTimer;
  
  bool                hostQ;
  double              totalKernelTime;
  
  SDKDeviceInfo       deviceInfo;
  
  /* cpu reference code class */
  RGS                 cpuRgs;
public:
  
  CLCommandArgs*      sampleArgs;
  
  RegionGrowingSegmentation()
  {
    sampleTimer = new SDKTimer();
    sampleArgs  = new CLCommandArgs();
    sampleArgs->sampleVerStr = SAMPLE_VERSION;
    pixelSize   = sizeof(cl_uchar4);
    hostQ       = false;
  }
  
  /**
   * Override from SDKSample, Generate binary image of given kernel
   * and exit application
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int genBinaryImage();
  
  /**
   * OpenCL related initialisations.
   * Set up Context, Device list, Command Queue, Memory buffers
   * Build CL kernel program executable
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int setupCL();
  /**
   * Set values for kernels' arguments, enqueue calls to the kernels
   * on to the command queue, wait till end of kernel execution.
   * Get kernel start and end time if timing is enabled
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int runCLKernels();
  
  /**
   * CPU verification
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int runRef();
  
  /**
   * Override from SDKSample. Print sample stats.
   */
  void printStats();
  
  /**
   * Override from SDKSample. Initialize
   * command line parser, add custom options
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int initialize();
  
  /**
   * Override from SDKSample, adjust width and height
   * of execution domain, perform all sample setup
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int setup();
  
  /**
   * Override from SDKSample
   * Run OpenCL BinarySearch
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int run();
  
  /**
   * Override from SDKSample
   * Cleanup memory allocations
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int cleanup();
  
  /**
   * Override from SDKSample
   * sampleArgs->verify against reference implementation
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int verifyResults();

  /**
   * Read the seed file
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int readSeedFile(std::string seedFileName);

  /**
   * Read the input image
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int readInputImage(std::string inputImageName);
  
  /**
   * Writes the device side and verification output image
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int writeOutputImage(std::string outputImageName, cl_uchar4* pImage);

  /*
   * Converts input image from RGB to YVU, and loads global variables.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int preProcess();

  /*
   * Converts output images from YUV to RGB. 
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int postProcess();

  /*
   * Compares device side and  and verification images.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   */
  int compareImages();
  
  /*
   * Converts segmented image's region information to a format
   * more suitable for diaplay.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   */
  int convertImage(cl_uchar4* pImage);

  /*
   * Converts an RGB image to YVU format
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   */
  int rgbToYvu(cl_uchar4* pImage);

  /*
   * Converts an YUV image to RGB format
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   */

  int yvuToRgb(cl_uchar4* pImage);
};

#endif
