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

#ifndef _SIMPLE_DEPTH_IMAGE_H_
#define _SIMPLE_DEPTH_IMAGE_H_

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"
#include "SDKBitMap.hpp"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

/**
 * Constants
 */

#define INPUT_IMAGE  "SimpleDepthImage_Input.bmp"
#define OUTPUT_IMAGE "SimpleDepthImage_Output.bmp"
#define REF_IMAGE    "SimpleDepthImage_Ref.bmp"

#define MIN_LUMA             0
#define MAX_LUMA             255
#define DEFAULT_THRESHOLD    127
#define BIN_ONE              0.0
#define BIN_TWO              0.9
#define F_EPSILON            (1E-05)

/**
 * Macros
 */

#define F_ABS(x)             (((x) > (0.0)) ? (x):-(x))

/**
* SimpleDepthImage
* Class implements OpenCL 2.0 feature of depth image.
* This sample shows how depth images could be manipulated inside an OpenCL kernel. 
* A monochrome image is read and its luma is passed to an OpenCL kernel as depth 
* information using a depth image. The kernel manipulates depth values at each 
* pixel (a simple binarization of each depth value in this sample). This 
* manipulated depth image is passed back to the host. Host reinterprets the depth
* information as luma, and displays manipulated image.   
*/

class SimpleDepthImage
{
  /* OpenCL runtime */
  cl_context            context;                 
  cl_device_id*         devices;              
  cl_command_queue      commandQueue;      
  cl_program            program;                 
  
  cl_kernel             simpleDepthKernel;  

  SDKDeviceInfo         deviceInfo;                    
  KernelWorkGroupInfo   kernelInfo;              

  /* host image containers */ 
  cl_float*             inputImageData;
  cl_float*             outputImageData;
  cl_float*             refOutputImageData;

  /* device image containers */
  cl_mem                oclImage;

  /* threshold for binarization */
  cl_int                threshold;                                      

  /* image read-write */
  SDKBitMap             inputBitmap;                            
  uchar4*               pixelData;                              

  size_t                origin[3];
  size_t                region[3];

  /* image parameters */
  cl_int                pixelSize;                  
  cl_int                width;                      
  cl_int                height;                     

  /* performance measurement */
  int                   iterations;                     
  cl_double             setupTime;                
  cl_double             kernelTime;               
  SDKTimer*             sampleTimer;        


  /* Orphaned variables */
  
  size_t blockSizeX;                  
  size_t blockSizeY;                  

  cl_bool imageSupport;               
  
public:
  
  CLCommandArgs*   sampleArgs;   
  
  /**
   * Read bitmap image and allocate host memory
   * @param inputImageName name of the input file
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int readInputImage(std::string inputImageName);
  
  /**
   * Write to an image file
   * @param outputImageName name of the output file
   * @param imageData  data pointer of the image
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int writeOutputImage(cl_float* imageData, std::string outputImageName);
  
  /**
   * Constructor
   * Initialize member variables
   */
  SimpleDepthImage()
  {
    sampleArgs                 = new CLCommandArgs() ;
    sampleTimer                = new SDKTimer();
    sampleArgs->sampleVerStr   = SAMPLE_VERSION;
    pixelSize                  = sizeof(uchar4);
    pixelData                  = NULL;


    inputImageData             = NULL;
    outputImageData            = NULL;
    refOutputImageData         = NULL;


    threshold                  = DEFAULT_THRESHOLD;
        iterations                 = 1;
  }
  
  ~SimpleDepthImage()
  {
  }
  
  /**
   * Override from SDKSample, Generate binary image of given kernel
   * and exit application
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
   * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int runCLKernels();

  /**
   * Reference CPU implementation for performance comparison
   */
  void CPUReference();

  /**
   * Override from SDKSample. Print sample stats.
   */
  void printStats();
  
  /**
   * Override from SDKSample. Initialize
   * command line parser, add custom options
   */
  int initialize();
  
  /**
   * Override from SDKSample, adjust width and height
   * of execution domain, perform all sample setup
   * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int setup();

  /**
   * Override from SDKSample
   * Run OpenCL SimpleImage
   * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int run();
  
  /**
   * Override from SDKSample
   * Cleanup memory allocations
   * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int cleanup();
  
  /**
   * Override from SDKSample
   * Verify against reference implementation
   * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
   */
  int verifyResults();
  
};

#endif //_SIMPLE_DEPTH_IMAGE_H_
