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

#ifndef SIMPLE_GENERIC_ADDRESS_SPACE_H_
#define SIMPLE_GENERIC_ADDRESS_SPACE_H_


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"
#include "SDKBitMap.hpp"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

#define INPUT_IMAGE "SimpleGenericAddressSpace_Input.bmp"
#define OUTPUT_IMAGE_GLOBAL "GlobalOutput_Convolution.bmp"
#define OUTPUT_IMAGE_LOCAL "LocalOutput_SepiaToning.bmp"

#define FILTER_HEIGHT 3
#define FILTER_WIDTH 3

// initializing filter data 
const float filterData[FILTER_HEIGHT*FILTER_WIDTH] =
{
       -1.0f, 0.0f, 1.0f,
       -2.0f, 0.0f, 2.0f,
       -1.0f, 0.0f, 1.0f
};

/**
* SimpleGenericAddressSpace
* Class implements OpenCL SimpleGenericAddressSpace sample
*/

class SimpleGenericAddressSpace
{
        cl_double setupTime;                /**< time taken to setup OpenCL resources and building kernel */
		cl_double ConvolutionGlobalKernelTime; /**< time taken to run kernel and read result back */
		cl_double sepiaToningLocalKernelTime;  /**< time taken to run kernel and read result back */
        cl_uchar4* inputImageData2D;         /**< Input bitmap data on host */
		cl_uchar4* outputImageData2DGlobal;  /**< Output data on host for Convolution Kernel using Global Address Space */
		cl_uchar4* outputImageData2DLocal;   /**< Output data on host for convolution Kernel using Local Address Space */
		cl_uchar* verificationConvolutionOutput;/**< Reference Output data for convolution Kernel on host */
		cl_uchar* verificationSepiaToningOutput;  /**< Reference Output data for sepiaToning Kernel on host */
	
		cl_context context;                 /**< CL context */
        cl_device_id *devices;              /**< CL device list */

		cl_mem deviceInputBuffer;			/**< CL buffer for storing input image data */
		cl_mem deviceOutputBuffer;			/**< CL buffer for storing output image data */
		cl_mem devicefilterBuffer;			/**< CL buffer for storing filter data */
		cl_int filterHeight;
		cl_int filterWidth;
		cl_int2 filterDim;
		cl_int2 filterRadius;
		cl_int2 padding;

        cl_command_queue commandQueue;      /**< CL command queue */
        cl_program program;                 /**< CL program  */

		cl_kernel convolutionGlobalKernel;  /**< CL convolutionGlobal kernel */
		cl_kernel sepiaToningLocalKernel;   /**< CL convolutionLocal kernel */
       
		SDKBitMap inputBitmap;				/**< Bitmap class object */
        uchar4* pixelData;					/**< Pointer to image data */
        cl_uint pixelSize;                  /**< Size of a pixel in BMP format> */
        cl_int  width;						/**< Width of image */
        cl_int  height;						/**< Height of image */
        size_t blockSizeX;                  /**< Work-group size in x-direction */
        size_t blockSizeY;                  /**< Work-group size in y-direction */
		size_t localSize[2];				/**< Local Work-group size for convolution kernel */
		size_t globalSize[2];				/**< Global Work-group size for convolution kernel */
		size_t localWidth;					/**< Width of shared memory image */
		size_t localHeight;					/**< Height of shared memory image */
		
        int iterations;                     /**< Number of iterations for kernel execution */
        cl_bool imageSupport;               /**< Flag to check whether images are supported */
        cl_image_format imageFormat;        /**< Image format descriptor */
        SDKDeviceInfo
        deviceInfo;                    /**< Structure to store device information*/
        KernelWorkGroupInfo
        kernelInfo;              /**< Structure to store kernel related info */

        SDKTimer    *sampleTimer;      /**< SDKTimer object */


    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        /**
        * Read bitmap image and allocate host memory
        * @param inputImageName name of the input file
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int readInputImage(std::string inputImageName);

        /**
        * Write to an image file
        * @param outputImageName name of the output file
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int writeOutputImage(std::string outputImageName, cl_uchar4 *outputImageData);

        /**
        * Constructor
        * Initialize member variables
        */
        SimpleGenericAddressSpace()
			: inputImageData2D(NULL),
			  outputImageData2DGlobal(NULL),
			  outputImageData2DLocal(NULL),
			  verificationConvolutionOutput(NULL),
			  verificationSepiaToningOutput(NULL)
        {
            sampleArgs = new CLCommandArgs() ;
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
			pixelSize = sizeof(uchar4);
			pixelData = NULL;
            blockSizeX = 16;
            blockSizeY = 16;
            iterations = 1;
			filterHeight = FILTER_HEIGHT;
			filterWidth = FILTER_WIDTH;
        }

        ~SimpleGenericAddressSpace()
        {
        }


		/**
		* Calculates the value of WorkGroup Size based in global NDRange
		* and kernel properties
		* @return 0 on success and nonzero on failure
		*/
		int setWorkGroupSize();


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
        int runConvolutionGlobalKernels();

		/**
        * Set values for kernels' arguments, enqueue calls to the kernels
        * on to the command queue, wait till end of kernel execution.
        * Get kernel start and end time if timing is enabled
        * @return  SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int runSepiaToningLocalKernels();

		/**
        * This function roundUp a given number to the nearest multiple of
		* provided number.
		* @param value the number to be rounded up
		* @param multiple rounded number should be multiple of this
		* @return rounded value
        */
        int roundUp(int value, unsigned int multiple);

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

	private:

		/**
		*  Calculate 
		*  convoultion on host for 1st kernel then
		*  calculate sepiaToning on convoluted image
		*/
		int CPUReference();
};

#endif // SIMPLE_GENERIC_ADDRESS_SPACE_H_
