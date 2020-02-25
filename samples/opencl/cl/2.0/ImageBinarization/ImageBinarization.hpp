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

#ifndef IMAGE_BINARIZATION_H_
#define IMAGE_BINARIZATION_H_


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"
#include "SDKBitMap.hpp"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-vx.y.z.s"

#define INPUT_IMAGE "ImageBinarization_Input.bmp"
#define OUTPUT_IMAGE "ImageBinarization_Output.bmp"

#define GROUP_SIZE 256
#define BIN_SIZE 256
#define NBANKS 16
#define PIXEL_PER_THREAD 8

#define LOCAL_SIZE_HISTOGRAM 256

#ifndef min
#define min(a, b)            (((a) < (b)) ? (a) : (b))
#endif

/**
* ImageBinarization
* Class implements OpenCL ImageBinarization sample
*/

class ImageBinarization
{
        cl_double setupTime;                /**< time taken to setup OpenCL resources and building kernel */
        cl_double kernelTime;               /**< time taken to run kernel and read result back */
		cl_uchar* inputImageDataGrayComponent; /**< Input bitmap data on host, but stores only Y component */
		cl_uchar* outputImageDataGrayComponent;/**< Output bitmap data on host, but stores only Y component */

		cl_uchar* refOutputBinarizationData;/**< Reference Output data for Image Binarization Kernel on host */
		cl_uint *hostBin;					/**< Host result for histogram bin */
		cl_uint *intermittentdeviceBinResult;/**< Intermittent sub-histogram bins */
		cl_uint *deviceBin;					/**< Device result for histogram bin */
        cl_context context;                 /**< CL context */
        cl_device_id *devices;              /**< CL device list */

        cl_mem image2D;						/**< CL image buffer for input as well as output Image*/
		cl_mem deviceImageData1D;			/**< CL buffer for storing image data in one dimentional array */
		cl_mem deviceBinResultBuffer;	    /**< CL buffer for storing histogram bins */
		cl_int threshold;					/**< Threshold for Binarization */

        cl_command_queue commandQueue;      /**< CL command queue */
        cl_program program;                 /**< CL program  */

		cl_kernel imageHistogramKernel;     /**< CL ImageHistogram kernel */
		cl_kernel imageBinarizationKernel;  /**< CL ImageBinarization kernel */
        

        SDKBitMap inputBitmap;				/**< Bitmap class object */
        uchar4* pixelData;					/**< Pointer to image data */
        cl_int  pixelSize;                  /**< Size of a pixel in BMP format> */
        cl_int  width;                      /**< Width of image */
        cl_int  height;                     /**< Height of image */
		cl_int  nPixelsPerThread;			/**< Number of pixels to be used per thread */
		cl_int  binSize;					/**< Size of Histogram Bins */
		cl_int  nBanks;						/**< Number of Banks */
		cl_int  subHistgCnt;				/**< Number of Histogram Counts */

		size_t globalWorkSizeHist;			/**< global work size for histogram kernel */
		size_t localThreadsHistogram;		/**< local work size for histogram kernel */
        size_t blockSizeX;                  /**< Work-group size in x-direction */
        size_t blockSizeY;                  /**< Work-group size in y-direction */
		size_t binarizationkernelWorkGroupSize;
		size_t origin[3];
		size_t region[3];

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
        int writeOutputImage(std::string outputImageName);

        /**
        * Constructor
        * Initialize member variables
        */
        ImageBinarization()
            : inputImageDataGrayComponent(NULL),
			  outputImageDataGrayComponent(NULL),
			  hostBin(NULL),
			  intermittentdeviceBinResult(NULL),
			  deviceBin(NULL),
			  refOutputBinarizationData(NULL),
			  subHistgCnt(1)
        {
            sampleArgs = new CLCommandArgs() ;
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            pixelSize = sizeof(uchar4);
            pixelData = NULL;
            blockSizeX = GROUP_SIZE;
            blockSizeY = 1;
			localThreadsHistogram = LOCAL_SIZE_HISTOGRAM;
			globalWorkSizeHist = localThreadsHistogram;
            iterations = 1;
			binSize = BIN_SIZE;
			nBanks = NBANKS;
			nPixelsPerThread = PIXEL_PER_THREAD;
			threshold = 0;
            imageFormat.image_channel_data_type = CL_UNSIGNED_INT8;
            imageFormat.image_channel_order = CL_R;

			context					= NULL;                 
            image2D						= NULL;
			deviceImageData1D			= NULL;
			deviceBinResultBuffer	    = NULL;
			commandQueue		= NULL;
			program					= NULL;
			imageHistogramKernel		= NULL;
			imageBinarizationKernel	= NULL;
        }

        ~ImageBinarization()
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
        * Reference CPU implementation of Binomial Option
        * for performance comparison
        */
        void ImageBinarizationCPUReference();

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
        *  Calculate histogram bin on host
        */
        void calculateHostBin();

		/**
		*  Calculate Threshold value using Otsu's algorithm on host
		*/
		int calculateThresholdValueOnHost();
};

#endif // IMAGE_BINARIZATION_H_
