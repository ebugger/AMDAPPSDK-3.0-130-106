/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

• Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
• Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include "BoxFilterSAT.hpp"
#include "bolt/cl/scan.h"
#include "bolt/cl/copy.h"
#include "bolt/cl/transform.h"
#include "bolt/cl/device_vector.h"
#include "bolt/cl/scan_by_key.h"
#include "bolt/cl/sort_by_key.h"
#include "bolt/cl/iterator/counting_iterator.h"

unsigned char 
BoxFilterSAT::clampToUchar(int n)
{
   n = ( (n < 0) ? 0 : ((n > 255) ? 255 : n) );
   return (unsigned char)(n);
}

/******************************************************************************
* Implementation of initialize()                                              *
******************************************************************************/
int BoxFilterSAT::init()
{
    Option* filter_width = new Option;
    CHECK_ALLOCATION(filter_width, "Memory Allocation error.\n");

    filter_width->_sVersion = "w";
    filter_width->_lVersion = "width";
    filter_width->_description = "Filter width";
    filter_width->_type = CA_ARG_INT;
    filter_width->_value = &filterWidth;

    sampleArgs->AddOption(filter_width);
    delete filter_width;

    // Since we are reading the pixels from input image, removing the -x or
    // samples option from the argument list.
    Option* samples = new Option;
    CHECK_ALLOCATION(sampleArgs->samples, "Memory Allocation error.\n");

    samples->_sVersion = "x";
    samples->_lVersion = "sampleArgs->samples";
    samples->_type = CA_ARG_INT;

    sampleArgs->DeleteOption(samples);
    delete samples;

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int BoxFilterSAT::setup()
{
    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    if( !sampleArgs->enable_tbb &&
            (strComparei(sampleArgs->runMode, "multicorecpu")) )
    {
        std::cout << "\nError:Use TBB configuration to run in multi-core CPU";
        return SDK_FAILURE;
    }

    bolt::cl::control::e_RunMode specifiedRunMode =
        (strComparei(sampleArgs->runMode, "opencl"))? (bolt::cl::control::OpenCL) :
        ((strComparei(sampleArgs->runMode,
                      "serialcpu"))? (bolt::cl::control::SerialCpu) :
         (((strComparei(sampleArgs->runMode,
                        "multicorecpu"))? (bolt::cl::control::MultiCoreCpu) :
           (bolt::cl::control::Automatic))));

    sampleArgs->boltControlObj->setForceRunMode(specifiedRunMode);
    sampleArgs->displayRunmodeInfo();



    // load input bitmap image
    std::string filePath = getPath() + std::string(INPUT_IMAGE);
    inputBitmap.load(filePath.c_str());
    if(!inputBitmap.isLoaded())
    {
        std::cout << "Failed to load input image!";
        return SDK_FAILURE;
    }
    // get the pointer to pixel data
    pixelData = inputBitmap.getPixels();
    if(NULL == pixelData)
    {
        std::cout << "Failed to read pixel Data!";
        return SDK_FAILURE;
    }
    // get width and height of input image
    height = inputBitmap.getHeight();
    width = inputBitmap.getWidth();
    sampleArgs->samples = width * height;

    // allocate memory for output image data and initializa to NULL
    outputImageData = (cl_uchar4*)malloc(sampleArgs->samples * sizeof(cl_uchar4));
    CHECK_ALLOCATION(outputImageData,
                     "Failed to allocate memory! (outputImageData)")
    memset(outputImageData, 0, sampleArgs->samples * sizeof(cl_uchar4));

    // allocate memory for verification output and initialize to NULL
    verificationOutput = (cl_uchar4*)malloc(sampleArgs->samples * sizeof(
            cl_uchar4));
    CHECK_ALLOCATION(verificationOutput,
                     "verificationOutput heap allocation failed!");
    memset(verificationOutput, 0, sampleArgs->samples * sizeof(cl_uchar4));

    // Resize the device buffers
    inputImgDeviceBuffer.resize(sampleArgs->samples);
    outputImageBuffer.resize(sampleArgs->samples);
    inputDeviceBuffer.resize(sampleArgs->samples);
    tempDeviceBuffer.resize(sampleArgs->samples);
    pixelKeyBuffer.resize(sampleArgs->samples);

    int status = setupCL();
    CHECK_BOLT_ERROR(status, "setupCL Failed");

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of setupCL()                                                 *
******************************************************************************/
int BoxFilterSAT::setupCL()
{
    cl_int status = 0;
    bolt::cl::control &defaultControl = bolt::cl::control::getDefault();
    cl_context context = (defaultControl.getContext())();
    cl_device_id deviceId = (defaultControl.getDevice())();
    commandQueue = (defaultControl.getCommandQueue())();

    // Read the CL file
    std::string KernelSource;
    if(readCLFile("BoxFilterSAT_Kernels.cl", KernelSource) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Create the compute program from the source buffer
    cl_int err;
    cl_program program;
    const char *source = KernelSource.c_str();
    program = clCreateProgramWithSource(context, 1,
                                        (const char **)&source,
                                        NULL, &err);

    if (!program)
    {
        std::cout << "Error: Failed to create compute program!" << std::endl;
        return SDK_FAILURE;
    }

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[2048];
        std::cout << "Error: Failed to build program executable!" << std::endl;
        clGetProgramBuildInfo(program, deviceId, CL_PROGRAM_BUILD_LOG,
                              sizeof(buffer), buffer, &len);
        std::cout << buffer << std::endl;
        return SDK_FAILURE;
    }


    // Create a kernel object for box filter
    boxFilter = clCreateKernel(program, "boxFilter", &status);
    CHECK_BOLT_ERROR(status, "clCreateKernel failed.(boxFilter)");

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int BoxFilterSAT::run()
{
    // Copy input image from host to device
    {
        bolt::cl::device_vector<UChar4>::pointer inputImgDeviceBufferPtr =
            inputImgDeviceBuffer.data();
        ::memcpy(inputImgDeviceBufferPtr.get(), pixelData,
                 width * height * sizeof(UChar4));
    }

    for(unsigned i = 0; i < 1 && sampleArgs->iterations != 1; i++)
    {
        if(boxFilterSATBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
        if(!sampleArgs->quiet)
        {
            std::cout << "Completed Warm up run of Bolt code" << std::endl;
        }
    }

    if(!sampleArgs->quiet)
        std::cout << "Executing BoxFilterSAT sample over " << sampleArgs->iterations
                  << " iteration(s)." << std::endl;

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);
    for(int i = 0; i < sampleArgs->iterations; i++)
    {

        if(boxFilterSATBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }
    sampleTimer->stopTimer(timer);
    sampleTimer->totalTime = (double)(sampleTimer->readTimer(timer));
    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of BoxFilterSAT sample" << std::endl;
    }
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of BoxFilterSATBOLT()                                        *
******************************************************************************/
int BoxFilterSAT::boxFilterSATBOLT()
{
    // Transform the each pixel values(bytes to unsigned int)
    bolt::cl::transform(inputImgDeviceBuffer.begin(),
                        inputImgDeviceBuffer.end(),
                        inputDeviceBuffer.begin(),
                        UChar4ToUInt4());

    // Horizontal scan : Make "key = row number (with transform)" and then scan by key
    bolt::cl::transform(pixelKeyBuffer.begin(),
                        pixelKeyBuffer.end(),
                        bolt::cl::make_counting_iterator(0),
                        pixelKeyBuffer.begin(),
                        GetRow(width));

    bolt::cl::inclusive_scan_by_key(pixelKeyBuffer.begin(),
                                    pixelKeyBuffer.end(),
                                    inputDeviceBuffer.begin(),
                                    inputDeviceBuffer.begin());


    cl_mem input = (inputDeviceBuffer.getBuffer())();
    cl_mem output = (tempDeviceBuffer.getBuffer())();

    bolt::cl::device_vector<unsigned> map(sampleArgs->samples);

    bolt::cl::transform(map.begin(),
                        map.end(),
                        bolt::cl::make_counting_iterator(0),
                        map.begin(),
                        GetTransposedIndex(width,height));

    bolt::cl::gather(map.begin(),
                     map.end(),
                     inputDeviceBuffer.begin(),
                     tempDeviceBuffer.begin());

     
    // Vertical scan
    bolt::cl::inclusive_scan_by_key(pixelKeyBuffer.begin(),
                                    pixelKeyBuffer.end(),
                                    tempDeviceBuffer.begin(),
                                    tempDeviceBuffer.begin());

    // Run boxFilter CL Kernel on SAT data
    int status = runBoxFilterCLKernel();
    CHECK_BOLT_ERROR(status, "runBoxFilterCLKernel Failed");

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of runBoxFilterCLKernel()                                    *
******************************************************************************/
int BoxFilterSAT::runBoxFilterCLKernel()
{
    // input buffer
    cl_mem input = (tempDeviceBuffer.getBuffer())();
    int status = clSetKernelArg(boxFilter,
                                0,
                                sizeof(cl_mem),
                                &input);

    CHECK_BOLT_ERROR(status, "clSetKernelArg failed. (input)");

    // output buffer
    cl_mem output = (outputImageBuffer.getBuffer())();
    status = clSetKernelArg(boxFilter,
                            1,
                            sizeof(cl_mem),
                            &output);
    CHECK_BOLT_ERROR(status, "clSetKernelArg failed.(output)");

    // current pass
    status = clSetKernelArg(boxFilter,
                            2,
                            sizeof(cl_uint),
                            &filterWidth);
    CHECK_BOLT_ERROR(status, "clSetKernelArg failed. (pass)");

    size_t globalThreads[] = { width, height, 1 };
    size_t localThreads[]  = { GROUP_SIZE, 1, 1 };


	// To verify whether device supports the specified workgroup size(GROUP_SIZE) or not ?;
	
	 bolt::cl::control &defaultControl = bolt::cl::control::getDefault();
     cl_device_id deviceId = (defaultControl.getDevice())();
	 size_t Device_Max_Work_Group_Size=0 ;
	
	 status = clGetDeviceInfo(
                         deviceId,
                         CL_DEVICE_MAX_WORK_GROUP_SIZE,
                         sizeof(size_t),
                         (void *)&Device_Max_Work_Group_Size,
                         NULL);
     CHECK_BOLT_ERROR(status, "clGetDeviceIDs(CL_DEVICE_MAX_WORK_GROUP_SIZE) failed");
	 
	if(localThreads[0] > Device_Max_Work_Group_Size)
		localThreads[0] =  Device_Max_Work_Group_Size ;


    cl_event ndrEvt;
    status = clEnqueueNDRangeKernel(commandQueue,
                                    boxFilter,
                                    3,
                                    NULL,
                                    globalThreads,
                                    localThreads,
                                    0,
                                    NULL,
                                    &ndrEvt);
    CHECK_BOLT_ERROR(status, "clEnqueueNDRangeKernel failed.");

    status = clFlush(commandQueue);
    CHECK_BOLT_ERROR(status, "clFlush failed.");

    status = sampleArgs->waitForEventAndRelease(&ndrEvt);
    CHECK_BOLT_ERROR(status, "WaitForEventAndRelease(ndrEvt) Failed");

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of writeOutputImage()                                        *
******************************************************************************/
int BoxFilterSAT::writeOutputImage(std::string outputImageName)
{
    // copy output image data back to original pixel data
    memcpy(pixelData, outputImageData, width * height * sizeof(uchar4));

    // write the output bmp file
    if(!inputBitmap.write(outputImageName.c_str()))
    {
        std::cout << "Failed to write output image!";
        return SDK_FAILURE;
    }
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int BoxFilterSAT::verifyResults()
{
    // Copy the results from device to host.
    {
        bolt::cl::device_vector<UChar4>::pointer outputImageBufferPtr =
            outputImageBuffer.data();
        ::memcpy(outputImageData, outputImageBufferPtr.get(),
                 width * height * sizeof(cl_uchar4));
    }

    if(sampleArgs->verify)
    {
        boxFilterSATCPUReference();

        // Compare between outputImageData and verificationOutput
        if(memcmp(outputImageData,
                  verificationOutput,
                  width * height * sizeof(cl_uchar4)))
        {
            std::cout << "Failed!" << std::endl;
            return SDK_FAILURE;
        }
        std::cout << "Passed!" << std::endl;

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed verifyResults() of BoxFilterSAT sample"
                      << std::endl;
        }
    }

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of boxFilterSATCPUReference()                                         *
******************************************************************************/
void BoxFilterSAT::boxFilterSATCPUReference()
{
    int t = (filterWidth - 1) / 2;
    int filterSize = filterWidth * filterWidth;
    cl_uchar4* inputImageData = (cl_uchar4*)pixelData;
    for(int y = 0; y < (int)height; y++)
    {
        for(int x = 0; x < (int)width; x++)
        {
            // Only threads inside apron will calculate their pixel value
            if(x >= t && x < (int)(width - t) && y >= t && y < (int)(height - t))
            {
                cl_int4 sum = {0, 0, 0, 0};
                // For all pixels inside box
                for(int y1 = -t; y1 <= t; y1++)
                {
                    for(int x1 = -t; x1 <= t; x1++)
                    {
                        sum.s[0] += inputImageData[x + x1 + (y + y1) * width].s[0];
                        sum.s[1] += inputImageData[x + x1 + (y + y1) * width].s[1];
                        sum.s[2] += inputImageData[x + x1 + (y + y1) * width].s[2];
                        sum.s[3] += inputImageData[x + x1 + (y + y1) * width].s[3];
                    }
                }
                verificationOutput[x + y * width].s[0] = clampToUchar(sum.s[0] / filterSize);
                verificationOutput[x + y * width].s[1] = clampToUchar(sum.s[1] / filterSize);
                verificationOutput[x + y * width].s[2] = clampToUchar(sum.s[2] / filterSize);
                verificationOutput[x + y * width].s[3] = clampToUchar(sum.s[3] / filterSize);
            }
        }
    }
}

/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void BoxFilterSAT::printStats()
{
    if(sampleArgs->timing)
    {
        double avgTime = sampleTimer->totalTime/sampleArgs->iterations;

        std::string strArray[3] = {"Number of pixels", "Avg Time (s)", "Pixels / sec"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for BoxFilterSAT()\n" << std::endl;
        stats[1] = toString(avgTime);
        stats[2] = toString(sampleArgs->samples / avgTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed printStats() of BoxFilterSAT sample"<<std::endl;
        }
    }
}

/******************************************************************************
* Implementation of readCLFile()                                              *
******************************************************************************/
int BoxFilterSAT::readCLFile(const std::string& filename, std::string &str)
{
    std::ifstream file;
    file.exceptions(std::ifstream::in|std::ifstream::badbit);
    file.open(filename.c_str());
    if(! file.good())
    {
        std::cout << "\n Unable to open CL file: " << filename.c_str() << std::endl;
        return SDK_FAILURE;
    }
    str = std::string(std::istreambuf_iterator<char>(file),
                      std::istreambuf_iterator<char>());
    file.close();
    return SDK_SUCCESS;
}

/******************************************************************************
* Execution of program begins from here                                       *
******************************************************************************/
int main(int argc, char * argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "BoxFilterSAT using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    BoxFilterSAT clBoxFilterSAT;

    if(clBoxFilterSAT.init() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBoxFilterSAT.sampleArgs->parseCommandLine(argc, argv))
    {
        return SDK_FAILURE;
    }

    try
    {
        if(clBoxFilterSAT.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(clBoxFilterSAT.run() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    catch(::cl::Error err)
    {
        std::cout << "Exception encountered with message: \n"
                  << err.what() << "\nExiting..." << std::endl;
        return SDK_FAILURE;
    }

    if(clBoxFilterSAT.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if(clBoxFilterSAT.writeOutputImage(OUTPUT_IMAGE) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    clBoxFilterSAT.printStats();

    return 0;
}
