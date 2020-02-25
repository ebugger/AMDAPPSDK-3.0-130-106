/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/


/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include <math.h>
#include <malloc.h>
#include "bolt/cl/transform.h"
#include "RgbToYuv.hpp"


/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int RgbToYuv::setup()
{
    rgbData.resize(sampleArgs->samples);
    boltYuv.resize(sampleArgs->samples);
    cpuYuv.resize(sampleArgs->samples);

    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    for(int i = 0; i < sampleArgs->samples; i++)
    {
        rgbData[i].r = (char)((float)rand() / (float)RAND_MAX);
        rgbData[i].g = (char)((float)rand() / (float)RAND_MAX);
        rgbData[i].b = (char)((float)rand() / (float)RAND_MAX);
    }

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


    if(!sampleArgs->quiet)
    {
        std::cout << "Completed setup() of RgbToYuv sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int RgbToYuv::run()
{
    for(unsigned i = 0; i < 1 && sampleArgs->iterations != 1; i++)
    {
        if(rgbToYuvBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
        if(!sampleArgs->quiet)
        {
            std::cout << "Completed Warm up run of Bolt code" << std::endl;
        }
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Executing RgbToYuv sample over " << sampleArgs->iterations
                  << " iteration(s)." << std::endl;
    }

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < sampleArgs->iterations; i++)
    {

        if(rgbToYuvBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    sampleTimer->totalTime = (double)(sampleTimer->readTimer(timer));

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of RgbToYuv sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int RgbToYuv::verifyResults()
{
    if(sampleArgs->verify)
    {
        rgbToYuvCPU();

        if(!sampleArgs->quiet)
        {
            std::cout << "\nComparing resulting data..." << std::endl;
        }

        /**********************************************************************
        * Compare the resulting data vectors to ensure we did not goof up     *
        **********************************************************************/
        bool priceResult = compare();

        std::cout << "YUV 4:4:4 using Bolt : ";

        if(!priceResult)
        {
            std::cout << "Data MISMATCH\n" << std::endl;
            return SDK_FAILURE;
        }
        std::cout << "Data matches with reference\n" << std::endl;
        std::cout << "Passed!\n" << std::endl;
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed verifyResults() of RgbToYuv sample"
                  << std::endl;
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void RgbToYuv::printStats()
{
    if(sampleArgs->timing)
    {
        double avgTime = sampleTimer->totalTime/sampleArgs->iterations;

        std::string strArray[3] = {"Number of samples", "Avg Time (s)", "Speed (GB/s)"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for RgbToYuv()\n" << std::endl;
        stats[1] = toString(avgTime);
        stats[2] = toString(sampleArgs->samples / avgTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed printStats() of RgbToYuv sample"<<std::endl;
    }
}


/******************************************************************************
* Implementation of rgbToYuvCPU()                                             *
******************************************************************************/
int RgbToYuv::rgbToYuvCPU()
{
    std::transform(rgbData.begin(), rgbData.end(),
                   cpuYuv.begin(), rgbToYuvFunctor());
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of rgbToYuvBOLT()                                            *
******************************************************************************/
int RgbToYuv::rgbToYuvBOLT()
{
    bolt::cl::transform(rgbData.begin(), rgbData.end(),
                        boltYuv.begin(), rgbToYuvFunctor());
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of compare()                                                 *
******************************************************************************/
bool RgbToYuv::compare()
{
    bool passed = true;
    unsigned totalMismatches = 0;

    for (int i = 0; i < sampleArgs->samples; ++i)
    {
        if (cpuYuv[i].y != boltYuv[i].y || cpuYuv[i].u != boltYuv[i].u
                || cpuYuv[i].v != boltYuv[i].v)
        {
            if (!totalMismatches)
            {
                std::cout << "First mismatch found"  << std::endl << "cpuYuv["
                          << i << "].y = " << cpuYuv[i].y << ", boltYuv[" << i
                          << "].y = " << boltYuv[i].y << std::endl << "cpuYuv[" << i
                          << "].u = " << cpuYuv[i].u << ", boltYuv[" << i << "] = "
                          << boltYuv[i].u << std::endl << "cpuYuv[" << i << "].v = "
                          << cpuYuv[i].v << ", boltYuv[" << i << "].v = "
                          << boltYuv[i].v << std::endl;
                passed = false;
            }
            totalMismatches++;
        }
    }

    if (totalMismatches)
        std::cout << "Total number of mismatches found = " << totalMismatches
                  << std::endl;

    return (totalMismatches == 0);
}

/******************************************************************************
* Execution of program begins from here                                       *
******************************************************************************/
int main(int argc, char * argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "RgbToYuv using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    /**************************************************************************
    * Create an object of RgbToYuv class                                      *
    **************************************************************************/
    RgbToYuv objRgbToYuv;

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objRgbToYuv.sampleArgs->parseCommandLine(argc, argv))
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Bolt APIs throw exception on failure                                    *
    **************************************************************************/
    try
    {
        /**************************************************************************
        * Initialize the random array of input samples                            *
        **************************************************************************/
        if(objRgbToYuv.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute BlackScholes algorithm on the input samples                     *
        **************************************************************************/
        if(objRgbToYuv.run() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    catch(::cl::Error err)
    {
        std::cout << "Exception encountered with message: " << std::endl
                  << err.what() << std::endl << "Exiting...." << std::endl;
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Verify the results that were generated                                  *
    **************************************************************************/
    if(objRgbToYuv.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objRgbToYuv.printStats();

    return 0;
}
