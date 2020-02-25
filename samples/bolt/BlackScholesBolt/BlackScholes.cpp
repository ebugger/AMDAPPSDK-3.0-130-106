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
#include <string>
#include "bolt/unicode.h"
#include "bolt/cl/control.h"
#include "bolt/cl/transform.h"
#include "bolt/statisticalTimer.h"
#include "BlackScholes.hpp"


/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int BlackScholes::setup()
{
    cpuOptions.resize(sampleArgs->samples);
    boltOptions.resize(sampleArgs->samples);
    randOptions.resize(sampleArgs->samples);
    boltPrice.resize(sampleArgs->samples);
    cpuPrice.resize(sampleArgs->samples);


    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    for(int i = 0; i < sampleArgs->samples; i++)
    {
        randOptions[i] = (float)rand() / (float)RAND_MAX;
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
        std::cout << "Completed setup() of BlackScholes sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int BlackScholes::run()
{
    for(unsigned i = 0; i < 1 && sampleArgs->iterations != 1; i++)
    {
        if(blackScholesBOLT() != SDK_SUCCESS)
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
        std::cout << "Executing BlackScholes sample over " << sampleArgs->iterations
                  << " iteration(s)." << std::endl;
    }

    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < sampleArgs->iterations; i++)
    {

        if(blackScholesBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    sampleArgs->totalTime = (double)(sampleTimer->readTimer(timer));

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of BlackScholes sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int BlackScholes::verifyResults()
{
    if(sampleArgs->verify)
    {
        blackScholesCPU();

        if(!sampleArgs->quiet)
        {
            std::cout << "\nComparing resulting prices..." << std::endl;
        }

        /**********************************************************************
        * Compare the resulting price vectors to ensure we did not goof up    *
        **********************************************************************/
        bool priceResult = compare();

        std::cout << "Call & Put price : ";

        if(!priceResult)
        {
            std::cout << "Data MISMATCH\n" << std::endl;
            return SDK_FAILURE;
        }
        std::cout << "Data matches with reference\n" << std::endl;
        std::cout << "Passed!\n" << std::endl;
    }

    if(!sampleArgs->quiet)
        std::cout << "Completed verifyResults() of BlackScholes sample"
                  << std::endl;

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void BlackScholes::printStats()
{
    if(sampleArgs->timing)
    {

        double avgTime = sampleArgs->totalTime/sampleArgs->iterations;

        std::string strArray[3] = {"Number of samples", "Avg Time (s)", "Speed (Samples/sec)"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for BlackScholes()\n" << std::endl;
        stats[1] = toString(avgTime);
        stats[2] = toString(sampleArgs->samples / avgTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed printStats() of BlackScholes sample"<<std::endl;
    }
}


/******************************************************************************
* Implementation of blackScholesCPU()                                         *
******************************************************************************/
int BlackScholes::blackScholesCPU()
{
    /**************************************************************************
    * For every loop iteration, we copy over fresh random data to the input   *
    * vector.  There is no need on the CPU device to make a copy of the       *
    * output data, as it already resides in host accessible memory.           *
    **************************************************************************/
    ::memcpy(&cpuOptions[0], &randOptions[0], sizeof(float) * sampleArgs->samples);

    std::transform(cpuOptions.begin(), cpuOptions.end(),
                   cpuPrice.begin(), blackScholesFunctor());

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of blackScholesBOLT()                                        *
******************************************************************************/
int BlackScholes::blackScholesBOLT()
{
    /**************************************************************************
    * The .data() method is called to map GPU memory to device memory         *
    * Retrieve a pointer to device memory and fill it with randomly           *
    * generated data.  Explicitely release the memory before calling into     *
    * bolt, which causes the updated data to transfer to the OpenCL device    *
    **************************************************************************/
    bolt::cl::device_vector<float>::pointer optionsPtr = boltOptions.data();

    ::memcpy(optionsPtr.get(), &randOptions[0],
             sizeof(float) * sampleArgs->samples);
    optionsPtr.reset();

    bolt::cl::transform(boltOptions.begin(), boltOptions.end(),
                        boltPrice.begin(), blackScholesFunctor());

    /**************************************************************************
    * The .data() method is called to map GPU memory to device memory         *
    * The following call brings the output data from device to host, and then *
    * the memory is unmapped implicitly when the pointer goes out of scope    *
    * The const reference object cboltPrice is created to be able to call the *
    * const version of the .data() method.  This is advantageous because we   *
    * only wish to read the output data, and there is no need to upload the   *
    * array back the cl device when the pointer unmaps                        *
    **************************************************************************/
    const bolt::cl::device_vector<blackScholesPrice>& cboltPrice = boltPrice;
    bolt::cl::device_vector<blackScholesPrice>::const_pointer pricePtr =
        cboltPrice.data();

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of compare()                                                 *
******************************************************************************/
bool BlackScholes::compare()
{
    float callError = 0.0f;
    float putError  = 0.0f;
    float callRef = 0.0f;
    float putRef  = 0.0f;
    const float epsilon = 1e-4f;

    /**************************************************************************
    * The .data() method of device_vector<> is called to map the pointer to   *
    * host memory                                                             *
    **************************************************************************/
    auto boltData = boltPrice.data();

    for(int i = 1; i < (int)cpuPrice.size(); ++i)
    {
        float diff = cpuPrice[i].callPrice - boltData[i].callPrice;
        callError += diff * diff;
        callRef   += cpuPrice[i].callPrice * cpuPrice[i].putPrice;

        diff = cpuPrice[i].putPrice - boltData[i].putPrice;
        putError += diff * diff;
        putRef   += cpuPrice[i].putPrice * cpuPrice[i].putPrice;
    }

    float callNormRef =::sqrtf((float) callRef);
    float putNormRef  =::sqrtf((float) putRef);
    if ((::fabs((float) callRef) < 1e-7f) || (::fabs((float) putRef) < 1e-7f))
    {
        return false;
    }

    float callNormError = ::sqrtf((float) callError);
    float putNormError  = ::sqrtf((float) putError);
    callError = callNormError / callNormRef;
    putError  = putNormError / putNormRef;

    return ((callError < epsilon) && (putError < epsilon));
}


/******************************************************************************
* Execution of program begins from here                                       *
******************************************************************************/
int main(int argc, char * argv[])
{

    std::cout << "**********************************************" << std::endl;
    std::cout << "BlackScholes using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    /**************************************************************************
    * Create an object of BlackScholes class                                  *
    **************************************************************************/
    BlackScholes objBlackScholes;

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objBlackScholes.sampleArgs->parseCommandLine(argc, argv))
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
        if(objBlackScholes.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute BlackScholes algorithm on the input samples                     *
        **************************************************************************/
        if(objBlackScholes.run() != SDK_SUCCESS)
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
    if(objBlackScholes.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objBlackScholes.printStats();

    return SDK_SUCCESS;
}
