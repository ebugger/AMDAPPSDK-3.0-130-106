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
#include <algorithm>
#include <functional>
#include "bolt/unicode.h"
#include "bolt/cl/transform.h"
#include "bolt/statisticalTimer.h"
#include "BoltSort.hpp"


/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int sort::setup()
{
    arrayInput.resize(sampleArgs->samples);
    cpuArrayInputSort.resize(sampleArgs->samples);
    cpuArrayInputSortByKey.resize(sampleArgs->samples);
    cpuArrayInputStableSort.resize(sampleArgs->samples);
    cpuKeys.resize(sampleArgs->samples);
    arrayKeys.resize(sampleArgs->samples);
    boltArrayInputSort.resize(sampleArgs->samples);
    boltArrayInputStableSort.resize(sampleArgs->samples);
    boltArrayInputSortByKey.resize(sampleArgs->samples);
    boltArrayKeys.resize(sampleArgs->samples);
    keyValPair.resize(sampleArgs->samples);


    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    for(int i = 0; i < sampleArgs->samples; i++)
    {
        arrayInput[i] = (int)rand() / (int)RAND_MAX;
    }

    for(int i=0; i<3; i++)
    {
        if ((sampleArgs->samples/2 + i) < sampleArgs->samples)
        {
            arrayInput[(sampleArgs->samples/2) + i] = 100;
        }
    }
    for(int i = 0; i < sampleArgs->samples; i++)
    {
        arrayKeys[i] = (int)rand() / (int)RAND_MAX;
    }

    if( !sampleArgs->enable_tbb && (strComparei(sampleArgs->runMode, "multicorecpu")) )
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
        std::cout << "Completed setup() of sort sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int sort::run()
{
int timer = sampleTimer->createTimer();
    if(sampleArgs->iterations>1)
    {
        if(sortBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(stableSortBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(sortByKeyBOLT() != SDK_SUCCESS)
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
        std::cout << "Executing sort sample over " << sampleArgs->iterations
                  << " iteration(s)." << std::endl;
    }

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < sampleArgs->iterations; i++) //sort
    {

        if(sortBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    sortTime = (double)(sampleTimer->readTimer(timer));
    sortTime /= sampleArgs->iterations;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < sampleArgs->iterations; i++) //stable sort
    {

        if(stableSortBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }
    sampleTimer->stopTimer(timer);
    stableSortTime = (double)(sampleTimer->readTimer(timer));
    stableSortTime /= sampleArgs->iterations;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i = 0; i < sampleArgs->iterations; i++) //sort by key
    {

        if(sortByKeyBOLT() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    sortByKeyTime = (double)(sampleTimer->readTimer(timer));
    sortByKeyTime /= sampleArgs->iterations;

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of Sort sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int sort::verifyResults()
{
    if(sampleArgs->verify)
    {
        sortCPU();

        if(!sampleArgs->quiet)
        {
            std::cout << "\nComparing .." << std::endl;
        }

        bool result = compare();

        if(!result )
        {
            std::cout << "Failed!" << std::endl;
            return SDK_FAILURE;
        }

        std::cout << "Passed!" << std::endl;
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void sort::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[3] = {"Size of Array", "Avg Time (s)", "Elements/sec"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for sort()\n" << std::endl;
        stats[1] = toString(sortTime);
        stats[2] = toString(sampleArgs->samples / sortTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for sortByKey()\n" << std::endl;
        stats[1] = toString(sortByKeyTime);
        stats[2] = toString(sampleArgs->samples / sortByKeyTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\nPrinting stats for stableSort()\n" << std::endl;
        stats[1] = toString(stableSortTime);
        stats[2] = toString(sampleArgs->samples / stableSortTime);

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;
    }
}


/******************************************************************************
* Implementation of sortCPU()                                         *
******************************************************************************/
int sort::sortCPU()
{
    /**************************************************************************
    * For every loop iteration, we copy over fresh random data to the input   *
    * vector.  There is no need on the CPU device to make a copy of the       *
    * output data, as it already resides in host accessible memory.           *
    **************************************************************************/
    //CPU sort() implementation
    ::memcpy(&cpuArrayInputSort[0], &arrayInput[0],
             sizeof(int) * sampleArgs->samples);
    std::sort(cpuArrayInputSort.begin(), cpuArrayInputSort.end(), std::less<int>());

    //No equivalent sort by key in stl.
    //Hence using key value pair structure and running stl stable_sort
    for(int i = 0; i < sampleArgs->samples; i++)
    {
        keyValPair[i].key = arrayKeys[i];
        keyValPair[i].value = arrayInput[i];
    }

    struct sortComp
    {
        bool operator() (keyVal a, keyVal b)
        {
            return (a.key < b.key);
        };
    };

    std::stable_sort(keyValPair.begin(), keyValPair.end(), sortComp());

    //CPU stable sort implementation
    ::memcpy(&cpuArrayInputStableSort[0], &arrayInput[0],
             sizeof(int) * sampleArgs->samples);
    std::stable_sort(cpuArrayInputStableSort.begin(), cpuArrayInputStableSort.end(),
                     std::less<int>());

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of sortBOLT()                                        *
******************************************************************************/
int sort::sortBOLT()
{
    /**************************************************************************
    * The .data() method is called to map CPU memory to device memory         *
    * Retrieve a pointer to device memory and fill it with randomly           *
    * generated data.  Explicitly release the memory before calling into     *
    * bolt, which causes the updated data to transfer to the OpenCL device    *
    **************************************************************************/

    bolt::cl::device_vector<int>::pointer optionsPtr = boltArrayInputSort.data();
    ::memcpy(optionsPtr.get(), &arrayInput[0], sizeof(int) * sampleArgs->samples);
    optionsPtr.reset();

    bolt::cl::sort(boltArrayInputSort.begin(), boltArrayInputSort.end(),
                   bolt::cl::less<int>());

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of sortByKeyBOLT()                                        *
******************************************************************************/
int sort::sortByKeyBOLT()
{
    bolt::cl::device_vector<int>::pointer optionsPtr1 =
        boltArrayInputSortByKey.data();
    bolt::cl::device_vector<int>::pointer optionsPtr2 = boltArrayKeys.data();
    ::memcpy(optionsPtr1.get(), &arrayInput[0], sizeof(int) * sampleArgs->samples);
    ::memcpy(optionsPtr2.get(), &arrayKeys[0], sizeof(int) * sampleArgs->samples);
    optionsPtr1.reset();
    optionsPtr2.reset();

    bolt::cl::sort_by_key(boltArrayKeys.begin(), boltArrayKeys.end(),
                          boltArrayInputSortByKey.begin(), bolt::cl::less<int>());

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of stableSortBOLT()                                        *
******************************************************************************/
int sort::stableSortBOLT()
{
    bolt::cl::device_vector<int>::pointer pointer1 =
        boltArrayInputStableSort.data();
    ::memcpy(pointer1.get(), &arrayInput[0], sizeof(int) * sampleArgs->samples);
    pointer1.reset();

    bolt::cl::stable_sort(boltArrayInputStableSort.begin(),
                          boltArrayInputStableSort.end(), bolt::cl::less<int>());

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of compare()                                                 *
******************************************************************************/
bool sort::compare()
{
    /**************************************************************************
    * The .data() method of device_vector<> is called to map the pointer to   *
    * host memory                                                             *
    **************************************************************************/

    //CPU sort() comparison
    auto boltData = boltArrayInputSort.data();
    int ret = memcmp(boltData.get(), cpuArrayInputSort.data(),
                     sampleArgs->samples*sizeof(int));
    if(ret != 0)
    {
        return false;
    }

    //CPU sort by key comparison
    //Comparing keys only since bolt::cl::sort_by_key() is not a stable sort in that it does not preserve order of values
    auto boltData2 = boltArrayKeys.data();

    for(int i=0; i<sampleArgs->samples; i++)
    {
        if(boltData2[i] != keyValPair[i].key)
        {
            return false;
        }
    }

    //CPU stable sort comparison
    auto boltData3 = boltArrayInputStableSort.data();
    ret = memcmp(boltData3.get(), cpuArrayInputStableSort.data(),
                 sampleArgs->samples*sizeof(int));
    if(ret != 0)
    {
        return false;
    }

    return true;
}


/******************************************************************************
* Execution of program begins from here                                       *
******************************************************************************/
int main(int argc, char * argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "Sort using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    /**************************************************************************
    * Create an object of sort class                                  *
    **************************************************************************/
    sort objSort;

    /**************************************************************************
    * initialize command line args                                            *
    **************************************************************************/

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objSort.sampleArgs->parseCommandLine(argc, argv))
    {
        return SDK_FAILURE;
    }
    try
    {
        /**************************************************************************
        * Initialize the random array of input samples                            *
        **************************************************************************/
        if(objSort.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute sort algorithm on the input samples                             *
        **************************************************************************/
        if(objSort.run() != SDK_SUCCESS)
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
    if(objSort.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objSort.printStats();

    return 0;
}
