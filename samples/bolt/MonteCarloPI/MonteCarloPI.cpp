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
#include "bolt/cl/transform.h"
#include "bolt/cl/reduce.h"
#include "bolt/cl/transform_reduce.h"
#include "bolt/cl/count.h"
#include "bolt/statisticalTimer.h"
#include "MonteCarloPI.hpp"
#include "gui.hpp"


/******************************************************************************
* Implementation of init()                                                   *
******************************************************************************/
int MonteCarloPI::init()
{
    Option guiOption;
    guiOption._sVersion = "g";
    guiOption._lVersion = "gui";
    guiOption._description = "Show the GUI.";
    guiOption._type = CA_NO_ARGUMENT;
    guiOption._value = &showGUI;

    (this->sampleArgs)->AddOption(&guiOption);
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int MonteCarloPI::setup()
{
    inputPoints.resize(sampleArgs->samples);
    boltInputPoints.resize(sampleArgs->samples);

    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    for(int i=0; i<sampleArgs->samples; i++)
    {
        float x = (float)((double) std::rand() * (2.0 * RADIUS / RAND_MAX) - RADIUS);
        float y = (float)((double) std::rand() * (2.0 * RADIUS / RAND_MAX) - RADIUS);
        inputPoints[i] = Point(x, y);
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
        std::cout << "Completed setup() of MonteCarloPI sample" << std::endl;
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int MonteCarloPI::run()
{
    int timer = sampleTimer->createTimer();

    if(sampleArgs->iterations > 1)
    {
        if(boltTransformReduce() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(boltFusedTransformReduce() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(boltCountIf() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed Warm up run of Bolt code" << std::endl;
        }
    }

    std::cout << "Executing MonteCarloPI sample over " << sampleArgs->iterations
              << " iteration(s)." << std::endl;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltTransformReduce() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    boltTransformReduceTime = (double)(sampleTimer->readTimer(timer));
    boltTransformReduceTime /= sampleArgs->iterations;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltFusedTransformReduce() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    boltFusedTransformReduceTime = (double)(sampleTimer->readTimer(timer));
    boltFusedTransformReduceTime /= sampleArgs->iterations;


    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltCountIf() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    boltCountIfTime = (double)(sampleTimer->readTimer(timer));
    boltCountIfTime /= sampleArgs->iterations;


    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(stlTransformAccumulate() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    stlTransformAccumulateTime = (double)(sampleTimer->readTimer(timer));
    stlTransformAccumulateTime /= sampleArgs->iterations;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(stlCountIf() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    stlCountIfTime = (double)(sampleTimer->readTimer(timer));
    stlCountIfTime /= sampleArgs->iterations;

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of MonteCarloPI sample" << std::endl;
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int MonteCarloPI::verifyResults()
{
    if(sampleArgs->verify)
    {
        MonteCarloReferenceImpl();

        bool pass = (fabs(cpuPIValue - boltTransformReducePIValue) < 1e-6) &&
                    (fabs(cpuPIValue - boltFusedTransformReducePIValue)  < 1e-6) &&
                    (fabs(cpuPIValue - boltCountIfPIValue)  < 1e-6) &&
                    (fabs(cpuPIValue - stlTransformAccumulatePIValue)  < 1e-6) &&
                    (fabs(cpuPIValue - stlCountIfPIValue)  < 1e-6);
        if(!pass)
        {
            std::cout << "Failed!" << std::endl;
            return SDK_FAILURE;
        }
        std::cout << "Passed!" << std::endl;

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed verifyResults() of MonteCarloPI sample"
                      << std::endl;
        }
    }
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void MonteCarloPI::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[3] = {"Points", "Avg. Time(sec)", "Points/sec"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout << "\n1. Bolt implementation using transform() and reduce()" <<
                  std::endl;
        stats[1] = toString(boltTransformReduceTime);
        stats[2] = toString((sampleArgs->samples / boltTransformReduceTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout << "\n2. Bolt implementation using fused transform_reduce()" <<
                  std::endl;
        stats[1] = toString(boltFusedTransformReduceTime);
        stats[2] = toString((sampleArgs->samples / boltFusedTransformReduceTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout << "\n3. Bolt implementation using count_if()" << std::endl;
        stats[1] = toString(boltCountIfTime);
        stats[2] = toString((sampleArgs->samples / boltCountIfTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout << "\n4. STL implementation using transform() and reduce()" <<
                  std::endl;
        stats[1] = toString(stlTransformAccumulateTime);
        stats[2] = toString((sampleArgs->samples / stlTransformAccumulateTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout << "\n5. STL implementation using count_if()" << std::endl;
        stats[1] = toString(stlCountIfTime);
        stats[2] = toString((sampleArgs->samples / stlCountIfTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;
    }
}


/******************************************************************************
* Implementation of MonteCarloReferenceImpl()                                 *
******************************************************************************/
int MonteCarloPI::MonteCarloReferenceImpl()
{
    int numPointsInCircle = 0;

    for(int i=0; i<sampleArgs->samples; i++)
    {
        float xCoord = inputPoints[i].x;
        float yCoord = inputPoints[i].y;
        float distFromCenter = sqrtf( (xCoord * xCoord) + (yCoord * yCoord) );
        numPointsInCircle += (distFromCenter <= RADIUS)? 1: 0;
    }

    cpuPIValue = (4.0f * numPointsInCircle) / sampleArgs->samples;
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of boltTransformReduce()                                     *
******************************************************************************/
int MonteCarloPI::boltTransformReduce()
{
    /***********************************************************************************
    * Mapping device_vector boltInputPoints to CPU for writing
    * .data() maps device_vector to CPU and returns a
    *   device_vector<T>::pointer (boost smart-pointer/shared-array)
    * .get() of device_vector<T>::pointer returns a T* pointer to actual memory in host
    *************************************************************************************/
    bolt::cl::device_vector<Point>::pointer inputPtr = boltInputPoints.data();
    Point *mappedToCpu = inputPtr.get();

    ::memcpy(mappedToCpu, &inputPoints[0], sizeof(Point) * sampleArgs->samples);
    inputPtr.reset();       // .reset() unmaps device_vector from host

    bolt::cl::device_vector<int> insideCircle(sampleArgs->samples);
    bolt::cl::transform(boltInputPoints.begin(), boltInputPoints.end(),
                        insideCircle.begin(), isInsideCircleFunctor(RADIUS));

    int pointsInCircle = bolt::cl::reduce(insideCircle.begin(), insideCircle.end(),
                                          0);

    boltTransformReducePIValue = (4.0f * pointsInCircle) / sampleArgs->samples;
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of boltFusedTransformReduce()                                *
******************************************************************************/
int MonteCarloPI::boltFusedTransformReduce()
{
    bolt::cl::device_vector<Point>::pointer inputPtr = boltInputPoints.data();
    Point *mappedToCpu = inputPtr.get();

    ::memcpy(mappedToCpu, &inputPoints[0], sizeof(Point) * sampleArgs->samples);
    inputPtr.reset();

    int pointsInCircle = bolt::cl::transform_reduce(boltInputPoints.begin(),
                         boltInputPoints.end(), isInsideCircleFunctor(RADIUS),
                         0, bolt::cl::plus<int>());

    boltFusedTransformReducePIValue = (4.0f * pointsInCircle) / sampleArgs->samples;

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of boltCountIf()                                             *
******************************************************************************/
int MonteCarloPI::boltCountIf()
{
    bolt::cl::device_vector<Point>::pointer inputPtr = boltInputPoints.data();
    Point *mappedToCpu = inputPtr.get();

    ::memcpy(mappedToCpu, &inputPoints[0], sizeof(Point) * sampleArgs->samples);
    inputPtr.reset();

    int pointsInCircle = (int) bolt::cl::count_if(boltInputPoints.begin(),
                         boltInputPoints.end(),
                         isInsideCircleFunctor(RADIUS));

    boltCountIfPIValue = (4.0f * pointsInCircle) / sampleArgs->samples;

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of stlTransformAccumulate()                                  *
******************************************************************************/
int MonteCarloPI::stlTransformAccumulate()
{
    std::vector<int> insideCircle(sampleArgs->samples);
    std::transform(inputPoints.begin(), inputPoints.end(), insideCircle.begin(),
                   isInsideCircleFunctor(RADIUS));
    int pointsInCircle = std::accumulate(insideCircle.begin(), insideCircle.end(),
                                         0);

    stlTransformAccumulatePIValue = (4.0f * pointsInCircle) / sampleArgs->samples;
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of stlCountIf()                                              *
******************************************************************************/
int MonteCarloPI::stlCountIf()
{
    int pointsInCircle = (int) std::count_if(inputPoints.begin(), inputPoints.end(),
                         isInsideCircleFunctor(RADIUS));

    stlCountIfPIValue = (4.0f * pointsInCircle) / sampleArgs->samples;
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of displayGUI()                                              *
******************************************************************************/
int MonteCarloPI::displayGUI(int argc, char **argv)
{
    if(showGUI)
    {
        MonteCarloPIGUI GUIObj;
        GUIObj.showGUI(argc, argv);
    }
    return SDK_SUCCESS;
}


int main(int argc, char * argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "MonteCarloPI using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    /**************************************************************************
    * Create an object of MonteCarloPI class                                  *
    **************************************************************************/
    MonteCarloPI objMonteCarloPI;

    /**************************************************************************
    * Add sample specific command line options                                *
    **************************************************************************/
    if(objMonteCarloPI.init() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objMonteCarloPI.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Bolt APIs throw exception on failure                                    *
    **************************************************************************/
    try
    {
        /**************************************************************************
        * Initialize the random array of input ssamples                           *
        **************************************************************************/
        if(objMonteCarloPI.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute BlackScholes algorithm on the input samples                     *
        **************************************************************************/
        if(objMonteCarloPI.run() != SDK_SUCCESS)
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
    if(objMonteCarloPI.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objMonteCarloPI.printStats();

    /**************************************************************************
    * Display GUI if explicitly specified at command-line                     *
    **************************************************************************/
    objMonteCarloPI.displayGUI(argc, argv);

    return SDK_SUCCESS;
}
