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


#ifndef _BOLTMONTECARLOPI_HPP_
#define _BOLTMONTECARLOPI_HPP_

#define RADIUS 1.0f

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.2"

/******************************************************************************
* Included header files                                                       *
*******************************************************************************/
#include <iostream>
#include <vector>
#include <iostream>
#include <algorithm>
#include "BoltCLUtil.hpp"
#include "bolt/cl/bolt.h"
#include "bolt/cl/device_vector.h"
#include "bolt/unicode.h"

using namespace appsdk;

/********************************************************************************
* Point                                                                         *
* struct definition that contains 2D coordinates of a point to be used in the   *
* next functor                                                                  *
*********************************************************************************/
BOLT_FUNCTOR(Point,
             struct Point
{
    float x;
    float y;

    Point(float x, float y):x(x),y(y) {};
    Point() {};
};
            );

// An iterator needs to be declared for every user-defined data-type
BOLT_TEMPLATE_REGISTER_NEW_ITERATOR(bolt::cl::device_vector, int, Point);


/********************************************************************************
* isInsideCircleFunctor                                                         *
* Compare the distance of a point from the origin to the radius                 *
*********************************************************************************/
BOLT_FUNCTOR(isInsideCircleFunctor,
             struct isInsideCircleFunctor
{
    isInsideCircleFunctor(float _radius):radius(_radius) { };

    /************************************************************************************
    * @fn operator override used as functor to check if point falls inside the circle   *
    * @brief Calulates distance between the point and center of circle.                 *
    *        Center of circle is (0, 0) here. A point is inside the circle if distance  *
    *        between the point and center of cicrcle is less than the radius of circle  *
    * @return an int indicating whether the point falls inside the circle               *
    **********************************************************************              */
    int operator() (const Point& point)
    {
        float tx = point.x;
        float ty = point.y;
        float t = sqrt( (tx*tx) + (ty*ty) );
        return (t <= radius)? 1: 0;
    };

private:
    float radius;
};
            );

/********************************************************************************
* MonteCarloPI                                                                  *
* Class implements Monte-Carlo method to find value of PI                       *
*********************************************************************************/
class MonteCarloPI
{
        std::vector<Point>
        inputPoints;             /**< Container for the random points */

        /****************************************************************************
        * Create container to hold points on the device using device_vector         *
        * to gain a huge performance boost. In this sample the input points used    *
        * for cpu calculations are replicated to exist on the device                *
        *****************************************************************************/
        bolt::cl::device_vector<Point> boltInputPoints;

        bool showGUI;
        float cpuPIValue;
        float boltTransformReducePIValue,                       /**< Hold answers from different implementations */
              boltFusedTransformReducePIValue,
              boltCountIfPIValue,
              stlTransformAccumulatePIValue,
              stlCountIfPIValue;

        double  boltTransformReduceTime,                     /**< Measure time for different implementations */
                boltFusedTransformReduceTime,
                boltCountIfTime,
                stlTransformAccumulateTime,
                stlCountIfTime;

        SDKTimer *sampleTimer;                      /** Sample Timer object */

    public:
        BoltCommandArgs *sampleArgs;                 /**< Command helper object *
    /**
    ***************************************************************************
    * @brief Constructor of MonteCarloPI to initialize member variables
    * Allocating the input device_vector in host visible device device memory
    **************************************************************************/
        MonteCarloPI() :
            boltInputPoints(0, Point(), CL_MEM_READ_ONLY | CL_MEM_USE_PERSISTENT_MEM_AMD),
            showGUI(false),
            boltTransformReduceTime(0.0), boltFusedTransformReduceTime(0.0),
            boltCountIfTime(0.0), stlTransformAccumulateTime(0.0), stlCountIfTime(0.0)
        {

            sampleArgs =  new BoltCommandArgs( 1000000,
#ifdef ENABLE_TBB
                                               true
#else
                                               false
#endif
                                             );
            sampleArgs->iterations = 10;
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            sampleTimer = new SDKTimer();
        }

        /**
        ***************************************************************************
        * @brief Destructor of MonteCarloPI
        **************************************************************************/
        ~MonteCarloPI()
        {
            inputPoints.clear();
            boltInputPoints.clear();
        }

        /**
        ***************************************************************************
        * @fn init
        * @brief Perform sample specific initilizations.
        *        Add more command-line options
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int init();

        /**
        ***************************************************************************
        * @fn setup
        * @brief Initialize the random array of input points
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int setup();

        /**
        ***************************************************************************
        * @fn run
        * @brief Run Monte-Carlo PI implementation in Bolt
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int run();

        /**
        ***************************************************************************
        * @fn verifyResults
        * @brief Verify against CPU reference implementation
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int verifyResults();

        /**
        ***************************************************************************
        * @fn printStats
        * @brief Print timer statistics
        * @return void
        **************************************************************************/
        void printStats();

        /**
        ***************************************************************************
        * @fn MonteCarloReferenceImpl
        * @brief Reference implementation of Monte-Carlo that calculates the value of PI
        *        using input options. Used to verify results
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int MonteCarloReferenceImpl();

        /**
        ***************************************************************************
        * @fn boltTransformReduce
        * @brief BOLT version of Monte-Carlo that calculates the value of PI
        *        using bolt::cl::transform() & bolt::cl::reduce()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int boltTransformReduce();

        /**
        ***************************************************************************
        * @fn boltFusedTransformReduce
        * @brief BOLT version of Monte-Carlo that calculates the value of PI
        *        using bolt::cl::transform_reduce()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int boltFusedTransformReduce();

        /**
        ***************************************************************************
        * @fn boltCountIf
        * @brief BOLT version of Monte-Carlo that calculates the value of PI
        *        using bolt::cl::count_if()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int boltCountIf();

        /**
        ***************************************************************************
        * @fn stlTransformAccumulate
        * @brief STL version of Monte-Carlo that calculates the value of PI
        *        using std::transform() & std::accumulate()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int stlTransformAccumulate();

        /**
        ***************************************************************************
        * @fn stlCountIf
        * @brief STL version of Monte-Carlo that calculates the value of PI
        *        using std::count_if()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int stlCountIf();

        /**
        ***************************************************************************
        * @fn displayGUI
        * @brief Member function that handles GUI display
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int displayGUI(int argc, char **argv);

};

#endif
