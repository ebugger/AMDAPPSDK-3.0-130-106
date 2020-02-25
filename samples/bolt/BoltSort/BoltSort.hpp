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


#ifndef SORT_H_
#define SORT_H_

#define MAX_NUM 10

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include <iostream>
#include <vector>
#include <iostream>
#include <algorithm>
#include "BoltCLUtil.hpp"
#include "bolt/cl/bolt.h"
#include "bolt/cl/sort.h"
#include "bolt/cl/sort_by_key.h"
#include "bolt/cl/stablesort.h"

using namespace appsdk;

struct keyVal
{
    int key;
    int value;
};
/******************************************************************************
* sort                                                                   *
* Class implements sort(), sort_by_key() and stable_sort() APIs          *
******************************************************************************/
class sort
{
        double sortTime;
        double stableSortTime;
        double sortByKeyTime;
        std::vector<int>
        cpuArrayInputSort;             /**< array used for CPU implementation of sort() */
        std::vector<int>
        cpuArrayInputSortByKey;        /**< array used for CPU implementation of sort_by_key() */
        std::vector<int>
        cpuArrayInputStableSort;       /**< array used for CPU implementation of stable_sort() */
        std::vector<int>
        cpuKeys;                       /**< keys array used for CPU implementation */
        std::vector<int>
        arrayInput;                    /**< host side integer input array */
        std::vector<int>
        arrayKeys;                     /**< host side input keys array */
        std::vector<keyVal> keyValPair;                 /**< vector of struct keyVal */




        /**************************************************************************
        * Create unordered input array and keys array on the device using device_vector *
        * to gain a huge performance boost.               *
        **************************************************************************/
        bolt::cl::device_vector<int>
        boltArrayInputSort;            /**< device side input array for sort() */
        bolt::cl::device_vector<int>
        boltArrayInputSortByKey;       /**< device side input array for sort_by_key()*/
        bolt::cl::device_vector<int>
        boltArrayInputStableSort;  /**< device side input array for stable_sort()*/
        bolt::cl::device_vector<int>
        boltArrayKeys;                 /**< device side key array*/

        SDKTimer *sampleTimer;                      /** Sample Timer object */

    public:
        BoltCommandArgs *sampleArgs;                 /**< Command helper object *
    /**
    ***************************************************************************
    * @brief Constructor of sort to initialize member variables
    * Allocating the input device_vector in host visible device device memory
    **************************************************************************/
        sort() :
            boltArrayInputSort(0, 0, CL_MEM_READ_WRITE),
            boltArrayKeys(0, 0, CL_MEM_READ_WRITE),
            boltArrayInputSortByKey(0, 0, CL_MEM_READ_WRITE),
            boltArrayInputStableSort(0, 0, CL_MEM_READ_WRITE),
            sortTime(0.0), stableSortTime(0.0), sortByKeyTime(0.0)
        {

            sampleArgs =  new BoltCommandArgs( 1024,
#ifdef ENABLE_TBB
                                               true
#else
                                               false
#endif
                                             );
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            sampleTimer = new SDKTimer();

        }

        /**
        ***************************************************************************
        * @brief Destructor of sort
        **************************************************************************/
        ~sort()
        {
            cpuArrayInputSort.clear();
            cpuArrayInputSortByKey.clear();
            cpuArrayInputStableSort.clear();
            cpuKeys.clear();
            arrayInput.clear();
            arrayKeys.clear();
            keyValPair.clear();
            boltArrayInputSort.clear();
            boltArrayInputSortByKey.clear();
            boltArrayInputStableSort.clear();
            boltArrayKeys.clear();
        }

        /**
        ***************************************************************************
        * @fn setup
        * @brief Initialize the random array of input samples
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int setup();

        /**
        ***************************************************************************
        * @fn run
        * @brief Run sort implementation in Bolt
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
        * @fn sortCPU
        * @brief CPU version of the sort sample that implements sort(), sort_by_key()
        *        and stable_sort() on different input arrays.
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int sortCPU();

        /**
        ***************************************************************************
        * @fn sortBOLT
        * @brief BOLT version of the sort sample that uses the sort() API existing in Bolt.
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int sortBOLT();

        /**
        ***************************************************************************
        * @fn stableSortBOLT
        * @brief BOLT version of the stable_sort sample that uses the stable_sort() API existing in Bolt.
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int stableSortBOLT();

        /**
        ***************************************************************************
        * @fn sortByKeyBOLT
        * @brief BOLT version of the sort by key sample that uses the sort_by_key() API existing in Bolt.
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int sortByKeyBOLT();
        /**
        ***************************************************************************
        * @fn compare
        * @brief compares cpu sorted array with bolt sorted arry and checks if they match.
        *       This function is called by verifyResults()
        * @return true if all values match, false otherwise
        **************************************************************************/
        bool compare();
};

#endif
