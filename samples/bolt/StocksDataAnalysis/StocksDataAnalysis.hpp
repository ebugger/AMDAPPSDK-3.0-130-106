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

#ifndef _BOLTSTOCKS_HPP_
#define _BOLTSTOCKS_HPP_

/******************************************************************************
* Included header files                                                       *
*******************************************************************************/
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "BoltCLUtil.hpp"
#include "bolt/cl/bolt.h"
#include "bolt/cl/device_vector.h"
#include "bolt/unicode.h"
#include "bolt/statisticalTimer.h"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

// Using safe versions of str apis on windows
#ifdef _MSC_VER
#define strcpy strcpy_s
#endif


/*
 * NUM_COMPANIES defines total number of companies listed in csv file
 * Please update this define when csv file contains more/less number of companies
 */
#define NUM_COMPANIES 4

// Bolt currently requires this for bolt-functors to use host-side defines
BOLT_CREATE_DEFINE(NUM_COMPANIES_DEFINE, NUM_COMPANIES, 4);

// A structure to store a tuple from stock data
BOLT_FUNCTOR(Quote,
             struct Quote
{
    /*
     * Ticker symbol can have maximum of 4 chars
     * 4 chars will be stored in an unsigned int as
     * (char[0]<<24) + (char[1]<<16) + (char[2]<<8) + char[3]
     */
    unsigned int tickerSymbol;

    /*
     * Use a single integer to represent the date
     * Date : (year*10000) + (month*100) + day
     */
    unsigned int date;

    float open;
    float high;
    float low;
    float close;
    unsigned int volume;

    unsigned int dummy;     // For future use
};
            );
// An iterator needs to be declared for every user-defined data-type
BOLT_TEMPLATE_REGISTER_NEW_ITERATOR(bolt::cl::device_vector, int, Quote);


// A structure to hold NUM_COMPANIES quotes, 1 tuple from each company
BOLT_FUNCTOR(QuoteNTuple,
             struct QuoteNTuple
{
    Quote q[NUM_COMPANIES];
};
            );
// An iterator needs to be declared for every user-defined data-type
BOLT_TEMPLATE_REGISTER_NEW_ITERATOR(bolt::cl::device_vector, int, QuoteNTuple);


// Binary comparison function used to compare 2 elements while sorting
BOLT_FUNCTOR(sortByDateDescending,
             struct sortByDateDescending
{
    bool operator() (const Quote& a, const Quote& b) const
    {
        // If 2 quotes have same date, compare based on their ticker names
        bool isBigger = (a.date > b.date) || ( (a.date == b.date) &&
        (a.tickerSymbol < b.tickerSymbol) );
        return isBigger;
    };
};
            );


// Functor to return closing price from Quote structure
BOLT_FUNCTOR(getClose,
             struct getClose
{
    float operator() (Quote& q)
    {
        return q.close;
    };
};
            );

// Functor to return date from Quote structure
BOLT_FUNCTOR(getDate,
             struct getDate
{
    unsigned int operator() (Quote& q)
    {
        return q.date;
    };
};
            );


// Functor to return volume from Quote structure
BOLT_FUNCTOR(getVolume,
             struct getVolume
{
    unsigned int operator() (Quote& q)
    {
        return q.volume;
    };
};
            );

/*
 * Functor to calculate 5 day avg closing price
 * After inclusive_scan() on closing-prices(cp), cp[current] - cp[curent-5] gives sum of closing-prices in 5 days
 * An average of 5 days' closing-price is then calculated
 */
BOLT_FUNCTOR(avgPrice,
             struct avgPrice
{
    avgPrice(int _days):days(_days) {};

    float operator()(float& current, float& old) const
    {
        return (current - old)/(float)days;
    };

private:
    int days;
};
            );


/*
 * Refer boltAvgTransformImpl() for how this functor is used
 * closing-price sum of 4 days is calculated using bolt::transform() with bolt::plus()
 * This functor adds the 5th day closing-price and returns the average
 */
BOLT_FUNCTOR(addLastAvgPrice,
             struct addLastAvgPrice
{
    addLastAvgPrice(int _days):days(_days) {};

    float operator()(float& last, float& accumulated) const
    {
        return (last + accumulated) / (float)days;
    };

private:
    int days;
};
            );


/*
 * 'QuoteNTuple' contains NUM_COMPANIES quotes for a particular day
 * Functor to calculate sum of all volumes in 'QuoteNTuple'
 */
BOLT_FUNCTOR(DailyVolumeFunctor,
             struct DailyVolumeFunctor
{
    DailyVolumeFunctor() {};

    unsigned int operator() (const QuoteNTuple& nTuple)
    {
        unsigned int total = 0;
        for (int i = 0; i < NUM_COMPANIES; i++)
        {
            total += nTuple.q[i].volume;
        }
        return total;
    };
};
            );


/********************************************************************************
* Stocks                                                                        *
* Class implements various operations on stock data to generate a report        *
*********************************************************************************/
class Stocks
{
        std::vector<Quote> quotes;             /**< Container for stock data */
        std::vector<float> verificationAvg;
        std::vector<unsigned int> verificationDailyVolume;

        /****************************************************************************
        * Create container to hold stock data on the device using device_vector     *
        * to gain a huge performance boost. In this sample the stock data used      *
        * for cpu calculations are replicated to exist on the device                *
        *****************************************************************************/
        bolt::cl::device_vector<Quote> boltQuotes;
        bolt::cl::device_vector<float> boltTransformAvgResults;
        bolt::cl::device_vector<float> boltScanAvgResults;
        bolt::cl::device_vector<unsigned int> boltDailyVolumeTransformResults;
        bolt::cl::device_vector<unsigned int> boltDailyVolumeReduceResults;

        double  avgScanTime,                     /**< Measure time for different implementations */
                avgTransformTime,
                dailyVolumeTransformTime,
                dailyVolumeReduceTime;

        std::string inputFileName;

        SDKTimer *sampleTimer;                      /** Sample Timer object */

    public:
        BoltCommandArgs *sampleArgs;                 /**< Command helper object *
    /**
    ***************************************************************************
    * @brief Constructor of Stocks to initialize member variables
    * Allocating the input device_vector in host visible device device memory
    **************************************************************************/
        Stocks() :
            avgScanTime(0.0), avgTransformTime(0.0),
            dailyVolumeTransformTime(0.0),
            dailyVolumeReduceTime(0.0),
            inputFileName("stocks.csv")
        {

            sampleArgs =  new BoltCommandArgs( 1,
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
        * @brief Destructor of Stocks
        **************************************************************************/
        ~Stocks()
        {
            quotes.clear();
            verificationAvg.clear();
            verificationDailyVolume.clear();

            boltQuotes.clear();
            boltTransformAvgResults.clear();
            boltScanAvgResults.clear();
            boltDailyVolumeTransformResults.clear();
            boltDailyVolumeReduceResults.clear();
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
        * @fn parseInputFile
        * @brief read a csv file and parse it for stock data
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int parseInputFile();

        /**
        ***************************************************************************
        * @fn run
        * @brief Run stocks implementation in Bolt
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
        ******************************************************************************
        * @fn boltAvgScanImpl
        * @brief Calculating avg closing price in N days using bolt::inclusive_scan()
        * @return SDK_SUCCESS on success and nonzero on failure
        ******************************************************************************/
        int boltAvgScanImpl();

        /**
        ***************************************************************************
        * @fn boltAvgTransformImpl
        * @brief Calculating avg closing price in N days using bolt::transform()
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int boltAvgTransformImpl();

        /**
        ******************************************************************************
        * @fn referenceAvgImpl
        * @brief Calculating avg closing price in N days using std functions
        * @param avgForDays indicates number of days for which average is taken
        * @return SDK_SUCCESS on success and nonzero on failure
        ******************************************************************************/
        int referenceAvgImpl(int avgForDays);

        /**
        *****************************************************************************
        * @fn boltDailyVolumeTransformImpl
        * @brief Calculate daily trade volume of all companies using bolt::transform
        * @return SDK_SUCCESS on success and nonzero on failure
        *****************************************************************************/
        int boltDailyVolumeTransformImpl();

        /**
        ********************************************************************************
        * @fn boltDailyVolumeReduceImpl
        * @brief Calculate daily trade volume of all companies using bolt::reduce_by_key
        * @return SDK_SUCCESS on success and nonzero on failure
        ********************************************************************************/
        int boltDailyVolumeReduceImpl();

        /**
        ******************************************************************************
        * @fn referenceDailyVolImpl
        * @brief Calculate daily trade volume of all companies using std functions
        * @return SDK_SUCCESS on success and nonzero on failure
        ******************************************************************************/
        int referenceDailyVolImpl();

        /**
        ***************************************************************************
        * @fn compare
        * @brief used to compare results with reference implementation
        * @param refData Pointer to reference data
        * @param data Pointer to data to be compared
        * @param length number of elements to be compared
        * @param epsilon maximum delta/epsilon that can be tolerated
        * @return true when all values match, false otherwise
        **************************************************************************/
        bool compare(const float *refData, const float *data,
                     size_t length, const float epsilon=1e-6f);
};

#endif
