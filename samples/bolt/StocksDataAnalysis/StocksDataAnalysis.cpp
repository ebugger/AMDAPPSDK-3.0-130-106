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
#include "bolt/cl/transform.h"
#include "bolt/cl/scan.h"
#include "bolt/cl/sort.h"
#include "bolt/cl/sort_by_key.h"
#include "bolt/cl/reduce_by_key.h"
#include "StocksDataAnalysis.hpp"


/******************************************************************************
* Implementation of init()                                                   *
******************************************************************************/
int Stocks::init()
{
    // Delete -x, --samples option. sample size is specified in first line of input file
    Option numSamples;
    numSamples._sVersion = "x";
    numSamples._lVersion = "samples";
    numSamples._description = "Number of sample input values.";

    (this->sampleArgs)->DeleteOption(&numSamples);

    Option fileOption;
    fileOption._sVersion = "f";
    fileOption._lVersion = "file";
    fileOption._description = "File contaninig stock data";
    fileOption._type = CA_ARG_STRING;
    fileOption._value = &inputFileName;

    (this->sampleArgs)->AddOption(&fileOption);

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of init()                                                   *
******************************************************************************/
int Stocks::parseInputFile()
{
    std::ifstream inputFile(inputFileName);

    if(!inputFile.is_open())
    {
        std::cout << "Can't open " << inputFileName << "!" << std::endl;
        return SDK_FAILURE;
    }
    if(!sampleArgs->quiet)
    {
        std::cout << "Reading input file..." << std::endl;
    }

    std::string line;

    // Number of entries in the file
    getline(inputFile, line);
    sampleArgs->samples = atoi(line.c_str());
    if(sampleArgs->samples <= 0)
    {
        std::cout << "File has " << sampleArgs->samples << " entries " << std::endl;
        return SDK_FAILURE;
    }
    quotes.reserve(sampleArgs->samples);

    // Skip the 2nd line, which is the description of what each field is
    getline(inputFile,line);

    // A macro to get next token in the line
#define GET_NEXT_TOKEN(_token, _delim, _str, _currPos, _nextPos)    \
    _currPos = _nextPos + 1;                                    \
    _nextPos = _str.find_first_of(_delim, _currPos);            \
    _token = _str.substr(_currPos, (_nextPos - _currPos));      \
     

    for (int i=0; ((i < sampleArgs->samples) && inputFile.good()); i++)
    {
        Quote q;
        std::string token;
        size_t currentPos, nextPos = -1;        // Used for string tokenization

        getline(inputFile, line);

        // Get ticker symbol
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        if(token.empty())
        {
            break;
        }

        /*
         * Ticker symbol will be strictly of length 4
         * 4 chars will be stored in an unsigned int as
         * (char[0]<<24) + (char[1]<<16) + (char[2]<<8) + char[3]
         */
        q.tickerSymbol = 0;
        q.tickerSymbol += (unsigned int)token[0] << 24;
        q.tickerSymbol += (token.size() > 1)? ((unsigned int)token[1] << 16): 0;
        q.tickerSymbol += (token.size() > 2)? ((unsigned int)token[2] << 8): 0;
        q.tickerSymbol += (token.size() > 3)? ((unsigned int)token[1]): 0;

        /* Date - Assuming dd-mm-yy or dd/mm/yy format
         * use a single integer to represent the date:
         * (year*10000) + (month*100) + day
         */
        GET_NEXT_TOKEN(token, "/-", line, currentPos, nextPos);
        q.date = atoi(token.c_str());

        GET_NEXT_TOKEN(token, "/-", line, currentPos, nextPos);
        q.date += atoi(token.c_str())*100;

        GET_NEXT_TOKEN(token, "/-", line, currentPos, nextPos);
        q.date += atoi(token.c_str())*10000;


        // Open
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        q.open = (float)atof(token.c_str());

        // High
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        q.high = (float)atof(token.c_str());

        // Low
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        q.low = (float)atof(token.c_str());

        // Close
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        q.close = (float)atof(token.c_str());

        // Volume
        GET_NEXT_TOKEN(token, ',', line, currentPos, nextPos);
        q.volume = atoi(token.c_str());

        quotes.push_back(q);
    }

    inputFile.close();
    std::cout << std::endl;
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of setup()                                                   *
******************************************************************************/
int Stocks::setup()
{
    if(parseInputFile() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

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



    boltQuotes.resize(sampleArgs->samples);

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed setup() of Stocks sample" << std::endl;
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of run()                                                     *
******************************************************************************/
int Stocks::run()
{
    int timer = sampleTimer->createTimer();
    // Copy data from host to device
    {
        bolt::cl::device_vector<Quote>::pointer boltQuotesPtr = boltQuotes.data();
        memcpy(boltQuotesPtr.get(), quotes.data(),
               (sizeof(Quote) * sampleArgs->samples));
    }

    if(sampleArgs->iterations > 1)
    {
        if(boltAvgScanImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(boltAvgTransformImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(boltDailyVolumeTransformImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(boltDailyVolumeReduceImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed Warm up run of Bolt code" << std::endl;
        }
    }

    std::cout << "Executing Stocks sample over " << sampleArgs->iterations
              << " iteration(s)." << std::endl;

    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltAvgScanImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    avgScanTime = (double)(sampleTimer->readTimer(timer));
    avgScanTime /= sampleArgs->iterations;


    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltAvgTransformImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    avgTransformTime = (double)(sampleTimer->readTimer(timer));
    avgTransformTime /= sampleArgs->iterations;


    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltDailyVolumeTransformImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    dailyVolumeTransformTime = (double)(sampleTimer->readTimer(timer));
    dailyVolumeTransformTime /= sampleArgs->iterations;


    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(boltDailyVolumeReduceImpl() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);
    dailyVolumeReduceTime = (double)(sampleTimer->readTimer(timer));
    dailyVolumeReduceTime /= sampleArgs->iterations;


    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of Stocks sample" << std::endl;
    }
    return SDK_SUCCESS;
}

bool Stocks::compare(const float *refData, const float *data,
                     size_t length, const float epsilon)
{
    float error = 0.0f;
    float ref = 0.0f;

    for(int i = 1; i < (int)length; ++i)
    {
        float diff = refData[i] - data[i];
        error += diff * diff;
        ref += refData[i] * refData[i];
    }

    float normRef =::sqrtf((float) ref);
    if (::fabs((float) ref) < 1e-7f)
    {
        return false;
    }
    float normError = ::sqrtf((float) error);
    error = normError / normRef;

    return error < epsilon;
}

/******************************************************************************
* Implementation of verifyResults()                                           *
******************************************************************************/
int Stocks::verifyResults()
{
    if(sampleArgs->verify)
    {
        bool avgTransformResult, avgScanResult, dailyVolTransformResult ,
             dailyVolReduceResult;

        // Calling reference implementation to store results in verificationAvg
        referenceAvgImpl(5);

        // Map transform avg results from device to host and compare
        {
            /***********************************************************************************
             * Mapping device_vector boltTransformAvgResults to CPU for reading
             * .data() maps device_vector to CPU and returns a
             *   device_vector<T>::pointer (boost smart-pointer/shared-array)
             * .get() of device_vector<T>::pointer returns a T* pointer to actual memory in host
             *************************************************************************************/
            bolt::cl::device_vector<float>::pointer boltTransformAvgResultsPtr =
                boltTransformAvgResults.data();
            avgTransformResult = compare(boltTransformAvgResultsPtr.get(),
                                         verificationAvg.data(), verificationAvg.size());
        }

        // Calling reference implementation to store results in verificationAvg
        referenceAvgImpl(30);

        // Map scan avg results from device to host and compare
        {
            bolt::cl::device_vector<float>::pointer boltScanAvgResultsPtr =
                boltScanAvgResults.data();

            // Scan involves progressive addition. More the samples/quotes, more the epsilon
            float scanEpsilon = sampleArgs->samples * 1e-7f;
            avgScanResult = compare(boltScanAvgResultsPtr.get(), verificationAvg.data(),
                                    verificationAvg.size(), scanEpsilon);
        }

        // Calling reference implementation to store results in verificationDailyVolume
        referenceDailyVolImpl();

        // Map daily volume transform results from device to host and compare
        {
            bolt::cl::device_vector<unsigned int>::pointer boltDailyVolResultsPtr =
                boltDailyVolumeTransformResults.data();
            dailyVolTransformResult = !(memcmp(boltDailyVolResultsPtr.get(),
                                               verificationDailyVolume.data(),
                                               (verificationDailyVolume.size() * sizeof(unsigned int))));
        }

        // Map daily volume reduce results from device to host and compare
        {
            bolt::cl::device_vector<unsigned int>::pointer boltDailyVolResultsPtr =
                boltDailyVolumeReduceResults.data();
            dailyVolReduceResult = !(memcmp(boltDailyVolResultsPtr.get(),
                                            verificationDailyVolume.data(),
                                            (verificationDailyVolume.size() * sizeof(unsigned int))));
        }

        if(!avgTransformResult || !avgScanResult || !dailyVolTransformResult ||
                !dailyVolReduceResult)
        {
            std::cout << "Failed!" << std::endl;
            return SDK_FAILURE;
        }
        std::cout << "Passed!" << std::endl;

        if(!sampleArgs->quiet)
        {
            std::cout << "Completed verifyResults() of Stocks sample"
                      << std::endl;
        }
    }
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of printStats()                                              *
******************************************************************************/
void Stocks::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[3] = {"Sample Size", "Avg. Execution Time(sec)", "Samples/sec"};
        std::string stats[3];

        stats[0] = toString<int>(sampleArgs->samples);

        std::cout <<
                  "\n1. Bolt implementation of avg closing price in 30 days using scan()" <<
                  std::endl;
        stats[1] = toString(avgScanTime);
        stats[2] = toString((sampleArgs->samples / avgScanTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout <<
                  "\n2. Bolt implementation of avg closing price in 5 days using transform()" <<
                  std::endl;
        stats[1] = toString(avgTransformTime);
        stats[2] = toString((sampleArgs->samples / avgTransformTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout <<
                  "\n3. Bolt implementation of daily trade volume calculation using transform()"
                  << std::endl;
        stats[1] = toString(dailyVolumeTransformTime);
        stats[2] = toString((sampleArgs->samples / dailyVolumeTransformTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;

        std::cout <<
                  "\n4. Bolt implementation of daily trade volume calculation using reduce_by_key()"
                  << std::endl;
        stats[1] = toString(dailyVolumeReduceTime);
        stats[2] = toString((sampleArgs->samples / dailyVolumeReduceTime));

        printStatistics(strArray, stats, 3);
        std::cout << std::endl;
    }
}


/******************************************************************************
* Implementation of referenceAvgImpl()                                        *
* Calculate avg closing price in N days using std functions                   *
*******************************************************************************/
int Stocks::referenceAvgImpl(int avgForDays)
{
    int elemsToWork = sampleArgs->samples - (avgForDays-1);
    verificationAvg.resize(elemsToWork);

    for(int i=0; i < elemsToWork; i++)
    {
        float sum = 0.0f;
        for(int days=0; days < avgForDays; days++)
        {
            sum += quotes[i + days].close;
        }
        verificationAvg[i] = sum / avgForDays;
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of referenceDailyVolImpl()                                   *
* Calculate daily trade volume of all companies using std functions           *
*******************************************************************************/
int Stocks::referenceDailyVolImpl()
{
    // Take a copy of quotes vector
    std::vector<Quote> quotesCopy(quotes);
    std::sort(quotesCopy.begin(), quotesCopy.end(), sortByDateDescending());

    int numDates = sampleArgs->samples / NUM_COMPANIES;
    verificationDailyVolume.resize(numDates);

    std::vector<QuoteNTuple> *nTuplePtr =
        reinterpret_cast<std::vector<QuoteNTuple>*>(&quotesCopy);
    std::vector<QuoteNTuple>::iterator qBegin = nTuplePtr->begin();

    std::transform(qBegin, (qBegin + numDates), verificationDailyVolume.begin(),
                   DailyVolumeFunctor());

    quotesCopy.clear();
    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of boltTransformImpl()                                       *
* Calculating avg closing price in N days. In this case 5 consecutive days    *
******************************************************************************/
int Stocks::boltAvgTransformImpl()
{
    int avgForDays = 5;

    boltTransformAvgResults.resize(sampleArgs->samples - (avgForDays-1));

    // Copy the close data
    bolt::cl::device_vector<float> dvClosingPrices(sampleArgs->samples);
    bolt::cl::transform(boltQuotes.begin(), boltQuotes.end(),
                        dvClosingPrices.begin(), getClose());

    // Create a sum of pairs (x, x+1)
    bolt::cl::device_vector<float> dvSumPairs(sampleArgs->samples-1);
    bolt::cl::transform(dvClosingPrices.begin(), dvClosingPrices.end()-1,
                        dvClosingPrices.begin()+1, dvSumPairs.begin(), bolt::cl::plus<float>());

    // Create a sum of pair of pairs (x = x, x+1, x+2, x+3)
    bolt::cl::device_vector<float> dvSumQuads(sampleArgs->samples-3);
    bolt::cl::transform(dvSumPairs.begin(), dvSumPairs.end()-2,
                        dvSumPairs.begin()+2, dvSumQuads.begin(), bolt::cl::plus<float>());

    // Add 5th day closing-price and get average using functor addLastAvgPrice()
    bolt::cl::transform((dvClosingPrices.begin()+(avgForDays-1)),
                        dvClosingPrices.end(), dvSumQuads.begin(),
                        boltTransformAvgResults.begin(), addLastAvgPrice(avgForDays));

    // Clear buffers
    dvClosingPrices.clear();
    dvSumPairs.clear();
    dvSumQuads.clear();

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of boltScanImpl()                                            *
* Calculating avg closing price in N days. In this case 30 consecutive days   *
******************************************************************************/
int Stocks::boltAvgScanImpl()
{
    int avgForDays = 30;

    boltScanAvgResults.resize(sampleArgs->samples - (avgForDays-1));

    bolt::cl::device_vector<float> dvClosingPrices(sampleArgs->samples);
    bolt::cl::transform (boltQuotes.begin(), boltQuotes.end(),
                         dvClosingPrices.begin(), getClose());

    // Perform inclusive scan on closing-price
    bolt::cl::device_vector<float> dvSumsClosingPrice(sampleArgs->samples+1);
    dvSumsClosingPrice[0] = 0.0f;
    bolt::cl::inclusive_scan(dvClosingPrices.begin(), dvClosingPrices.end(),
                             (dvSumsClosingPrice.begin()+1));

    // avgPrice() functor calculates 5 days' average closing price
    bolt::cl::transform((dvSumsClosingPrice.begin()+avgForDays),
                        dvSumsClosingPrice.end(), dvSumsClosingPrice.begin(),
                        boltScanAvgResults.begin(), avgPrice(avgForDays));

    // Clear buffers
    dvClosingPrices.clear();
    dvSumsClosingPrice.clear();

    return SDK_SUCCESS;
}

/*********************************************************************************
* Implementation of boltDailyVolumeImpl()                                        *
* Calculating daily volume trade of all companies combined using bolt::transform *
**********************************************************************************/
int Stocks::boltDailyVolumeTransformImpl()
{
    // Take a copy of boltQuotes vector
    bolt::cl::device_vector<Quote> boltQuotesCopy(boltQuotes);
    bolt::cl::sort(boltQuotesCopy.begin(), boltQuotesCopy.end(),
                   sortByDateDescending());

    int numDates = sampleArgs->samples / NUM_COMPANIES;
    boltDailyVolumeTransformResults.resize(numDates);

    /*
     * Casting a vector from type 'Quote' to 'QuoteNTuple'
     * 'QuoteNTuple' is a structure that holds NUM_COMPANIES quotes, 1 tuple from each company
     * since boltQuotesCopy is sorted by date, every item in 'QuoteNTuple'
     * will contain tuple of all companies for a particular date
     */
    bolt::cl::device_vector<QuoteNTuple> *nTuplePtr =
        reinterpret_cast<bolt::cl::device_vector<QuoteNTuple>*>(&boltQuotesCopy);
    bolt::cl::device_vector<QuoteNTuple>::iterator qBegin = nTuplePtr->begin();

    // Bolt currently requires dependent structures/defines to be explicitly specified
    BOLT_ADD_DEPENDENCY(QuoteNTuple, NUM_COMPANIES_DEFINE);
    BOLT_ADD_DEPENDENCY(QuoteNTuple, Quote);

    // 'QuoteNTuple' contains NUM_COMPANIES quotes for a particular day
    // DailyVolumeFunctor() will calculate sum of all volumes in 'QuoteNTuple'
    bolt::cl::transform(qBegin, (qBegin + numDates),
                        boltDailyVolumeTransformResults.begin(), DailyVolumeFunctor());

    boltQuotesCopy.clear();

    return SDK_SUCCESS;
}

/**************************************************************************************
* Implementation of boltDailyVolumeReduceImpl()                                       *
* Calculating daily volume trade of all companies combined using bolt::reduce_by_key  *
***************************************************************************************/
int Stocks::boltDailyVolumeReduceImpl()
{
    int numDates = sampleArgs->samples / NUM_COMPANIES;

    boltDailyVolumeReduceResults.resize(numDates);

    bolt::cl::device_vector<unsigned int> dvQuoteDates(sampleArgs->samples);
    bolt::cl::device_vector<unsigned int> dvQuoteVolumes(sampleArgs->samples);
    // Allocate a temporary vector to store unique keys returned by reduce_by_key()
    bolt::cl::device_vector<unsigned int> boltQuoteUniqueDates(numDates);

    // Extract dates & volumes from Quotes and store it in a vector
    bolt::cl::transform(boltQuotes.begin(), boltQuotes.end(), dvQuoteDates.begin(),
                        getDate());
    bolt::cl::transform(boltQuotes.begin(), boltQuotes.end(),
                        dvQuoteVolumes.begin(), getVolume());

    /*
     * reduce_by_key() reduces common keys which are consecutive
     * If dates are sorted, then reduce_by_key() can reduce all volumes for a particular date
     * so, using sort_by_key(), date being key
     */
    bolt::cl::sort_by_key(dvQuoteDates.begin(), dvQuoteDates.end(),
                          dvQuoteVolumes.begin(),
                          bolt::cl::greater<unsigned int>());

    // Reducing volume, date-wise
    bolt::cl::reduce_by_key(dvQuoteDates.begin(), dvQuoteDates.end(),
                            dvQuoteVolumes.begin(),
                            boltQuoteUniqueDates.begin(), boltDailyVolumeReduceResults.begin());

    dvQuoteDates.clear();
    dvQuoteVolumes.clear();
    boltQuoteUniqueDates.clear();

    return SDK_SUCCESS;
}


int main(int argc, char * argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "Stock Data Analysis using BOLT" << std::endl;
    std::cout << "**********************************************" << std::endl;

    /**************************************************************************
    * Create an object of Stocks class                                        *
    **************************************************************************/
    Stocks objStocks;

    /**************************************************************************
    * Add sample specific command line options                                *
    **************************************************************************/
    if(objStocks.init() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objStocks.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
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
        if(objStocks.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute BlackScholes algorithm on the input samples                     *
        **************************************************************************/
        if(objStocks.run() != SDK_SUCCESS)
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
    if(objStocks.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objStocks.printStats();

    return SDK_SUCCESS;
}
