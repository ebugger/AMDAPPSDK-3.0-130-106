/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

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

#ifndef BOX_FILTER_SAT_H_
#define BOX_FILTER_SAT_H_

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include "BoltCLUtil.hpp"
#include "SDKBitMap.hpp"

#include "bolt/cl/bolt.h"
#include "bolt/cl/device_vector.h"
#include "bolt/cl/functional.h"
#include "bolt/cl/gather.h"

using namespace appsdk;

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

#define INPUT_IMAGE  "BoxFilter_Input.bmp"
#define OUTPUT_IMAGE "BoxFilter_Output.bmp"

#define GROUP_SIZE  256     //This should be a multiple of 64 for AMD GPUs.
#define FILTER      6

BOLT_FUNCTOR(UInt4,
             struct UInt4
{
    unsigned int ui[4];
};

UInt4 operator+ (const UInt4& left, const UInt4& right)
{
    UInt4 temp;
    temp.ui[0] = left.ui[0] + right.ui[0];
    temp.ui[1] = left.ui[1] + right.ui[1];
    temp.ui[2] = left.ui[2] + right.ui[2];
    temp.ui[3] = left.ui[3] + right.ui[3];
    return temp;
}
            );

BOLT_FUNCTOR(UChar4,
             struct UChar4
{
    unsigned char uc[4];
};
            );

BOLT_FUNCTOR(GetRow,
             struct GetRow
{
    GetRow(unsigned int& _width) : width(_width) { }

    unsigned int operator() (const unsigned int& f, const unsigned int& s) const
    {
        return (s / width);
    }

    unsigned int width;
};
            );

BOLT_FUNCTOR(GetTransposedIndex,
struct GetTransposedIndex 
{
  unsigned int  w,h;
GetTransposedIndex(unsigned int & _w,unsigned int & _h) : w(_w),h(_h) { }
unsigned  int operator() (const unsigned int &f,const unsigned int & val) const
{
   unsigned int i = val/h;
   unsigned int j = val%w;

   return j * h + i;

}

};
);
/********************************************************************************
* UChar4ToUInt4                                                                  *
* Convert UChar4 to UInt4                                                  *
*********************************************************************************/
BOLT_FUNCTOR(UChar4ToUInt4,
             struct UChar4ToUInt4
{
    UInt4 operator() (const UChar4& v)
    {
        UInt4 p = {v.uc[0], v.uc[1], v.uc[2], v.uc[3]};
        return p;
    }
};
            );

BOLT_TEMPLATE_REGISTER_NEW_ITERATOR(bolt::cl::device_vector, int, UInt4);
BOLT_TEMPLATE_REGISTER_NEW_TYPE(bolt::cl::plus, int, UInt4);
BOLT_TEMPLATE_REGISTER_NEW_ITERATOR(bolt::cl::device_vector, int, UChar4);

/******************************************************************************
* BoxFilterSAT                                                                *
* Class implements BoxFilterSAT implementation                                *
******************************************************************************/
class BoxFilterSAT
{
        uchar4* pixelData;         /**< Pointer to image data */
        cl_uchar4* outputImageData;         /**< Output from device */
        cl_uchar4*
        verificationOutput;      /**< Output from host reference implementation */
        bolt::cl::device_vector<UChar4>
        inputImgDeviceBuffer;  /**< Input device buffer */
        bolt::cl::device_vector<UChar4>
        outputImageBuffer;    /**< Output device buffer */
        bolt::cl::device_vector<UInt4>  inputDeviceBuffer;  /**< Input device buffer */
        bolt::cl::device_vector<UInt4>
        tempDeviceBuffer;      /**< Temp device buffer */
        bolt::cl::device_vector<cl_uint> pixelKeyBuffer;
        cl_uint width;                      /**< Width of image */
        cl_uint height;                     /**< Height of image */
        cl_uint filterWidth;                /**< Width of filter */
        SDKBitMap inputBitmap;    /**< Bitmap class object */

        // CL dependency
        cl_program program;
        cl_kernel boxFilter;
        cl_command_queue commandQueue;      /**< CL command queue */

        SDKTimer *sampleTimer;                      /** Sample Timer object */

    public:
        BoltCommandArgs *sampleArgs;                 /**< Command helper object *
    /**
    ***************************************************************************
    * @brief Constructor of BoxFilterSAT to initialize member variables
    * Allocating the input device_vector in host visible device device memory
    ***************************************************************************
    */
        BoxFilterSAT() :
            pixelData(NULL),
            outputImageData(NULL),
            verificationOutput(NULL),
            filterWidth(FILTER)
        {
            sampleArgs =  new BoltCommandArgs( 1,
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
        * @brief Destructor of BoxFilterSAT
        ***************************************************************************
        */
        ~BoxFilterSAT()
        {
            FREE(outputImageData);
            FREE(verificationOutput);

            inputImgDeviceBuffer.clear();
            outputImageBuffer.clear();
            inputDeviceBuffer.clear();
            tempDeviceBuffer.clear();
            pixelKeyBuffer.clear();
        }

        /**
        ***************************************************************************
        * @brief Initialize BoxFilterSAT
        ***************************************************************************
        */
        int init();

        /**
        ***************************************************************************
        * @fn setup
        * @brief Initialize the random array of input samples
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        int setup();

        /**
        ***************************************************************************
        * @fn run
        * @brief Run BoxFilterSAT implementation in Bolt
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        int run();

        /**
        ***************************************************************************
        * @fn verifyResults
        * @brief Verify against CPU reference implementation
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        int verifyResults();

        /**
        ***************************************************************************
        * @fn printStats
        * @brief Print timer statistics
        * @return void
        ***************************************************************************
        */
        void printStats();

        /**
        ***************************************************************************
        * @fn writeOutputImage
        * @brief Write the filtered image data to output file.
        * @param outputImageName Output image file name.
        * @return true on success and false on failure
        ***************************************************************************
        */
        int writeOutputImage(std::string outputImageName);

    private:
        /**
        ***************************************************************************
        * @fn setupCL
        * @brief Initialize the CL dependent items
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        int setupCL();

        /**
        ***************************************************************************
        * @fn runBoxFilterCLKernel
        * @brief Run the BoxFilter CL kernel on SAT image data.
        * @return true on success and false on failure
        ***************************************************************************
        */
        int runBoxFilterCLKernel();

        /**
        ***************************************************************************
        * @fn runTransposeCLKernel
        * @brief Run the transpose CL kernel.
        * @param input  Input buffer
        * @param output Output buffer
        * @return true on success and false on failure
        ***************************************************************************
        */
        int runTransposeCLKernel(cl_mem *input, cl_mem *output);

        /**
        ***************************************************************************
        * @fn blackScholesBOLT
        * @brief BOLT version of boxFilterSAT that calculates the call & put
        *        prices based on the input options
        * @return true on success and false on failure
        ***************************************************************************
        */
        int boxFilterSATBOLT();

        /**
        ***************************************************************************
        * @fn boxFilterSATCPUReference
        * @brief CPU version of box Filter SAT
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        void boxFilterSATCPUReference();

        /**
        ***************************************************************************
        * @fn readCLFile
        * @brief Read the CL file and return the string content
        * @param filename  CL file name
        * @param str       Content of the file
        * @return SDK_SUCCESS on success and nonzero on failure
        ***************************************************************************
        */
        int BoxFilterSAT::readCLFile(const std::string& filename, std::string &str);
		
		unsigned char clampToUchar(int n);
};

#endif // BOX_FILTER_SAT_H_