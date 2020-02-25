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
#ifndef _BOLTNOISE_HPP_
#define _BOLTNOISE_HPP_


#include "BoltCLUtil.hpp"

#include "bolt/cl/transform.h"

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

using namespace appsdk;

BOLT_FUNCTOR(PerlinNoise,

             struct PerlinNoise
{

    int width,height,red,green,blue,octaves;
    float persistence,zoom;

    PerlinNoise(int _w,int _h,float _zoom,float _p,int _octaves, int _r, int _g, int _b)
        :width(_w),
        height(_h),
        persistence(_p),
        zoom(_zoom),
        red(_r),
        green(_g),
        blue(_b),
        octaves(_octaves)
    { }

    float interpolate(float a,float b,float x)
    {
        float ft=x * 3.1415;
        float f=(1.0-cos(ft))* 0.5;
        return a*(1.0-f)+b*f;
    }


    float findnoise(float x,float y)
    {
        int n=(int)x+(int)y*57;
        n=(n<<13)^n;
        int nn=(n*(n*n*60493+19990303)+1376312589)&0x7fffffff;
        return 1.0-((float)nn/1073741824.0);
    }

    float smoothNoise(float x, float y)
    {
        float corners = ( findnoise(x-1, y-1)+findnoise(x+1, y-1)+findnoise(x-1,
        y+1)+findnoise(x+1, y+1) ) / 16;
        float sides   = ( findnoise(x-1, y)  +findnoise(x+1, y)  +findnoise(x,
        y-1)  +findnoise(x, y+1) ) /  8;
        float center  =  findnoise(x, y) / 4;

        return corners + sides + center;
    }

    float interpolatedNoise(float x,float y)
    {
        float floorx=(float)((int)x);
        float floory=(float)((int)y);
        float s,t,u,v;
        s=smoothNoise(floorx,floory);
        t=smoothNoise(floorx+1,floory);
        u=smoothNoise(floorx,floory+1);
        v=smoothNoise(floorx+1,floory+1);
        float int1=interpolate(s,t,x-floorx);
        float int2=interpolate(u,v,x-floorx);
        return interpolate(int1,int2,y-floory);

    }


    int operator() (const unsigned int &pos)
    {
        float  getnoise = 0;
        int x= pos % width;
        int y =pos / height;
        float frequency,amplitude;
        for(int a= 0; a<octaves-1; a++)
        {

            frequency = pow((float)2,(float)a);
            amplitude = pow((float)persistence,(float)a);
            getnoise += interpolatedNoise(((float)x)*frequency/zoom,
            ((float)y)/zoom*frequency)*amplitude;
        }

        //Convert to 0-256 values.
        int color= (int)floor(((getnoise*128.0)+128.0) + 0.5);

        if(color>255)

        {
            color=255;
        }

        if(color<0)

        {
            color=0;
        }


        int r = ((int)((red/255.0)*(float)color)) << 16;
        int g = ((int)((green/255.0)*(float)color)) << 8;
        int b  = ((int)((blue/255.0)*(float)color));


        return (r + g + b);
    }

};
            );

class PerlinNoiseBolt
{

    private:

        std::vector<unsigned int > outputBufferCPU; /**< Output Image by CPU */
        std::vector<unsigned int> outputBufferBOLT; /**< Output Image by BOLT */
        std::vector<unsigned int> inputPixels;      /**< Pixel positions */
        unsigned int width,
                 height;                    /**< Width and height of the image */
        float zoom;                         /**< Zoom in and out */
        float persistence;                  /**< Used to roughness of the image */
        unsigned int
        octaves;                           /**< Number of coherent noise functions added */
        int red,blue,
            green;                         /**<  Intial values of RGB component */
        SDKTimer *sampleTimer;             /**< Timer object from SDKUtil */
        float avgExecTime;                 /**< Execution time /iterations */

    public:

        BoltCommandArgs *sampleArgs;     /**< Command-line helper class */


        /**
        ***************************************************************************
        * @fn SpMV
        * @brief Default constructor
        **************************************************************************/
        PerlinNoiseBolt()
        {
            //Default row/column size
            width = height = 1024;
            zoom =75;
            persistence = 0.5;
            octaves = 8;
            red = blue = green = 255;

            bool tbb ;
#ifdef ENABLE_TBB
            tbb=    true;
#else
            tbb=false;
#endif

            sampleArgs =  new BoltCommandArgs(1,tbb);
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
            sampleTimer = new SDKTimer();
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
        * @brief Generating the Sparse Matrix
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int setup();

        /**
        ***************************************************************************
        * @fn run
        * @brief Run implementation in Bolt
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int run();

        /**
        ***************************************************************************
        * @fn runPerlinNoise
        * @brief Run the  implementation in Bolt
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int runPerlinNoise();

        /**
        ***************************************************************************
        * @fn runPerlinNoiseCPu
        * @brief Run implementation in CPU
        * @return SDK_SUCCESS on success and nonzero on failure
        **************************************************************************/
        int runPerlinNoiseCPU();

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
        * @fn cleanup
        * @brief free all memory allocations
        * @return void
        **************************************************************************/
        void cleanup();

        /**
        ***************************************************************************
        * @fn PerlinNoise
        * @brief generate 2D Image in CPU
        * @return int
        **************************************************************************/
        int PerlinNoiseCPU(const unsigned int &pos);

        /**
        ***************************************************************************
        * @fn interpolatedNoise
        * @brief generate 2D Image in CPU
        * @return float
        **************************************************************************/
        float interpolatedNoise(float x,float y);

        /**
        ***************************************************************************
        * @fn findnoise
        * @brief random number generation
        * @return float
        **************************************************************************/
        float findnoise(float x,float y);

        /**
        ***************************************************************************
        * @fn interpolate
        * @brief co-sine interpolation
        * @return float
        **************************************************************************/
        inline float interpolate(float a,float b,float x);

        /**
        ***************************************************************************
        * @fn smoothNoise
        * @brief generate 2D Image in CPU
        * @return float
        **************************************************************************/
        float smoothNoise(float x, float y);



};



#endif


