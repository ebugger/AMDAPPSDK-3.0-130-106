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

#include "PerlinNoise.hpp"
#include "SDKBitMap.hpp"

int PerlinNoiseBolt::init()
{
    // Delete -samples option.
    Option numSamples;
    numSamples._sVersion = "x";
    numSamples._lVersion = "samples";
    numSamples._description = "Number of sample input values.";

    (this->sampleArgs)->DeleteOption(&numSamples);

    // Adding additional options.
    Option rOption;
    rOption._sVersion = "r";
    rOption._lVersion = "red";
    rOption._description = "Red component of the Image [ 0 - 255]";
    rOption._type = CA_ARG_INT;
    rOption._value = &red;

    if ( red < 0 || red  > 255  )
    {
        std::cout << "\n Error: Valid range for RGB component is [0,255]";
        exit(SDK_EXPECTED_FAILURE);
    }


    (this->sampleArgs)->AddOption(&rOption);

    Option gOption;
    gOption._sVersion = "g";
    gOption._lVersion = "green";
    gOption._description = "Green component of the Image [ 0 - 255]";
    gOption._type = CA_ARG_INT;
    gOption._value = &green;


    (this->sampleArgs)->AddOption(&gOption);

    Option bOption;
    bOption._sVersion = "b";
    bOption._lVersion = "blue";
    bOption._description = "Blue component of the Image [ 0 - 255]";
    bOption._type = CA_ARG_INT;
    bOption._value = &blue;


    (this->sampleArgs)->AddOption(&bOption);

    if ( red < 0 || red  > 255 ||
            green > 255 || green  < 0 ||
            blue  < 0 || blue > 255
       )
    {
        std::cout << "\n Error: Valid range for RGB component is [0,255]";
        exit(SDK_EXPECTED_FAILURE);
    }

    Option zoomOption;
    zoomOption._sVersion = "z";
    zoomOption._lVersion = "zoom";
    zoomOption._description = "zoom-in / zoom-out ";
    zoomOption._type = CA_ARG_FLOAT;
    zoomOption._value = &zoom;


    (this->sampleArgs)->AddOption(&zoomOption);

    Option pOption;
    pOption._sVersion = "";
    pOption._lVersion = "per";
    pOption._description = "Persistence: Controls roughness of the Image";
    pOption._type = CA_ARG_FLOAT;
    pOption._value = &persistence;


    (this->sampleArgs)->AddOption(&pOption);

    Option octOption;
    octOption._sVersion = "";
    octOption._lVersion = "octaves";
    octOption._description =
        "Number of iterations of coherent noise functions/pixel";
    octOption._type = CA_ARG_INT;
    octOption._value = &octaves;


    (this->sampleArgs)->AddOption(&octOption);

    return SDK_SUCCESS;
}


int PerlinNoiseBolt::setup()
{
    /****************************************************************************************
    * Get the default bolt control object. 'boltControlObj' is a reference to  default
    * bolt control object. Any changes to it is reflected globally.
    *****************************************************************************************/
    sampleArgs->boltControlObj = &(bolt::cl::control::getDefault());

    if( !sampleArgs->enable_tbb &&
            (strComparei(sampleArgs->runMode, "multicorecpu")) )
    {
        std::cout << "\nError:Use TBB configuration to run in multi-core CPU";
        return SDK_EXPECTED_FAILURE;
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

    outputBufferCPU.resize(width*height);
    outputBufferBOLT.resize(width*height);
    inputPixels.resize(width * height);

    unsigned int count =0;
    std::generate(inputPixels.begin(),inputPixels.end(),[&count]()
    {
        return count++;
    }
                 );

    return SDK_SUCCESS;

}

int PerlinNoiseBolt::runPerlinNoise()
{

    //For each pixel position run the binary operator in struct PerlinNoise
    bolt::cl::transform(inputPixels.begin(),inputPixels.end(),
                        outputBufferBOLT.begin(),
                        PerlinNoise(width,height,zoom,persistence,octaves,red,green,blue));

    return SDK_SUCCESS;
}

int PerlinNoiseBolt::run()
{
    int timer =  sampleTimer->createTimer();

    if(sampleArgs->iterations > 1 )
    {
        if(runPerlinNoise() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
        if(!sampleArgs->quiet)
        {
            std::cout << "Completed Warm-up run of Bolt code" << std::endl;
        }
    }
    std::cout << "Executing sample over " << sampleArgs->iterations
              << " iteration(s)..." << std::endl;

    sampleTimer->startTimer(timer);

    for(int i=0; i < sampleArgs->iterations; i++)
    {

        if(runPerlinNoise() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

    }

    sampleTimer->stopTimer(timer);

    avgExecTime = (float)(sampleTimer->readTimer(timer));
    avgExecTime /= sampleArgs->iterations;

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Run() of Perlin Noise sample" << std::endl;
    }

    SDKBitMap NewBitMapImage;
    NewBitMapImage.write("Clouds_Bolt.bmp",width,height,outputBufferBOLT.data());

    return SDK_SUCCESS;
}

float PerlinNoiseBolt::interpolate(float a,float b,float x)
{
    float ft=x * 3.1415;
    float f=(1.0-cos(ft))* 0.5;
    return a*(1.0-f)+b*f;
}


float PerlinNoiseBolt::findnoise(float x,float y)
{
    int n=(int)x+(int)y*57;
    n=(n<<13)^n;
    int nn=(n*(n*n*60493+19990303)+1376312589)&0x7fffffff;
    return  1.0-(nn/1073741824.0);
}

float PerlinNoiseBolt::smoothNoise(float x, float y)
{
    float corners = ( findnoise(x-1, y-1)+findnoise(x+1, y-1)+findnoise(x-1,
                      y+1)+findnoise(x+1, y+1) ) / 16;
    float sides   = ( findnoise(x-1, y)  +findnoise(x+1, y)  +findnoise(x,
                      y-1)  +findnoise(x, y+1) ) /  8;
    float center  =  findnoise(x, y) / 4;

    return corners + sides + center;
}

float PerlinNoiseBolt::interpolatedNoise(float x,float y)
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



int PerlinNoiseBolt::PerlinNoiseCPU(const unsigned int &pos)
{
    float  getnoise = 0;
    int x= pos % width;
    int y =pos / height;
    float frequency,amplitude;
    for(unsigned int a= 0; a<octaves-1; a++)
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

    outputBufferCPU[pos] = (r + g+ b);

    return SDK_SUCCESS;
}

int PerlinNoiseBolt::runPerlinNoiseCPU()
{
    for (unsigned int i = 0 ; i < inputPixels.size() ; i++)
    {
        PerlinNoiseCPU(i);
    }

    SDKBitMap NewBitMapImage;
    NewBitMapImage.write("Output.bmp",width,height,outputBufferBOLT.data());
    return SDK_SUCCESS;
}


int PerlinNoiseBolt::verifyResults()
{
    if(runPerlinNoiseCPU() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    for(unsigned int i = 0 ; i< width * height ; i++)
    {

          int red_cpu = outputBufferCPU[i] >>16;
          int red_gpu = outputBufferBOLT[i]>> 16;

          //Obtain the value of "color".
          float color_cpu =  (red_cpu * 255)/red;
          float color_gpu =  (red_gpu * 255)/red;

          if ( abs(color_gpu - color_cpu) > 1) 
          {    
            std::cout << "Data mismatch at Output Vector Position " << i;
            std::cout << "\nFailed!";
            return SDK_FAILURE;
        }
    }

    std::cout << "\nPassed!";

    return SDK_SUCCESS;
}

void PerlinNoiseBolt::printStats()
{
    std::string strArray[4] = {"Width of the Image","Height of the Image", "Avg. Execution Time(sec)", "Values/sec"};

    std::string stats[4];
    stats[0] = toString<int>(width);
    stats[1] = toString<int>(height);
    stats[2] = toString(avgExecTime);
    stats[3] = toString(inputPixels.size() / avgExecTime);
    printStatistics(strArray, stats, 4);
}


int main(int argc, char * argv[])
{
    std::cout << "**************************************************" << std::endl;
    std::cout << "Generation of 2D texture (Perlin Noise) using BOLT" << std::endl;
    std::cout << "**************************************************" << std::endl;

    /**************************************************************************
    * Create an object of SpMV class                                          *
    **************************************************************************/
    PerlinNoiseBolt objNoise;

    /**************************************************************************
    * Add sample specific command line options                                *
    **************************************************************************/
    if(objNoise.init() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objNoise.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Bolt APIs throw exception on failure                                    *
    **************************************************************************/
    try
    {
        /**************************************************************************
        * Generate the vectors based on the width and height of the Image         *
        **************************************************************************/
        if(objNoise.setup() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        /**************************************************************************
        * Execute algorithm using BOLT                                            *
        **************************************************************************/
        if(objNoise.run() != SDK_SUCCESS)
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
    * Verify the results that were generated.                                 *
    **************************************************************************/
    if(objNoise.sampleArgs->verify)
    {
        if(objNoise.verifyResults() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    /**************************************************************************
    * Print performance statistics.                                           *
    **************************************************************************/
    if(objNoise.sampleArgs->timing && !objNoise.sampleArgs->quiet)
    {
        objNoise.printStats();
    }

    return SDK_SUCCESS;
}