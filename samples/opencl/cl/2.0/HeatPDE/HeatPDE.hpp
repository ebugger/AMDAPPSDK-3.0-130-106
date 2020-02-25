/*****************************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this l
ist of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#ifndef _HEAT_PDE_H_
#define _HEAT_PDE_H_

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <omp.h>
#include "SDKThread.hpp"



using namespace appsdk;
using namespace std;

#define SAMPLE_VERSION "AMD-APP-SDK-vx.y.z.s"

#define SIZEX                512
#define SIZEY                256
#define CONDUCTIVITY         0.2
#define DS                   0.005f
#define DT                   0.01f 
#define PDE_ITER             50

#define BURNER_SIZE_X        10
#define BURNER_SIZE_Y        10
#define BURNER_COUNT_X       4
#define BURNER_COUNT_Y       4

#define SENSOR_COUNT_X       3
#define SENSOR_COUNT_Y       3

#define BURNER_HEAT          15000 
#define SENSOR_MAX_HEAT      10000
#define SENSOR_MIN_HEAT      9000


#define GPU_UPDATE           0                  
#define CPU_UPDATE           1                  

#define BURNER_ON            1
#define BURNER_OFF           0 

#define SENSOR_ON            1
#define SENSOR_OFF           0 

#define EPSILON              1e-04

#define PAD_SIZE			 2

#define GUI_WINDOW_WIDTH     1024
#define GUI_WINDOW_HEIGHT    512
#define GUI_WINDOW_POS_X     100
#define GUI_WINDOW_POS_Y     100


#define CHECK_OPENCL_ERROR_RETURN_NULL(actual, msg) \
    if(actual != CL_SUCCESS) \
    { \
        std::cout<<"Error :"<<msg<<" Error Code :"<<actual<<std::endl; \
        std::cout << "Location : " << __FILE__ << ":" << __LINE__<< std::endl; \
        return NULL; \
    }


static std::atomic<bool> ready;
static std::atomic<bool> clExex_stop;
static std::atomic<bool> feedback_stop;
static std::condition_variable cv_gui;
static std::condition_variable cv_clExec;
static std::condition_variable cv_feedBack;

/**
 * HeatPDE class
 */

class HeatPDE
{
private:
  /* OpenCL runtime */
  cl_context            context;      
  cl_device_id*         devices;      
  cl_command_queue      commandQueue;      
  cl_program            program;      
  cl_kernel             pdeKernel;
  cl_kernel             tempToRgbKernel;

  SDKDeviceInfo         deviceInfo;
  KernelWorkGroupInfo   kernelInfo;

  /* Timing information */
  cl_double             setupTime;   
  cl_double             kernelTime; 
  cl_double             cpuRunTime;
  SDKTimer*             sampleTimer;
  
  /* kernel iterations for exact kernel timing measurement */
  int                   iterations;      

  /* SVM buffer */
  void*                 pSVMBuf;
  cl_float*				pHeatfield ;
  cl_float*             pPingHeatField;
  cl_float*             pPongHeatField;
  unsigned int*	        pSVMControlBuf;
  unsigned int*			pControlField;

  /* conductivity field */
  cl_float*             pCondField;
  cl_mem                clCondField;

  /* heat image */
  cl_mem                clHeatImage;

  /* simulation constants */
  cl_uint               sizex;
  cl_uint               sizey;
  cl_float              ds;
  cl_float              dt;
  cl_uint               pde_iter;

  cl_uint               condFieldSize;

public:
  CLCommandArgs*        sampleArgs;   
  cl_uint*              pHeatImage;
  bool                  isGUI;

  cl_uint               sensorCountX;
  cl_uint               sensorCountY;
  cl_uint*              pSensorPosX; 
  cl_uint*              pSensorPosY; 
  cl_uchar*             pSensorState;
  cl_float*             pSensorData;
  cl_float*             pSensorMin;
  cl_float*             pSensorMax;

  cl_uint               burnerCountX;
  cl_uint               burnerCountY;
  cl_int                burnerSizeX;
  cl_int                burnerSizeY;
  cl_uint*              pBurnerPosX; 
  cl_uint*              pBurnerPosY; 
  cl_uchar*             pBurnerState;

  HeatPDE()
  {
    sampleArgs               =  new CLCommandArgs();
    sampleTimer              =  new SDKTimer();
    sampleArgs->sampleVerStr = SAMPLE_VERSION;

    setupTime                = 0.0;
    kernelTime               = 0.0;
    iterations               = 1;
    cpuRunTime		     = 0.0;

    sizex                    = SIZEX;
    sizey                    = SIZEY;
    ds                       = DS;
    dt                       = DT;
    pde_iter                 = PDE_ITER ;  

    burnerCountX             = BURNER_COUNT_X;
    burnerCountY             = BURNER_COUNT_Y;
    burnerSizeX              = BURNER_SIZE_X; 
    burnerSizeY              = BURNER_SIZE_Y; 

    sensorCountX             = SENSOR_COUNT_X;
    sensorCountY             = SENSOR_COUNT_Y;

    condFieldSize            = (sizex +2)*(sizey +2);

    isGUI                    = false;
  };
  
  ~HeatPDE()
  {
  delete sampleArgs;
  delete sampleTimer;
  };

  /**
   *************************************************************************
   * @fn     setupHeatPDE
   * @brief  Allocates and initializes any host memory pointers.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     setupHeatPDE();

  /**
   *************************************************************************
   * @fn     setupCL
   * @brief  Sets up OpenCL runtime including querying OCL divices, setting
   *         up context and command queues.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     setupCL();

  /**
   *************************************************************************
   * @fn     setupBurnersAndSensors
   * @brief  Sets up burners and sensor 
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     setupBurnersAndSensors();


  /**
   *************************************************************************
   * @fn     runGUI
   * @brief  simulates visuals of heat equation evolution.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     runGUI(int argc, char* argv[]);

  /**
   *************************************************************************
   * @fn     setupInitialConditions
   * @brief  Sets up initial conditions for the heat equation
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     setupInitialConditions();

  /**
   *************************************************************************
   * @fn     setupBoundaryConditions
   * @brief  Sets up initial conditions for the heat equation
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     feedbackAndControl();

  /**
   *************************************************************************
   * @fn     genBinaryImage
   * @brief  generates binary image of the .cl source.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     genBinaryImage();

  /**
   *************************************************************************
   * @fn     runCLKernels
   * @brief  Calls kernel functions for warm up run and then running them
   *         for number of iteration specified.Gets kernel start and end 
   *         time if timing is enabled.
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     runCLKernels();

  /**
   *************************************************************************
   * @fn     cpuReference
   * @brief  Executes an equivalent of OpenCL code on host device and 
   *         generates output used to compare with OpenCL code.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     cpuReference();

  /**
   *************************************************************************
   * @fn     initialize
   * @brief  Top level initialization. Sets up any new command line options.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     initialize();

  /**
   *************************************************************************
   * @fn     setup
   * @brief  Top level setup. Calls host side and device side setup 
   *         functions.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     setup();

  /**
   *************************************************************************
   * @fn     run
   * @brief  Top level function. Initializes data needed for kernels and
   *         calls functions executing kernels.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     run();
  
  /**
   *************************************************************************
   * @fn     verifyResults
   * @brief  Calls host reference code to generate host side output. 
   *         Compares this with OpenCL output.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure
   *************************************************************************
   */
  int     verifyResults();

  /**
   *************************************************************************
   * @fn     compare
   * @brief  compares host side and OpenCL output and establishes 
   *         correctness of the sample.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   *************************************************************************
   */
  int     compare();

  /**
   *************************************************************************
   * @fn     printStats
   * @brief  Prints stastics related to sample. This include kernel 
   *         execution time and speed with which keys are searched.
   *         
   * @return None.
   *************************************************************************
   */
  void    printStats();

  /**
   *************************************************************************
   * @fn     cleanup
   * @brief  Releases resources utilized by the program.
   *         
   * @return SDK_SUCCESS on success and SDK_FAILURE on failure.
   *************************************************************************
   */
  int     cleanup();

};
#endif
