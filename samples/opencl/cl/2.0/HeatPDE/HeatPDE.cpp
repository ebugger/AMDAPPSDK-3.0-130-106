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

#include "HeatPDE.hpp"

HeatPDE clHeatPDE;

#include "HeatPDE_GL.hpp"


void* feedbackThread(void *data1)
{

	while(1)
	{
		int status = clHeatPDE.feedbackAndControl();
		CHECK_OPENCL_ERROR_RETURN_NULL(status, "clEnqueueMapBuffer failed !!");
			
		if(ready)
		{
			feedback_stop = true;
			cv_feedBack.notify_one();
			break;
		}
	}
	return NULL;
}

void* clExecutionThread(void *data1)
{
	while(1)
	{
		int status = clHeatPDE.runCLKernels() ;
		CHECK_OPENCL_ERROR_RETURN_NULL(status, "clEnqueueMapBuffer failed !!");

		if(ready) 
		{
			clExex_stop = true;
			cv_clExec.notify_one();
			break;
		}

	}
	return NULL;
}

int HeatPDE::setupHeatPDE()
{
	/* define conductivity field */
	pCondField = (cl_float *)malloc(condFieldSize*sizeof(cl_float));
	CHECK_ALLOCATION(pCondField,"memory allocation failure.(pCondField)");

	cl_float cond = CONDUCTIVITY;

	for(unsigned int i = 0; i < condFieldSize; ++i)
	pCondField[i] = cond;

	pHeatImage = (cl_uint *)malloc(condFieldSize*sizeof(cl_uint));
	CHECK_ALLOCATION(pCondField,"memory allocation failure.(pHeatImage)");

	for(unsigned int y = 0; y < sizey +PAD_SIZE; ++y)
	{
		unsigned int offset = y*(sizex +PAD_SIZE);
		pHeatImage[offset]  = 0;
		for(unsigned int x = 1; x < sizex + PAD_SIZE; ++x)
		{
			pHeatImage[offset + x] = 0;
		}
	}
	return SDK_SUCCESS;
}

int HeatPDE::setupBurnersAndSensors()
{
	pBurnerPosX  = (cl_uint *)malloc(burnerCountX*sizeof(cl_uint));  
	pBurnerPosY  = (cl_uint *)malloc(burnerCountY*sizeof(cl_uint));  
	pBurnerState = (cl_uchar *)malloc(burnerCountX*burnerCountY*sizeof(cl_uchar));  

	pSensorPosX  = (cl_uint *)malloc(sensorCountX*sizeof(cl_uint));  
	pSensorPosY  = (cl_uint *)malloc(sensorCountY*sizeof(cl_uint));  
	pSensorState = (cl_uchar *)malloc(sensorCountX*sensorCountY*sizeof(cl_uchar));
	pSensorData  = (cl_float *)malloc(sensorCountX*sensorCountY*sizeof(cl_float));    
	pSensorMin   = (cl_float *)malloc(sensorCountX*sensorCountY*sizeof(cl_float));    
	pSensorMax   = (cl_float *)malloc(sensorCountX*sensorCountY*sizeof(cl_float));    

	//put burners in their positions
	cl_uint distX = sizex/(burnerCountX);
	cl_uint distY = sizey/(burnerCountY);

	for(cl_uint count = 0; count < burnerCountX; ++count)
	{
		pBurnerPosX[count] = count*distX + distX/2;
	}

	for(cl_uint count = 0; count < burnerCountY; ++count)
	{
		pBurnerPosY[count] = count*distY + distY/2;
	}

	for(cl_uint count = 0; count < sensorCountX; ++count)
	{
		pSensorPosX[count] = pBurnerPosX[count] + distX/2;
	}

	for(cl_uint count = 0; count < sensorCountY; ++count)
	{
		pSensorPosY[count] = pBurnerPosY[count] + distY/2;
	}

	for(cl_uint count = 0; count < sensorCountX*sensorCountY; ++count)
	{
		pSensorData[count]  = 0.0;
		pSensorState[count] = SENSOR_OFF;
	}

	pSensorState[1] = SENSOR_ON;
	pSensorMin[1]   = 6000.0;
	pSensorMax[1]   = 9000.0;

	pSensorState[3] = SENSOR_ON;
	pSensorMin[3]   = 7000.0;
	pSensorMax[3]   = 10000.0;

	pSensorState[5] = SENSOR_ON;
	pSensorMin[5]   = 5000.0;
	pSensorMax[5]   = 10000.0;

	pSensorState[7] = SENSOR_ON;
	pSensorMin[7]   = 8000.0;
	pSensorMax[7]   = 12000.0;

	return SDK_SUCCESS;
}

int HeatPDE::runGUI(int argc, char* argv[])
{
	/* initial and boundary conditions */
	if(setupInitialConditions() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}
  
	/* initialize gui and loop */
	glutInit(&argc,argv);
	glutInitWindowSize(GUI_WINDOW_WIDTH,GUI_WINDOW_HEIGHT);
	glutInitWindowPosition(GUI_WINDOW_POS_X,GUI_WINDOW_POS_Y);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  
	glutCreateWindow("Heat Field");

	// Creating threads for OpenCL Exection of the Heat Equation and feedback/Control of Burners .
	SDKThread clThread ;
	SDKThread feedBackThread ;
	
	clThread.create(clExecutionThread,NULL);
	feedBackThread.create(feedbackThread,NULL);
	
	//callback GL functions
	glutDisplayFunc(displayGL);
	glutKeyboardFunc(keyboardGL);
	glutIdleFunc(idleGL);

	glutMainLoop();

	clThread.join();
	feedBackThread.join();
	return 0;
}

int HeatPDE::setupCL(void)
{
	cl_int         status = 0;
	cl_device_type dType;
  
	if(sampleArgs->deviceType.compare("cpu") == 0)
	{
		dType = CL_DEVICE_TYPE_CPU;
	}
	else //deviceType = "gpu"
	{
		dType = CL_DEVICE_TYPE_GPU;
		if(sampleArgs->isThereGPU() == false)
		{
		std::cout << "GPU not found. Falling back to CPU device" << std::endl;
		dType = CL_DEVICE_TYPE_CPU;
		}
	}
  
	// get platform
	cl_platform_id platform = NULL;

	status = getPlatform(platform, 
				sampleArgs->platformId,
				sampleArgs->isPlatformEnabled());
	CHECK_ERROR(status, SDK_SUCCESS, "getPlatform() failed");
  
	// display all available devices.
	status = displayDevices(platform, dType);
	CHECK_ERROR(status, SDK_SUCCESS, "displayDevices() failed");
    
	// if we could find our platform, use it. Otherwise use just available 
	// platform.
	cl_context_properties cps[3] =
	{
		CL_CONTEXT_PLATFORM,
		(cl_context_properties)platform,
		0
	};
  
	context = clCreateContextFromType(cps,
					dType,
					NULL,
					NULL,
					&status);
	CHECK_OPENCL_ERROR(status, "clCreateContextFromType failed.");
  
	status = getDevices(context, 
				&devices, 
				sampleArgs->deviceId,
				sampleArgs->isDeviceIdEnabled());
	CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");
  
	// set device info of given cl_device_id
	status = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
	CHECK_ERROR(status, SDK_SUCCESS, "SDKDeviceInfo::setDeviceInfo() failed");

	// check of OPENCL_C_VERSION if device version is 2.0 or higher
	int isOpenCL2_XSupported = deviceInfo.checkOpenCL2_XCompatibility();
	if (!isOpenCL2_XSupported)
	{
		OPENCL_EXPECTED_ERROR("Unsupported device! Required CL_DEVICE_OPENCL_C_VERSION 2.0 or higher");
	}

  //check SVM capabilities Finegrain and Atomics

  if (!(deviceInfo.svmcaps & CL_DEVICE_SVM_ATOMICS))
  {
	OPENCL_EXPECTED_ERROR("Unsupported device! Device does not support SVM Atomics");
  }
  
	// create command queue
	cl_queue_properties prop[] = {0};
	commandQueue = clCreateCommandQueueWithProperties(
					context,
			devices[sampleArgs->deviceId],
			prop,
			&status);
	CHECK_OPENCL_ERROR(status, "clCreateCommandQueue failed.");

	// create a CL program using the kernel source
	buildProgramData buildData;
	buildData.kernelName = std::string("HeatPDE_Kernels.cl");
	buildData.devices    = devices;
	buildData.deviceId   = sampleArgs->deviceId;
	buildData.flagsStr   = std::string("-I. -cl-std=CL2.0");
  
	if(sampleArgs->isLoadBinaryEnabled())
	{
		buildData.binaryName = std::string(sampleArgs->loadBinary.c_str());
	}

	if(sampleArgs->isComplierFlagsSpecified())
	{
		buildData.flagsFileName = std::string(sampleArgs->flags.c_str());
	}

	status = buildOpenCLProgram(program, context, buildData);
	CHECK_ERROR(status, SDK_SUCCESS, "buildOpenCLProgram() failed");

	// get a kernel object handle for a kernel with the given name
	pdeKernel = clCreateKernel(program, "pdeKernel", &status);
	CHECK_OPENCL_ERROR(status, "clCreateKernel::pdeKernel failed.");

	tempToRgbKernel = clCreateKernel(program, "tempToRgbKernel", &status);
	CHECK_OPENCL_ERROR(status, "clCreateKernel::tempToRgbKernel failed.");

	/* conductivity field */
	clCondField = clCreateBuffer(context,
					CL_MEM_READ_ONLY,
					sizeof(cl_float)*condFieldSize,
					NULL,
					&status);
	CHECK_OPENCL_ERROR(status,"clCreateBuffer Failed.(clCondField)");

	status = clEnqueueWriteBuffer(commandQueue,
				clCondField,
				CL_TRUE,
				0,
				sizeof(cl_float)*condFieldSize,
				(void *)pCondField,
				0,
				NULL,
				NULL);
	CHECK_OPENCL_ERROR(status,"clEnqueueWriteBuffer Failed.(clCondField)");

	clHeatImage = clCreateBuffer(context,
					CL_MEM_READ_WRITE,
					sizeof(cl_uint)*condFieldSize,
					NULL,
					&status);
	CHECK_OPENCL_ERROR(status,"clCreateBuffer Failed.(clHeatImage)");

	/* heat field */
	pSVMBuf = clSVMAlloc(context,
				CL_MEM_READ_WRITE|CL_MEM_SVM_FINE_GRAIN_BUFFER,
				sizeof(cl_float)*2*condFieldSize,
				0);
  
	if(NULL == pSVMBuf)
	{
		std::cout << "SVM Buffer allocation failed.(pSVMBuf)" << std::endl;
		return SDK_FAILURE;
	}

	/* Buffer used to check whether GPU has to update  */
	pSVMControlBuf = (unsigned int*)clSVMAlloc(context,
				CL_MEM_READ_WRITE|CL_MEM_SVM_FINE_GRAIN_BUFFER|CL_MEM_SVM_ATOMICS,
				sizeof(unsigned int)*condFieldSize,
				0);
  
	if(NULL == pSVMControlBuf)
	{
		std::cout << "SVM Buffer allocation failed.(pSVMControlBuf)" << std::endl;
		return SDK_FAILURE;
	}

	return SDK_SUCCESS;
}

int HeatPDE::setupInitialConditions(void)
{
	int status        = SDK_SUCCESS;

	//put the initial field to zero
	pPingHeatField = (cl_float *)(pSVMBuf);
	pPongHeatField = pPingHeatField + condFieldSize;

	memset(pSVMControlBuf, GPU_UPDATE, sizeof(unsigned int)*condFieldSize);
	pControlField = pSVMControlBuf;


	for(cl_uint y = 0; y < sizey+PAD_SIZE; ++y)
	{
		cl_uint row_offest = y*(sizex+PAD_SIZE);
		for(cl_uint x = 0; x < sizex+PAD_SIZE; ++x)
		{
		pPingHeatField[row_offest + x] = 0.0;
		pPongHeatField[row_offest + x] = 0.0;
		}
	}

	//burners
 
	for(cl_uint countY = 0; countY < burnerCountY; ++countY)
	{
		cl_uint posY = pBurnerPosY[countY];
		for(cl_uint countX = 0; countX < burnerCountX; ++countX)
  		{
  		cl_uint posX = pBurnerPosX[countX];
	  
  			for(cl_int y = -burnerSizeY; y < burnerSizeY; ++y)
			{
				cl_uint offset = (posY+y)*(sizex+PAD_SIZE);
				for(cl_int x = -burnerSizeX; x < burnerSizeX; ++x)
				{
				pPingHeatField[offset + posX +x] = BURNER_HEAT;
				pPongHeatField[offset + posX +x] = BURNER_HEAT;
				pControlField[offset + posX +x] = CPU_UPDATE;
				}
			}
		}
	}

	return SDK_SUCCESS;
}

int HeatPDE::feedbackAndControl(void)
{
	int status           = SDK_SUCCESS;
	cl_uchar burnerState = CPU_UPDATE;

	//get the sensor data as feedback 
	for(cl_uint i = 0; i < sensorCountX*sensorCountY; ++i)
	{
		//check if the sensor is present
		if(pSensorState[i] == SENSOR_ON)
		{
		//find its location
		cl_uint sensorY    = i/sensorCountX;
		cl_uint sensorX    = i - sensorY*sensorCountX;

		cl_uint sensorPosX = pSensorPosX[sensorX];
		cl_uint sensorPosY = pSensorPosY[sensorY];

		cl_uint sensorPos  = sensorPosY*(sizex+PAD_SIZE) + sensorPosX;

		pSensorData[i]     = pPingHeatField[sensorPos];

		//switch burners on or off based on feedback. 
			if(pSensorData[i] < pSensorMin[i])
			{
				for(cl_uint c = 0; c < 2; ++c)
				{
				cl_uint burnerOffset = (sensorY + c)*burnerCountX;
					for(cl_uint r = 0; r < 2; ++r)
					{
						cl_uint burnerPos = burnerOffset + sensorX + r;
						pBurnerState[burnerPos] = BURNER_ON;
					}
				}
			}
			else if(pSensorData[i] > pSensorMax[i] || (pSensorData[i] > pSensorMin[i] && pSensorData[i] < pSensorMax[i]))
			{
				for(cl_uint c = 0; c < 2; ++c)
				{
				cl_uint burnerOffset = (sensorY + c)*burnerCountX;
					for(cl_uint r = 0; r < 2; ++r)
					{
						cl_uint burnerPos = burnerOffset + sensorX + r;
						pBurnerState[burnerPos] = BURNER_OFF;
					}
				}
			}
		}
	}

	//make the burners on
	for(cl_uint countY = 0; countY < burnerCountY; ++countY)
	{
		cl_uint posY         = pBurnerPosY[countY];
		cl_uint countOffset  = countY*burnerCountX;
		for(cl_uint countX = 0; countX < burnerCountX; ++countX)
  		{
  		cl_uint  posX         = pBurnerPosX[countX];
		cl_uchar burnerState  = pBurnerState[countOffset +countX];

  			for(cl_int y = -burnerSizeY; y < burnerSizeY; ++y)
			{
				cl_uint offset = (posY+y)*(sizex+PAD_SIZE);
				for(cl_int x = -burnerSizeX; x < burnerSizeX; ++x)
				{
					if(burnerState == BURNER_ON)
					{
					  std::atomic_store((std::atomic<int>*)&pControlField[offset + posX +x], CPU_UPDATE);
					  pPingHeatField[offset + posX +x] = BURNER_HEAT;
					  pPongHeatField[offset + posX +x] = BURNER_HEAT;
					}
					else
					{
					  std::atomic_store((std::atomic<int>*)&pControlField[offset + posX +x], GPU_UPDATE);
					}					
				}
			}
		}
	}
  
	return SDK_SUCCESS;
}

int HeatPDE::genBinaryImage()
{
	bifData binaryData;
	binaryData.kernelName = std::string("HeatPDE_Kernels.cl");
	binaryData.flagsStr = std::string("");
	if(sampleArgs->isComplierFlagsSpecified())
	{
		binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
	}
	binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
	int status = generateBinaryImage(binaryData);
	return status;
}

int HeatPDE::runCLKernels(void)
{
	cl_int    status;
	cl_float* pTempBuf;

	size_t    globalThreads[] = {sizex,sizey};

	/* Set kernel arguments */
	status = clSetKernelArg(pdeKernel,
				0,
				sizeof(cl_int),
				(void *)(&sizex));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(sizex) failed.");

	status = clSetKernelArg(pdeKernel,
				1,
				sizeof(cl_int),
				(void *)(&sizey));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(sizex) failed.");

	status = clSetKernelArg(pdeKernel,
				2,
				sizeof(cl_mem),
				(void *)(&clCondField));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(clCondField) failed.");

	for(cl_uint i = 0; i < pde_iter; ++i)
	{
	status = clSetKernelArgSVMPointer(pdeKernel,
						3,
						(void *)(pPingHeatField));
	CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer(pPingHeatField) failed.");

	status = clSetKernelArgSVMPointer(pdeKernel,
						4,
						(void *)(pPongHeatField));
	CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer(pPingHeatField) failed.");
	
	status = clSetKernelArgSVMPointer(pdeKernel,
						5,
						(unsigned int *)(pSVMControlBuf));
	CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer(pSVMControlBuf) failed.");

	cl_event ndrEvt;
	status = clEnqueueNDRangeKernel(commandQueue,
					pdeKernel,
					2,
					NULL,
					globalThreads,
					NULL,
					0,
					NULL,
					&ndrEvt);
	CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");
	
	status = clFlush(commandQueue);
	CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");
	
	status = waitForEventAndRelease(&ndrEvt);
	CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(ndrEvt) Failed");
	
	pTempBuf       = pPingHeatField;
	pPingHeatField = pPongHeatField;
	pPongHeatField = pTempBuf;
	}

	if(isGUI)
	{
	/* generate the image */
	status = clSetKernelArg(tempToRgbKernel,
				0,
				sizeof(cl_int),
				(void *)(&sizex));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(sizex) failed.");

	status = clSetKernelArg(tempToRgbKernel,
				1,
				sizeof(cl_int),
				(void *)(&sizey));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(sizex) failed.");

	status = clSetKernelArgSVMPointer(tempToRgbKernel,
						2,
						(void *)(pPingHeatField));
	CHECK_OPENCL_ERROR(status, "clSetKernelArgSVMPointer(pPingHeatField) failed.");

	status = clSetKernelArg(tempToRgbKernel,
				3,
				sizeof(cl_mem),
				(void *)(&clHeatImage));
	CHECK_OPENCL_ERROR(status, "clSetKernelArg(clCondField) failed.");

	globalThreads[0] = sizex +PAD_SIZE;
	globalThreads[1] = sizey +PAD_SIZE;    

	status = clEnqueueNDRangeKernel(commandQueue,
					tempToRgbKernel,
					2,
					NULL,
					globalThreads,
					NULL,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status, "clEnqueueNDRangeKernel failed.");
	
	status = clFinish(commandQueue);
	CHECK_OPENCL_ERROR(status, "clFlush failed.(commandQueue)");
	
	status = clEnqueueReadBuffer(commandQueue,
					clHeatImage,
					CL_TRUE,
					0,
					sizeof(cl_uint)*condFieldSize,
					(void *)pHeatImage,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR(status,"clEnqueueReadBuffer Failed.(clHeatImage)");
	}

	return SDK_SUCCESS;
}

int HeatPDE::cpuReference()
{  
	unsigned int svmBufSize = 2*condFieldSize;
	cl_float* pTempBuf;

	int timer = sampleTimer->createTimer();
	
	pHeatfield = (cl_float *)malloc(sizeof(cl_float)*svmBufSize);
		  
	for(int iter_CPU = 0 ; iter_CPU < (iterations) ; iter_CPU++)
	{
 
		float * pCondField_cpu = (float *) pCondField ;

		float *pPingHeatField_CPU = (cl_float *)(pHeatfield);
		float * pPongHeatField_CPU =  pPingHeatField_CPU + condFieldSize;

		memset(pSVMControlBuf, GPU_UPDATE, sizeof(unsigned int)*condFieldSize);
		pControlField = pSVMControlBuf;

		for(cl_uint y = 0; y < sizey +PAD_SIZE; ++y)
		{
			cl_uint row_offest = y*(sizex+PAD_SIZE);
			for(cl_uint x = 0; x < sizex+PAD_SIZE; ++x)
			{
			pPingHeatField_CPU[row_offest + x] = 0.0;
			pPongHeatField_CPU[row_offest + x] = 0.0;
			}
		}


		for(cl_uint countY = 0; countY < burnerCountY; ++countY)
		{
			cl_uint posY = pBurnerPosY[countY];
			for(cl_uint countX = 0; countX < burnerCountX; ++countX)
  			{
  			cl_uint posX = pBurnerPosX[countX];
	  
  				for(cl_int y = -burnerSizeY; y < burnerSizeY; ++y)
				{
					cl_uint offset = (posY+y)*(sizex+PAD_SIZE);
					for(cl_int x = -burnerSizeX; x < burnerSizeX; ++x)
					{
					pPingHeatField_CPU[offset + posX +x] = BURNER_HEAT;
					pPongHeatField_CPU[offset + posX +x] = BURNER_HEAT;
					pControlField[offset + posX +x] = CPU_UPDATE;
					}
				}
			}
		}

		sampleTimer->resetTimer(timer);
		sampleTimer->startTimer(timer);

		for(int pdeIter= 0 ; pdeIter < ((int)pde_iter);pdeIter++)
		{
			//No support for OpenMP 3.0 and higher in MSVC yet. Hence the below check
#if defined(_MSC_VER) 
			#pragma omp parallel for 
#else
			#pragma omp parallel for collapse(2)
#endif
			for(int row_x=0 ; row_x < (int)(sizex) ;row_x++)
			{
				for(int row_y=0 ; row_y < (int)(sizey) ;row_y++)
				{
					int x     = row_x + 1;
					int y     = row_y + 1;
					int sizeX = sizex;
					int sizeY = sizey;
  
					int exSizeX = sizex + PAD_SIZE;
					int exSizeY = sizey + PAD_SIZE;

					int c = y*exSizeX + x;
					int t = (y-1)*exSizeX + x;
					int l = c -1;
					int b = (y+1)*exSizeX + x;
					int r = c +1;

				  
					float *prevField     = (float *)pPingHeatField_CPU;
					float *nextField     = (float *)pPongHeatField_CPU;
					
					//heat equation
					float laplacian = prevField[t] + prevField[l] +  
									prevField[b] + prevField[r];  

					laplacian = laplacian - (float)4.0*prevField[c];

					if (pControlField[c] == GPU_UPDATE)
						nextField[c] = pCondField_cpu[c]*laplacian + prevField[c]; 
					else
						nextField[c] = prevField[c]; 						
				}
			}
		
			pTempBuf			 = pPingHeatField_CPU;
			pPingHeatField_CPU = pPongHeatField_CPU;
			pPongHeatField_CPU = pTempBuf;
		}

		sampleTimer->stopTimer(timer);
		cpuRunTime += (cl_double)sampleTimer->readTimer(timer);
	}
   
	return SDK_SUCCESS;
}

int HeatPDE::initialize()
{
	// Call base class Initialize to get default configuration
	if(sampleArgs->initialize() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}
  
	Option* new_option = new Option;
	CHECK_ALLOCATION(new_option, "Memory allocation error. (new_option)");
  
	new_option->_sVersion = "i";
	new_option->_lVersion = "iterations";
	new_option->_description = "Number of iterations for kernel execution";
	new_option->_type = CA_ARG_INT;
	new_option->_value = &iterations;
  
	sampleArgs->AddOption(new_option);

  
	new_option->_sVersion = "g";
	new_option->_lVersion = "gui";
	new_option->_description = "Display simulation on a GUI";
	new_option->_type = CA_NO_ARGUMENT;
	new_option->_value = &isGUI;
  
	sampleArgs->AddOption(new_option);

	delete new_option;
  
	return SDK_SUCCESS;
}

int HeatPDE::setup()
{
	if(setupHeatPDE() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	if(setupBurnersAndSensors() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	if(sampleArgs->verify)
		isGUI = false;
  
	int timer = sampleTimer->createTimer();
	sampleTimer->resetTimer(timer);
	sampleTimer->startTimer(timer);
  
	int status = setupCL();
	if (status != SDK_SUCCESS)
	{
		return status;
	}
  
	sampleTimer->stopTimer(timer);
	setupTime = (cl_double)sampleTimer->readTimer(timer);

	return SDK_SUCCESS;
}

int HeatPDE::run()
{
	int status = 0;

	/* Set initial and boundary conditions */
	if(setupInitialConditions() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	//warm up run
	if(runCLKernels() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	std::cout << "-------------------------------------------" << std::endl;
	std::cout << "Executing kernel for " << iterations
			<< " iterations" << std::endl;
	std::cout << "-------------------------------------------" << std::endl;
    
	int timer = sampleTimer->createTimer();
	
    
	for(int i = 0; i < iterations; i++)
	{

		if(setupInitialConditions() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
		
		sampleTimer->resetTimer(timer);
		sampleTimer->startTimer(timer);
		// Arguments are set and execution call is enqueued on command buffer
		if(runCLKernels() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
		sampleTimer->stopTimer(timer);

		kernelTime += (double)(sampleTimer->readTimer(timer));
	}
    
	

	return SDK_SUCCESS;
}

int HeatPDE::compare()
{
  float        diff;
  unsigned int count = 0;
  float* pGpuHeatField = (float *)pSVMBuf;

  for(unsigned int i = 0; i < condFieldSize; ++i)
    {
      diff = pHeatfield[i] - pGpuHeatField[i];
      if (diff < 0) 
	diff = -diff;

      if(diff > EPSILON)
	{
	  std::cout << "[" << i << "]:" << EPSILON*BURNER_HEAT << ":";
	  std::cout << pHeatfield[i] << ":";
	  std::cout << pGpuHeatField[i] << std::endl;

	  count += 1;

	}
    }

  if(count)
    return SDK_FAILURE;

  return SDK_SUCCESS;
}

int HeatPDE::verifyResults()
{
	int status = SDK_SUCCESS;
	if(sampleArgs->verify)
	{
		// reference implementation
		cpuReference();
      
		// compare the results and see if they match
		int isPass = compare();

		if(isPass == SDK_SUCCESS)
		{
			std::cout<<"Passed!\n" << std::endl;
			return SDK_SUCCESS;
		}
		else
		{
			std::cout<<"Failed\n" << std::endl;
			return SDK_FAILURE;
		}
	
	}

	return status;
}

int HeatPDE::cleanup()
{
	// Releases OpenCL resources (Context, Memory etc.)
	cl_int status = 0;

	status = clReleaseMemObject(clCondField);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(clCondField)");

	status = clReleaseMemObject(clHeatImage);
	CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(clCondField)");

	if(pSVMBuf)
		clSVMFree(context,pSVMBuf);
	

	status = clReleaseKernel(pdeKernel);
	CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(pdeKernel)");
	

	status = clReleaseKernel(tempToRgbKernel);
	CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(tempToRgbKernel)");

	status = clReleaseProgram(program);
	CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");

	status = clReleaseCommandQueue(commandQueue);
	CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");

	status = clReleaseContext(context);
	CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");

	FREE(pBurnerPosX);
	FREE(pBurnerPosY);
	FREE(pBurnerState);

	FREE(pSensorPosX);
	FREE(pSensorPosY);
	FREE(pSensorState);
	FREE(pSensorData);
	FREE(pSensorMin);
	FREE(pSensorMax);

	FREE(pCondField);
	FREE(pHeatImage);
	FREE(devices);
	FREE(pHeatfield);

	return SDK_SUCCESS;
}

void HeatPDE::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[5] =
        {
            "Size X",
            "Size Y",
            "Setup Time(sec)",
            "Avg. kernel time (sec)",
            "Avg. CPU Execution Time (sec)",
        };
        std::string stats[5];
        double avgKernelTime = kernelTime / iterations;
	double avgCputime    = cpuRunTime/iterations;

        stats[0] = toString(sizex, std::dec);
        stats[1] = toString(sizey, std::dec);
        stats[2] = toString(setupTime, std::dec);
        stats[3] = toString(avgKernelTime, std::dec);
	stats[4] = toString(avgCputime, std::dec);
		
        printStatistics(strArray, stats, 5);
    }
}



int main(int argc, char * argv[])
{
	// Initialize
	if(clHeatPDE.initialize() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	if(clHeatPDE.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
	{
		return SDK_EXPECTED_FAILURE;
	}

	if(clHeatPDE.sampleArgs->isDumpBinaryEnabled())
	{
		//GenBinaryImage
		return clHeatPDE.genBinaryImage();
	}

	// Setup
	int status = clHeatPDE.setup();
	if(status != SDK_SUCCESS)
	{
		return status;
	}

	// Run
	if(clHeatPDE.isGUI)
	{
		clHeatPDE.runGUI(argc,argv);
	}
	else
	{
		if(clHeatPDE.run() != SDK_SUCCESS)
		{
			return SDK_FAILURE;
		}
	}

	// VerifyResults
	if(clHeatPDE.verifyResults() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	// Cleanup
	if (clHeatPDE.cleanup() != SDK_SUCCESS)
	{
		return SDK_FAILURE;
	}

	clHeatPDE.printStats();
	return SDK_SUCCESS;
}
