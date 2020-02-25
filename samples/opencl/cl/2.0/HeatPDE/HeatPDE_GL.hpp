#ifndef __HEAT_PDE_GL_H__
#define __HEAT_PDE_GL_H__

#include <GL/glut.h>
#ifdef WIN32
#define snprintf sprintf_s
#endif

#define SENSOR_SIZE   0.03
#define BURNER_SIZE   0.03
#define SIN_PI_6      0.5
#define COS_PI_6      0.866

void drawBurner(float x1, float y1, unsigned char status)
{
	unsigned char r,g,b;
	float dx1 = 0;
	float dy1 = BURNER_SIZE;
	float dx2 = BURNER_SIZE*COS_PI_6*0.5;
	float dy2 = -BURNER_SIZE*SIN_PI_6;
	float dx3 = -dx2;BURNER_SIZE*COS_PI_6*0.5;
	float dy3 = dy2;

	if(status == BURNER_ON)
	{
		r = 255;
		g = 10;
		b = 0;
	}
	else
	{
		r = 0;
		g = 200;
		b = 0;
	}

	glBegin(GL_TRIANGLES);
	glColor3ub(r,g,b);
	glVertex2f(x1 + dx1, y1 + dy1);
	glVertex2f(x1 + dx2, y1 + dy2);
	glVertex2f(x1 + dx3, y1 + dy3);
	glEnd();
}

void drawSensor(float x1, float y1, unsigned int sensorNo)
{
	char txt[4];

	float sx1 = x1 - SENSOR_SIZE*0.5;
	float sy1 = y1 - SENSOR_SIZE;
	float sx2 = x1 + SENSOR_SIZE*0.5;
	float sy2 = y1 + SENSOR_SIZE;

	glColor3ub(0,250,250);
	glRectf(sx1,sy1,sx2,sy2);

	snprintf(txt,
		sizeof(txt),
		"%2d",
		sensorNo);

	glColor3ub(10,10,10);  
	glRasterPos2f(sx1, sy1+0.01);
	for(int i = 0; txt[i] != '\0'; i++)
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, txt[i]);

}

void rectWithBorder(float x1, 
		    float y1, 
		    float x2, 
		    float y2,
		    char* text)
{

	glRectf(x1,y1,x2,y2);

	glColor3ub(0,0,0); 
	glBegin(GL_LINES);
	glVertex2f(x1,y1);
	glVertex2f(x2,y1);
	glVertex2f(x2,y2);
	glVertex2f(x1,y2);
	glVertex2f(x1,y1);
	glEnd();

	glColor3ub(0,0,255);

	if(text != NULL)
	{
		float tx1 = x1 + (x2 -x1)/5.0;
		float ty1 = y1 - 4*(y1 -y2)/5.0;

		glRasterPos2f(tx1, ty1);
		for(int i = 0; text[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);
	}
}

static void idleGL()
{
  glutPostRedisplay();
}

static void displayGL()
{
	
	glClearColor(0.1f,0.1f,0.1f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float flx = -0.95f;
	float fty = 0.95f;
	float frx = 0.45f;
	float fby = -0.45f;
    
	float dx  = (frx-flx)/(float)(SIZEX + PAD_SIZE);
	float dy  = (fty-fby)/(float)(SIZEY + PAD_SIZE);

	//paint heat field.
	float y1  = fty;
	for(unsigned int y = 0; y < SIZEY + PAD_SIZE; ++y)
	{
		unsigned int offset = y*(SIZEX+PAD_SIZE);

		float y2 = y1 - dy;
		float x1 = flx;

		for(unsigned int x = 0; x < SIZEX + PAD_SIZE; ++x)
		{

		float         x2  = x1 + dx;
		unsigned int  rgb = clHeatPDE.pHeatImage[offset + x];
		unsigned char cr  = (unsigned char)((rgb >> 16) & (0xFF));
		unsigned char cg  = (unsigned char)((rgb >> 8) & (0xFF));
		unsigned char cb  = (unsigned char)((rgb) & (0xFF));

		glColor3ub(cr,cg,cb);
		glRectf(x1,y1,x2,y2);

		x1 = x2;
		}

		y1 = y2;
	}

	//paint sensors
	int            sensorCountX = (int)clHeatPDE.sensorCountX;
	int            sensorCountY = (int)clHeatPDE.sensorCountY;
	unsigned int*  pSensorPosX  = (unsigned int *)clHeatPDE.pSensorPosX;
	unsigned int*  pSensorPosY  = (unsigned int *)clHeatPDE.pSensorPosY;
	unsigned char* pSensorState = (unsigned char *)clHeatPDE.pSensorState;

	glColor3ub(0,250,250);
	int  sensorCount = 0;
	for(int i = 0; i < sensorCountX*sensorCountY; ++i)
	{
		if(pSensorState[i] == SENSOR_ON)
		{
		int sY    = i/sensorCountX;
		int sX    = i - sY*sensorCountX;
	  
		int sX1   = pSensorPosX[sX];
		int sY1   = pSensorPosY[sY];
	  
		float x1  = flx + sX1*dx;
		float y1  = fty - sY1*dy;
	  
		drawSensor(x1,y1,sensorCount++);
		}
	} 

	//paint burners
	int            burnerCountX = (int)clHeatPDE.burnerCountX;
	int            burnerCountY = (int)clHeatPDE.burnerCountY;
	unsigned int*  pBurnerPosX  = (unsigned int *)clHeatPDE.pBurnerPosX;
	unsigned int*  pBurnerPosY  = (unsigned int *)clHeatPDE.pBurnerPosY;
	unsigned char* pBurnerState = (unsigned char *)clHeatPDE.pBurnerState;

	glColor3ub(250,250,0);
	for(int i = 0; i < burnerCountX*burnerCountY; ++i)
	{
		int sY    = i/burnerCountX;
		int sX    = i - sY*burnerCountX;
      
		int sX1   = pBurnerPosX[sX];
		int sY1   = pBurnerPosY[sY];
		int sX2   = sX1 + BURNER_SIZE;
		int sY2   = sY1 + BURNER_SIZE;
      
		float x1  = flx + sX1*dx;
		float x2  = flx + sX2*dx;
      
		float y1  = fty - sY1*dy;
		float y2  = fty - sY2*dy;
      
		drawBurner(x1,y1, pBurnerState[i]);
	} 


	//sensor information

	//title
	float sx1 = 0.47f;
	float sx2 = 0.97f;
	float sy1 = 0.95f;
	float sy2 = sy1 - 0.10f;

	glColor3ub(250,250,250); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"SENSOR TEMPERATURE");

	//heading
	sy1 = sy2;
	sy2 = sy1 - 0.10f;
	sx2 = sx1 + 0.125;
	glColor3ub(120,120,120); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"SN");

	sx1 = sx2;
	sx2 = sx1 + 0.125;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"MIN");

	sx1 = sx2;
	sx2 = sx1 + 0.125;
	glColor3ub(120,120,120); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"MAX");

	sx1 = sx2;
	sx2 = sx1 + 0.125;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"CUR");

	float* pSensorMin  = (float *)clHeatPDE.pSensorMin;
	float* pSensorMax  = (float *)clHeatPDE.pSensorMax;
	float* pSensorData = (float *)clHeatPDE.pSensorData;

	char textSensorInfo[21];
	sensorCount = 0;
	for(int i = 0; i < sensorCountX*sensorCountY; ++i)
	{
		sx1 = 0.47f;

		if(pSensorState[i] == SENSOR_ON)
		{
		sy1 = sy2;
		sy2 = sy1 - 0.10f;
		sx2 = sx1 + 0.125;

		snprintf(textSensorInfo,
			sizeof(textSensorInfo),
			"%4d",
			sensorCount);

		glColor3ub(120,120,120); 
		rectWithBorder(sx1, sy1, sx2, sy2, textSensorInfo);


		sx1 = sx2;
		sx2 = sx1 + 0.125;
		snprintf(textSensorInfo,
			sizeof(textSensorInfo),
			"%7d",
			(int)pSensorMin[i]);

		glColor3ub(140,140,140); 
		rectWithBorder(sx1, sy1, sx2, sy2, textSensorInfo);
	  
		sx1 = sx2;
		sx2 = sx1 + 0.125;
		snprintf(textSensorInfo,
			sizeof(textSensorInfo),
			"%7d",
			(int)pSensorMax[i]);

		glColor3ub(120,120,120); 
		rectWithBorder(sx1, sy1, sx2, sy2, textSensorInfo);
	  
		sx1 = sx2;
		sx2 = sx1 + 0.125;
		snprintf(textSensorInfo,
			sizeof(textSensorInfo),
			"%7d",
			(int)pSensorData[i]);

		glColor3ub(140,140,140); 
		rectWithBorder(sx1, sy1, sx2, sy2, textSensorInfo);

		sensorCount++;
		}
	} 

	//Legends
	//title
	sx1 = 0.47f;
	sx2 = 0.97f;
	sy1 = sy2 - 0.01f;
	sy2 = sy1 - 0.10f;

	glColor3ub(250,250,250); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"LEGENDS");

	sx1 = 0.47f;
	sx2 = sx1 + 0.125;
	sy1 = sy2;
	sy2 = sy1 - 0.10f;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, NULL);
	drawBurner((sx1+sx2)/2.0,(sy1+sy2)/2.0, BURNER_ON);

	sx1 = sx2;
	sx2 = sx1+0.375;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"BURNER[ON]");

	sx1 = 0.47f;
	sx2 = sx1 + 0.125;
	sy1 = sy2;
	sy2 = sy1 - 0.10f;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, NULL);
	drawBurner((sx1+sx2)/2.0,(sy1+sy2)/2.0, BURNER_OFF);

	sx1 = sx2;
	sx2 = sx1+0.375;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"BURNER[OFF]");

	sx1 = 0.47f;
	sx2 = sx1 + 0.125;
	sy1 = sy2;
	sy2 = sy1 - 0.10f;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, NULL);
	drawSensor((sx1+sx2)/2.0,(sy1+sy2)/2.0, 0);

	sx1 = sx2;
	sx2 = sx1+0.375;
	glColor3ub(140,140,140); 
	rectWithBorder(sx1, sy1, sx2, sy2, (char*)"SENSOR");

	glutSwapBuffers();
}

static void keyboardGL(unsigned char key, int x, int y)
{
  int done = 0;
  switch(key)
	{
		case 'r':
		case 'R':
		{
			break;
		}
		case 27:
		case 'q':
		case 'Q':
		{ 

			ready = true;
			cv_gui.notify_one();
			
			while(1)
			{
				if(feedback_stop && clExex_stop)
				{
					if(clHeatPDE.cleanup() != SDK_SUCCESS)
					{
						exit(1);
					}
					else
					{
						exit(0);
					}
				}
			}
		}
		default:
			break;
	}
}

#endif
