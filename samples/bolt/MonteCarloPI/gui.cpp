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

#include <cmath>
#include "GL/glut.h"
#include <iostream>
#if (defined(WIN32) && defined(WIN64))
#include <Windows.h>
#endif
#include "MonteCarloPI.hpp"
#include "bolt/cl/count.h"
#include "bolt/cl/transform_reduce.h"
#include "gui.hpp"

// Use safe version of std functions on windows
#if defined(_MSC_VER)
#define sprintf sprintf_s
#endif

/*
 * Maintain a global-pointer to MonteCarloPIGUI object.
 * Since glut takes function handlers withour any arguments,
 * we maintain a global pointer to the object to access its members
 */
MonteCarloPIGUI *GUIObj;

void draw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0f,0.0f,-3.0f);

    // Unit square
    glBegin(GL_QUADS);
    glColor3f(255.0f/255.0f,242.0f/255.0f,0.0f/255.0f);     // Yellow
    glVertex3f(-1.0f, 1.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, 0.0f);
    glVertex3f(1.0f, -1.0f, 0.0f);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glEnd();

    // Unit circle
    glBegin(GL_POLYGON);
    glColor3f(75.0f/255.0f,75.0f/255.0f,200.0f/255.0f);     // Blue
#define CIRCLE_SEGMENTS    400
    float radianIncrement = 3.1416f * 2/CIRCLE_SEGMENTS;
    float radians = 0.0f;
    for (int i = 0; i < CIRCLE_SEGMENTS; i++, radians+=radianIncrement)
    {
        glVertex3f(cos(radians), sin(radians), 0.0f);
    }
    glEnd();

    // Lines
    glColor3f(0.0f, 0.0f, 0.0f);    // Black
    glBegin(GL_LINES);
    glVertex3f(-1.2f, 0.0f, 0.0f);
    glVertex3f(1.2f, 0.0f, 0.0f);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(0.0f, 1.2f, 0.0f);
    glVertex3f(0.0f, -1.2f, 0.0f);
    glEnd();

    // Draw all the random points
    glColor3f(0.0f, 0.0f, 0.0f);    // Black
    glBegin(GL_POINTS);
    for (int i=0; i<(GUIObj->numPoints); i++)
    {
        glVertex3f((GUIObj->inputPoints[i].x), (GUIObj->inputPoints[i].y), 0.0f);
    }
    glEnd();

    char str[128];
    sprintf(str, "PI Value: %1.10f", GUIObj->PIValue);
    glRasterPos3f(0.1f, 1.05f, 0.1f);
    for (char *c=str; *c != '\0'; c++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }

    sprintf(str, "Points: %-10d", GUIObj->numPoints);
    glRasterPos3f(-1.1f, 1.05f, 0.1f);
    for (char *c=str; *c != '\0'; c++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }

    glutSwapBuffers();
}

void keyboard(unsigned char key, int mouseX, int mouseY)
{
    int newSize;

    switch ( key )
    {
    case 'q':
    case 27 :               // On Esc key
        exit(0);
        break;

    case '+':
    case 'a':
        newSize = (int)(GUIObj->numPoints * 1.10f);
        GUIObj->numPoints = newSize;
        GUIObj->update();
        glutPostRedisplay();
        break;

    case '-':
    case 'z':
        newSize = (int)(GUIObj->numPoints * 0.90f);
        newSize = (newSize < 1000)? 1000: newSize;
        GUIObj->numPoints = newSize;
        GUIObj->update();
        glutPostRedisplay();
        break;

    case 'h':
        GUIObj->usage();
        break;

    default:
        break;
    }
}

int MonteCarloPIGUI::usage()
{
    std::cout << "Press '+' or 'a' to increase the number of points." << std::endl;
    std::cout << "Press '-' or 'z' to decrease the number of points." << std::endl;
    std::cout << "Press 'q' to quit." << std::endl;
    std::cout << "Press 'h' to show this message." << std::endl;

    return 0;
}

int MonteCarloPIGUI::update()
{
    inputPoints.resize(numPoints);
    boltInputPoints.resize(numPoints);

    for(int i=0; i<numPoints; i++)
    {
        float x = (float)((double) std::rand() * (2.0 * RADIUS / RAND_MAX) - RADIUS);
        float y = (float)((double) std::rand() * (2.0 * RADIUS / RAND_MAX) - RADIUS);
        inputPoints[i] = Point(x, y);
    }

    bolt::cl::device_vector<Point>::pointer inputPtr = boltInputPoints.data();
    Point *mappedToCpu = inputPtr.get();

    ::memcpy(mappedToCpu, &inputPoints[0], sizeof(Point) * numPoints);
    inputPtr.reset();

    int pointsInCircle = bolt::cl::transform_reduce(boltInputPoints.begin(),
                         boltInputPoints.end(), isInsideCircleFunctor(RADIUS),
                         0, bolt::cl::plus<int>());

    PIValue = (4.0f * pointsInCircle) / numPoints;
    return 0;
}

int MonteCarloPIGUI::showGUI(int argc, char **argv)
{
    GUIObj = this;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);

    const int width = 480;
    const int height = 480;
    const char* title = "Monte Carlo PI Estimation";

    glutInitWindowSize(width, height);
    glutCreateWindow(title);

    glMatrixMode(GL_PROJECTION);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat aspect = (GLfloat) width/height;

    const float fieldOfViewAngle = 45;
    const float zNear = 1.0f;
    const float zFar = 500.0f;
    gluPerspective(fieldOfViewAngle, aspect, zNear, zFar);
    glMatrixMode(GL_MODELVIEW);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearColor(1.0, 1.0, 1.0, 1.0);

    // Register the callbacks
    glutDisplayFunc(draw);
    glutKeyboardFunc(keyboard);

    usage();
    update();
    glutMainLoop();

    return 0;
}
