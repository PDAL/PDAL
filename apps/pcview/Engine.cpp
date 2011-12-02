/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/pdal.hpp>

#ifdef PDAL_PLATFORM_WIN32
#include <windows.h>
#endif

#include <iostream>
#include <cstdlib>
#include <cassert>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4505)  // unreferenced local function has been removed
#endif
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h> // for mouse wheel support (might not work under X Windows)

#include "glext.h"

#include "Engine.hpp"
#include "Controller.hpp"
#include "Commands.hpp"

#include "ArcBall.hpp"

static void displayFunc();
static void reshapeFunc(int w, int h);
static void keyboardFunc(unsigned char key, int x, int y);
static void mouseFunc(int button, int state, int x, int y);
static void motionFunc(int x, int y);
static void idleFunc();
static void mouseWheelFunc(int wheel, int direction, int x, int y);


#include "Engine.hpp"
#include "Commands.hpp"


Engine::Engine(Controller& controller)
    : m_controller(controller)
    , m_windowId(-1)
    , m_arcBall(NULL)
    , m_isFullScreen(false)
    , m_fullScreenSavedX(0)
    , m_fullScreenSavedY(0)
    , m_fullScreenSavedW(0)
    , m_fullScreenSavedH(0)
    , m_scale(1.0)
{
    return;
}

Engine::~Engine()
{
    delete m_arcBall;
    return;
}


void Engine::initialize(int argc, char** argv)
{
    const Controller& controller = getController();

    // argc/argv not used now, but might be needed for X displays later
    glutInit(&argc,argv);

    // BUG: double-buffer?
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_SINGLE);

    glutInitWindowSize(controller.getWindowSizeW(), controller.getWindowSizeH());
    glutInitWindowPosition(controller.getWindowPositionX(), controller.getWindowPositionY());

    glutInitContextVersion(3,0);
    //glutInitContextFlags(GLUT_FORWARD_COMPATIBLE); // BUG: needed?

    m_windowId = glutCreateWindow("pcview");

    reset();

    glutSetWindowData(this);

    glutDisplayFunc(displayFunc);

    glutReshapeFunc(reshapeFunc);
    glutKeyboardFunc(keyboardFunc);
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);
    glutIdleFunc(idleFunc);
    glutMouseWheelFunc(mouseWheelFunc);

    glClearColor(0.0,0.0,0.0,0.0);

    glutMainLoop();
}


void Engine::reset()
{
    Controller& controller = getController();

    controller.reset();

    delete m_arcBall;
    m_arcBall = new ArcBallControl(controller.getWindowSizeW(), controller.getWindowSizeH());

    m_scale = 1.0;

    return;
}


void Engine::commandExit()
{
    glutLeaveMainLoop();
    return;
}


void Engine::commandToggleFullScreen()
{
    if (m_isFullScreen)
    {
        glutPositionWindow(m_fullScreenSavedX, m_fullScreenSavedY);
        glutReshapeWindow(m_fullScreenSavedW, m_fullScreenSavedH);
    }
    else
    {
        m_fullScreenSavedX = glutGet(GLUT_WINDOW_X);
        m_fullScreenSavedY = glutGet(GLUT_WINDOW_Y);
        m_fullScreenSavedW = glutGet(GLUT_WINDOW_WIDTH);
        m_fullScreenSavedH = glutGet(GLUT_WINDOW_HEIGHT);
        glutFullScreen();
    }

    m_isFullScreen = !m_isFullScreen;

    return;
}


void Engine::commandReset()
{
    reset();

    printf("reset()\n");

    return;
}


void Engine::commandZoom(const CommandZoom& command)
{
    double d = command.getDelta();

    double delta = (d > 0.0) ? d * 1.5 : 1.0/(-d * 1.5);

    m_scale *= delta;

    return;
}


const Controller& Engine::getController() const
{
    return m_controller;
}


Controller& Engine::getController()
{
    return m_controller;
}


void Engine::doKeyboard(unsigned char key, int /*x*/, int /*y*/)
{
    Controller& controller = getController();

    switch (key)
    {
    case 'h':
        controller.pushCommand(new CommandReset());
        break;

    case 'f':
        controller.pushCommand(new CommandFullScreen());
        break;

    case '+':
    case '=':
        controller.pushCommand(new CommandZoom(1.0));
        break;

    case '-':
    case '_':
        controller.pushCommand(new CommandZoom(-1.0));
        break;

    case 27: // ESC key
    case 'q':
        controller.pushCommand(new CommandExit());
        break;

    default:
        assert(false);
    }

    return;
}


void Engine::doMotion(int x, int y)
{
    bool changed = m_arcBall->update(x,y);
    if (changed)
    {
        glutPostRedisplay();
    }

    return;
}


void Engine::doIdle()
{
    // ask the controller if we have any work to do
    Controller& controller = getController();
    
    // we just do one command at a time then return, so as not to block too much
    Command* command = controller.popCommand();
    if (!command)
    {
        // mothing to do
        return;
    }

    switch (command->getType())
    {
    case Command::EXIT:
        this->commandExit();
        break;
    case Command::FULLSCREEN:
        this->commandToggleFullScreen();
        break;
    case Command::RESET:
        this->commandReset();
        break;
    case Command::ZOOM:
        this->commandZoom(*static_cast<CommandZoom*>(command));
        break;
    default:
        assert(0);
    }

    delete command;

    glutPostRedisplay();

    return;
}


void Engine::doMouseWheel(int wheel, int direction, int /*x*/, int /*y*/)
{
    if (wheel!=0) return;

    Controller& controller = getController();

    double delta = (direction > 0) ? 1.0 : -1.0;

    controller.pushCommand(new CommandZoom(delta));

    return;
}


void Engine::doReshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    gluPerspective(60, (double)w/(double)h, 0.01, 10000);

    m_arcBall->setBounds((GLfloat)w, (GLfloat)h);

    return;
}

bool s_drawCube = true;

void Engine::doDisplay()
{
    Controller& controller = getController();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    // lookAt is expressed in view space
    const double cameraX = controller.getCameraX();
    const double cameraY = controller.getCameraY();
    const double cameraZ = controller.getCameraZ();
    gluLookAt(cameraX, cameraY, cameraZ, 0, 0, 0, 0,1,0);

    // handle zooming
    glScaled(m_scale, m_scale, m_scale);

    // handle arc rotation
    glMultMatrixf(m_arcBall->getTransform());

    float zAdjust = 1.0f;

    // scale and translate scale from model coords to view coords
    float minx, miny, minz, maxx, maxy, maxz, delx, dely, delz;
    controller.getBounds(minx, miny, minz, maxx, maxy, maxz, delx, dely, delz);

    float rangeMax = delx;
    if (dely > rangeMax) rangeMax = dely;
    if (delz > rangeMax) rangeMax = delz;
    glScaled(1.0/rangeMax, 1.0/rangeMax, zAdjust * 1.0/rangeMax);

    glTranslated(-minx-delx/2.0,-miny-dely/2.0,(-minz-delx/2.0)/zAdjust);

    if (s_drawCube)
    {
    // draw cube
    glBegin(GL_LINES);
    
    glColor3f(1,0,0);
    // X, front plane
    glVertex3f(minx,miny,minz);
    glVertex3f(maxx,miny,minz);
    glVertex3f(minx,maxy,minz);
    glVertex3f(maxx,maxy,minz);

    // X, back plane
    glVertex3f(minx,miny,maxz);
    glVertex3f(maxx,miny,maxz);
    glVertex3f(minx,maxy,maxz);
    glVertex3f(maxx,maxy,maxz);

    glColor3f(0,1,0);
    // Y, front plane
    glVertex3f(minx,miny,minz);
    glVertex3f(minx,maxy,minz);
    glVertex3f(maxx,miny,minz);
    glVertex3f(maxx,maxy,minz);
    // Y, back plane
    glVertex3f(minx,miny,maxz);
    glVertex3f(minx,maxy,maxz);
    glVertex3f(maxx,miny,maxz);
    glVertex3f(maxx,maxy,maxz);

    glColor3f(0,0,1);
    // Z, bottom plane
    glVertex3f(minx,miny,minz);
    glVertex3f(minx,miny,maxz);
    glVertex3f(maxx,miny,minz);
    glVertex3f(maxx,miny,maxz);
    // Z, top plane
    glVertex3f(minx,maxy,minz);
    glVertex3f(minx,maxy,maxz);
    glVertex3f(maxx,maxy,minz);
    glVertex3f(maxx,maxy,maxz);

    glEnd();

#if 0
    // put indicator at orgin
    glBegin(GL_LINE_STRIP);
    glColor3f(1,1,1);
    glVertex3f(minx+0.2f*delx, miny, minz);
    glVertex3f(minx, miny+0.2f*dely, minz);
    glVertex3f(minx, miny, minz+0.2f*delz);
    glVertex3f(minx+0.2f*delx, miny, minz);
    glEnd();
#endif
    }

    for (size_t v=0; v<controller.getDataVector().size(); v++)
    {
        const float* points = controller.getDataVector()[v].points;
        const boost::uint16_t* colors = controller.getDataVector()[v].colors;
        const int numPoints = controller.getDataVector()[v].numPoints;
        
#if 0
        // draw points
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        for (int i=0; i<numPoints*3; i+=3)
        {
            float x = points[i];
            float y = points[i+1];
            float z = points[i+2];

            if (colors != NULL)
            {
                boost::uint16_t r = colors[i];
                boost::uint16_t g = colors[i+1];
                boost::uint16_t b = colors[i+2];

                glColor3us(r, g, b);
            }

            glVertex3f(x, y, z);
        }
        glEnd();
#else
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, points);

        if (colors)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_UNSIGNED_SHORT, 0, colors);
        }
        else
        {
            glColor3f(1,1,1);
        }

        glBegin(GL_POINTS);
        {
            if (!colors)
            {
                glColor3f(1,1,1);
            }
            for (int i=0; i<numPoints; i++)
            {
                glArrayElement(i);
            }
        }
        glEnd();
        //glDrawElements(GL_POINTS, numPoints, GL_FLOAT, points);
#endif
    }

    glFlush();

    return;
}


void Engine::doMouse(int button, int state, int mx, int my)
{
    if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON)
    {
        GLint viewport[4];
        GLdouble mvmatrix[16], projmatrix[16];
        GLint realy;
        GLdouble wx0, wy0, wz0; // returned
        GLdouble wx1, wy1, wz1; // returned

        glGetIntegerv(GL_VIEWPORT, viewport);
        glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
        glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

        realy = viewport[3] - (GLint)my - 1;
        printf("\n");
        //printf("Cursor = (%d, %d)\n", mx, realy);
        
        int ok = gluUnProject((GLdouble)mx, (GLdouble)realy, 0.0, mvmatrix, projmatrix, viewport, &wx0, &wy0, &wz0);
        if (ok != GL_TRUE)
        {
            printf("** FAIL **\n");
        }
        //printf("World @ z0 = (%.1f, %.1f, %.1f)\n", wx0, wy0, wz0);

        ok = gluUnProject((GLdouble)mx, (GLdouble)realy, 1.0, mvmatrix, projmatrix, viewport, &wx1, &wy1, &wz1);
        if (ok != GL_TRUE)
        {
            printf("** FAIL **\n");
        }
        //printf("World @ z1 = (%.1f, %.1f, %.1f)\n", wx1, wy1, wz1);

        double c0 = (10-wz0) / (wz1 - wz0);
        double x0 = wx0 + c0 * (wx1 - wx0);
        double y0 = wy0 + c0 * (wy1 - wy0);
        printf("at zmax, (x,y)=(%.0f,%.0f)\n", x0, y0);

        double c1 = (15 - wz0) / (wz1 - wz0);
        double x1 = wx0 + c1 * (wx1 - wx0);
        double y1 = wy0 + c1 * (wy1 - wy0);
        printf("at zmin, (x,y)=(%.0f,%.0f)\n", x1, y1);
    }

    if (button == GLUT_LEFT_BUTTON)
    {
        const bool isClicked = (state == GLUT_DOWN);
        bool changed = m_arcBall->changeState(isClicked, mx, my);
        if (changed)
        {
            glutPostRedisplay();
        }
    }

    return;
}


// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------


static void keyboardFunc(unsigned char key, int x, int y)
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doKeyboard(key, x, y);
}


static void motionFunc(int x, int y)
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doMotion(x, y);
}


static void idleFunc()
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doIdle();
}


static void mouseWheelFunc(int wheel, int direction, int x, int y)
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doMouseWheel(wheel, direction, x, y);
}


static void reshapeFunc(int w, int h)
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doReshape(w, h);
}


void displayFunc()
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doDisplay();
}


static void mouseFunc(int button, int state, int x, int y)
{
    Engine* engine = (Engine*)glutGetWindowData();
    engine->doMouse(button, state, x, y);
}
