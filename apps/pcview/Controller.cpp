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

#include <stdlib.h>
#include "Controller.hpp"

const double Controller::m_defaultCameraX = 0;
const double Controller::m_defaultCameraY = 0;
const double Controller::m_defaultCameraZ = 2.0;


Controller::Controller()
    : m_windowSizeW(256)
    , m_windowSizeH(256)
    , m_windowPositionX(0)
    , m_windowPositionY(0)
    , m_cameraX(m_defaultCameraX)
    , m_cameraY(m_defaultCameraY)
    , m_cameraZ(m_defaultCameraZ)
    , m_minx(0.0f)
    , m_miny(0.0f)
    , m_minz(0.0f)
    , m_maxx(0.0f)
    , m_maxy(0.0f)
    , m_maxz(0.0f)
{
    return;
}

Controller::~Controller()
{
    return;
}


void Controller::reset()
{
    m_cameraX = m_defaultCameraX;
    m_cameraY = m_defaultCameraY;
    m_cameraZ = m_defaultCameraZ;

    return;
}


void Controller::pushCommand(Command* command)
{
   m_commands.push_back(command);
}


Command* Controller::popCommand()
{
    if (m_commands.size() == 0)
        return NULL;

    Command* command = m_commands.front();
    m_commands.pop_front();
    return command;
}


void Controller::setWindowPosition(int x, int y)
{
    m_windowPositionX = x;
    m_windowPositionY = y;
}


int Controller::getWindowPositionX() const
{
    return m_windowPositionX;
}


int Controller::getWindowPositionY() const
{
    return m_windowPositionY;
}


void Controller::setWindowSize(int w, int h)
{
    m_windowSizeW = w;
    m_windowSizeH = h;
}


int Controller::getWindowSizeW() const
{
    return m_windowSizeW;
}


int Controller::getWindowSizeH() const
{
    return m_windowSizeH;
}


double Controller::getDefaultCameraX() const
{
    return m_defaultCameraX;
}


double Controller::getDefaultCameraY() const
{
    return m_defaultCameraY;
}


double Controller::getDefaultCameraZ() const
{
    return m_defaultCameraZ;
}


double Controller::getCameraX() const
{
    return m_cameraX;
}


double Controller::getCameraY() const
{
    return m_cameraY;
}


double Controller::getCameraZ() const
{
    return m_cameraZ;
}


void Controller::setCamera(double x, double y, double z)
{
    m_cameraX = x;
    m_cameraY = y;
    m_cameraZ = z;
}


std::vector<Controller::Data> Controller::getDataVector() const
{
    return m_dataVector;
}


void Controller::addPoints(float* points, boost::uint16_t* colors, int numPoints)
{
    Data data;
    data.points = points;
    data.colors = colors;
    data.numPoints = numPoints;
    m_dataVector.push_back(data);
}


void Controller::setBounds(float minx, float miny, float minz, float maxx, float maxy, float maxz)
{
    m_minx = minx;
    m_miny = miny;
    m_minz = minz;
    m_maxx = maxx;
    m_maxy = maxy;
    m_maxz = maxz;
}


void Controller::getBounds(float& minx, float& miny, float& minz, float& maxx, float& maxy, float& maxz, float& delx, float& dely, float& delz)
{
    minx = m_minx;
    miny = m_miny;
    minz = m_minz;
    maxx = m_maxx;
    maxy = m_maxy;
    maxz = m_maxz;
    delx = m_maxx - m_minx;
    dely = m_maxy - m_miny;
    delz = m_maxz - m_minz;
}

