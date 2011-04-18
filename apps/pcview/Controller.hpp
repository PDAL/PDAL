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

#ifndef INCLUDED_PCVIEW_CONTROLLER_H
#define INCLUDED_PCVIEW_CONTROLLER_H

#include <deque>

#include <boost/cstdint.hpp>

class Command;


class Controller
{
public:
    Controller();
    ~Controller();

    void reset();

    void setWindowPosition(int x, int y);
    void setWindowSize(int w, int h);
    int getWindowPositionX() const;
    int getWindowPositionY() const;
    int getWindowSizeW() const;
    int getWindowSizeH() const;

    double getDefaultCameraX() const;
    double getDefaultCameraY() const;
    double getDefaultCameraZ() const;
    double getCameraX() const;
    double getCameraY() const;
    double getCameraZ() const;
    void setCamera(double x, double y, double z);

    int getNumPoints() const;
    const float* getPoints() const;
    const boost::uint16_t* getColors() const;
    void setPoints(const float* points, const boost::uint16_t* colors, int numPoints);

    void setBounds(float minx, float miny, float minz, float maxx, float maxy, float maxz);
    void getBounds(float& minx, float& miny, float& minz, float& maxx, float& maxy, float& maxz, float& delx, float& dely, float& delz);
   
    // deque takes ownership
    void pushCommand(Command*);

    // caller takes ownership
    Command* popCommand();

private:
    std::deque<Command*> m_commands;

    int m_windowSizeW;
    int m_windowSizeH;
    int m_windowPositionX;
    int m_windowPositionY;

    static const double m_defaultCameraX;
    static const double m_defaultCameraY;
    static const double m_defaultCameraZ;
    double m_cameraX;
    double m_cameraY;
    double m_cameraZ;

    int m_numPoints;
    const float* m_points;
    const boost::uint16_t* m_colors;

    float m_minx, m_miny, m_minz, m_maxx, m_maxy, m_maxz;

    bool m_exitRequested;
    bool m_toggleFullScreenRequested;
    bool m_resetRequested;

    Controller(const Controller&); // nope
    Controller& operator=(const Controller&); // nope
};

#endif
