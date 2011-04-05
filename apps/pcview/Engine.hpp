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

#ifndef INCLUDED_PCVIEW_ENGINE_H
#define INCLUDED_PCVIEW_ENGINE_H

class Controller;
class ArcBallControl;

class CommandZoom;

class Engine
{
public:
    Engine(Controller&);
    ~Engine();

    void initialize(int argc, char** argv);

    // user interfaces may call these at will
    void commandExit();
    void commandToggleFullScreen();
    void commandReset();
    void commandZoom(const CommandZoom& command);

    // do not call these directly; used only by the callbacks passed to glut
    void doDisplay();
    void doReshape(int w, int h);
    void doKeyboard(unsigned char key, int x, int y);
    void doMouse(int button, int state, int x, int y);
    void doMotion(int x, int y);
    void doIdle();
    void doMouseWheel(int wheel, int direction, int x, int y);

private:
    const Controller& getController() const;
    Controller& getController();
    void reset();

    Controller& m_controller;
    int m_windowId;
    ArcBallControl* m_arcBall;

    bool m_isFullScreen;
    int m_fullScreenSavedX;
    int m_fullScreenSavedY;
    int m_fullScreenSavedW;
    int m_fullScreenSavedH;

    double m_scale;

    Engine(const Engine&); // nope
    Engine& operator=(const Engine&); // nope
};

#endif
