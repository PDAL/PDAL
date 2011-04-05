//
// This code is derived from the well-known, much-copied Arcball class:
//      (C) 1999-2003 Tatewake.com
// This code is provided under the license terms from the http://nehe.gamedev.net/ 
// tutorials/basecode series, which (depending on where you look) is either CC-SA or 
// the following:
//
//     If you plan to put this program on your web page or a cdrom of
//     any sort, let me know via email, I'm curious to see where
//     it ends up :)
//     If you use the code for your own projects please give me credit,
//     or mention my web site somewhere in your program or it's docs. 
//
// http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=48
//

#ifndef INCLUDED_PCVIEW_ARCBALL_H
#define INCLUDED_PCVIEW_ARCBALL_H

#include "ArcBallMath.hpp"

class ArcBall : public ArcBallMath
{
public:
    ArcBall(GLfloat NewWidth, GLfloat NewHeight);
    virtual ~ArcBall() { };

    void setBounds(GLfloat NewWidth, GLfloat NewHeight);

    // mouse down
    void click(const Point2fT* NewPt);

    // mouse drag, calculate rotation
    void drag(const Point2fT* NewPt, Quat4fT* NewRot);

protected:
    void _mapToSphere(const Point2fT* NewPt, Vector3fT* NewVec) const;

    Vector3fT   StVec;          // saved click vector
    Vector3fT   EnVec;          // saved drag vector
    GLfloat     AdjustWidth;    // mouse bounds width
    GLfloat     AdjustHeight;   // mouse bounds height
};


// this class encapsulates all the state-management code thatm using regular ArcBall,
// has to live in the client app
class ArcBallControl : public ArcBall
{
public:
    ArcBallControl(int width, int height);
    virtual ~ArcBallControl();

    void reset();
    bool update(int x, int y);
    bool changeState(bool isClicked, int x, int y);

    const GLfloat* getTransform() const;

private:
    void dragAction();

    ArcBall::Matrix4fT m_transform;
    ArcBall::Matrix3fT m_lastRot;
    ArcBall::Matrix3fT m_thisRot;
    ArcBall::Point2fT m_mousePt;
    bool m_isClicked;
    bool m_isDragging;
};


#endif
