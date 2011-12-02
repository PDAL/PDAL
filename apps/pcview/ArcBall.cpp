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

#include <pdal/pdal_internal.hpp>

#ifdef PDAL_PLATFORM_WIN32
#include <windows.h>
#endif

#include <gl/gl.h>
#include <gl/glu.h>

#include <cmath>
#include <cassert>

#include "ArcBall.hpp"


//-----------------------------------------------------------------------------
// ArcBallControl
//-----------------------------------------------------------------------------


ArcBall::ArcBall(GLfloat NewWidth, GLfloat NewHeight)
{
    //Clear initial values
    this->StVec.s.X     =
    this->StVec.s.Y     = 
    this->StVec.s.Z     = 

    this->EnVec.s.X     =
    this->EnVec.s.Y     = 
    this->EnVec.s.Z     = 0.0f;

    //Set initial bounds
    this->setBounds(NewWidth, NewHeight);
}


void ArcBall::setBounds(GLfloat NewWidth, GLfloat NewHeight)
{
    assert((NewWidth > 1.0f) && (NewHeight > 1.0f));

    //Set adjustment factor for width/height
    this->AdjustWidth  = 1.0f / ((NewWidth  - 1.0f) * 0.5f);
    this->AdjustHeight = 1.0f / ((NewHeight - 1.0f) * 0.5f);
}


//Arcball sphere constants:
//Diameter is       2.0f
//Radius is         1.0f
//Radius squared is 1.0f

void ArcBall::_mapToSphere(const Point2fT* NewPt, Vector3fT* NewVec) const
{
    Point2fT TempPt;
    GLfloat length;

    //Copy paramter into temp point
    TempPt = *NewPt;

    //Adjust point coords and scale down to range of [-1 ... 1]
    TempPt.s.X  =        (TempPt.s.X * this->AdjustWidth)  - 1.0f;
    TempPt.s.Y  = 1.0f - (TempPt.s.Y * this->AdjustHeight);

    //Compute the square of the length of the vector to the point from the center
    length      = (TempPt.s.X * TempPt.s.X) + (TempPt.s.Y * TempPt.s.Y);

    //If the point is mapped outside of the sphere... (length > radius squared)
    if (length > 1.0f)
    {
        GLfloat norm;

        //Compute a normalizing factor (radius / sqrt(length))
        norm    = 1.0f / sqrtf(length);

        //Return the "normalized" vector, a point on the sphere
        NewVec->s.X = TempPt.s.X * norm;
        NewVec->s.Y = TempPt.s.Y * norm;
        NewVec->s.Z = 0.0f;
    }
    else    //Else it's on the inside
    {
        //Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
        NewVec->s.X = TempPt.s.X;
        NewVec->s.Y = TempPt.s.Y;
        NewVec->s.Z = sqrtf(1.0f - length);
    }
}


void ArcBall::click(const Point2fT* NewPt)
{
    //Map the point to the sphere
    this->_mapToSphere(NewPt, &this->StVec);
}


void ArcBall::drag(const Point2fT* NewPt, Quat4fT* NewRot)
{
    //Map the point to the sphere
    this->_mapToSphere(NewPt, &this->EnVec);

    //Return the quaternion equivalent to the rotation
    if (NewRot)
    {
        Vector3fT  Perp;

        //Compute the vector perpendicular to the begin and end vectors
        Vector3fCross(&Perp, &this->StVec, &this->EnVec);

        //Compute the length of the perpendicular vector
        if (Vector3fLength(&Perp) > Epsilon)    //if its non-zero
        {
            //We're ok, so return the perpendicular vector as the transform after all
            NewRot->s.X = Perp.s.X;
            NewRot->s.Y = Perp.s.Y;
            NewRot->s.Z = Perp.s.Z;
            //In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
            NewRot->s.W= Vector3fDot(&this->StVec, &this->EnVec);
        }
        else                                    //if its zero
        {
            //The begin and end vectors coincide, so return an identity transform
            NewRot->s.X = 
            NewRot->s.Y = 
            NewRot->s.Z = 
            NewRot->s.W = 0.0f;
        }
    }
}


//-----------------------------------------------------------------------------
// ArcBallControl
//-----------------------------------------------------------------------------


ArcBallControl::ArcBallControl(int width, int height)
    : ArcBall((GLfloat)width, (GLfloat)height)
    , m_isClicked(false)
    , m_isDragging(false)
{
    reset();
    return;
}


ArcBallControl::~ArcBallControl()
{
    return;
}

void ArcBallControl::reset()
{
    ArcBall::Matrix3fSetIdentity(&m_lastRot);
    ArcBall::Matrix3fSetIdentity(&m_thisRot);
    ArcBall::Matrix4fSetIdentity(&m_transform);

    ArcBall::Matrix4fSetRotationFromMatrix3f(&m_transform, &m_thisRot);
}


// returns true if an update is needed
bool ArcBallControl::update(int x, int y)
{
    if (!m_isDragging)
    {
        return false;
    }

    if (!m_isClicked)
    {
        m_isDragging = false;
        return false;
    }

    // still clicked, still dragging
    m_mousePt.s.X = (GLfloat)x;
    m_mousePt.s.Y = (GLfloat)y;

    dragAction();

    return true;
}


void ArcBallControl::dragAction()
{
    ArcBall::Quat4fT thisQuat;

    this->drag(&m_mousePt, &thisQuat);					// Update End Vector And Get Rotation As Quaternion
    ArcBall::Matrix3fSetRotationFromQuat4f(&m_thisRot, &thisQuat);			// Convert Quaternion Into Matrix3fT
    ArcBall::Matrix3fMulMatrix3f(&m_thisRot, &m_lastRot);				// Accumulate Last Rotation Into This One
    ArcBall::Matrix4fSetRotationFromMatrix3f(&m_transform, &m_thisRot);			// Set Our Final Transform's Rotation From This One

    return;
}


bool ArcBallControl::changeState(bool isClicked, int mx, int my)
{
    m_isClicked = isClicked;

    m_mousePt.s.X = (GLfloat)mx;
    m_mousePt.s.Y = (GLfloat)my;

    if (!m_isDragging)
    {
        if (!m_isClicked)
        {
            return false;
        }

        // first click
        m_isDragging = true;							// Prepare For Dragging
        m_lastRot = m_thisRot;							// Set Last Static Rotation To Last Dynamic One
        this->click(&m_mousePt);						// Update Start Vector And Prepare For Dragging
        return false;
    }

    // we are dragging
    if (!m_isClicked)
    {
        // no longer dragging
        m_isDragging = false;
        return false;
    }

    dragAction();

    return true;
}

const GLfloat* ArcBallControl::getTransform() const
{
    return m_transform.M;
}
