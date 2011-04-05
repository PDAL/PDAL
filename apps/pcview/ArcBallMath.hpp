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

#ifndef INCLUDED_PCVIEW_ARCBALLMATH_H
#define INCLUDED_PCVIEW_ARCBALLMATH_H

#include <Windows.h>

#include <gl/gl.h>												// Header File For The OpenGL32 Library
#include <gl/glu.h>												// Header File For The GLu32 Library

class ArcBallMath
{
public:

    typedef union Tuple2f_t
    {
        struct
        {
            GLfloat X, Y;
        } s;

        GLfloat T[2];
    } Tuple2fT;      //A generic 2-element tuple that is represented by single-precision floating point x,y coordinates. 

    typedef union Tuple3f_t
    {
        struct
        {
            GLfloat X, Y, Z;
        } s;

        GLfloat T[3];
    } Tuple3fT;      //A generic 3-element tuple that is represented by single precision-floating point x,y,z coordinates. 

    typedef union Tuple4f_t
    {
        struct
        {
            GLfloat X, Y, Z, W;
        } s;

        GLfloat T[4];
    } Tuple4fT;      //A 4-element tuple represented by single-precision floating point x,y,z,w coordinates. 

    typedef union Matrix3f_t
    {
            struct
            {
                //column major
                union { GLfloat M00; GLfloat XX; GLfloat SX; };  //XAxis.X and Scale X
                union { GLfloat M10; GLfloat XY;             };  //XAxis.Y
                union { GLfloat M20; GLfloat XZ;             };  //XAxis.Z
                union { GLfloat M01; GLfloat YX;             };  //YAxis.X
                union { GLfloat M11; GLfloat YY; GLfloat SY; };  //YAxis.Y and Scale Y
                union { GLfloat M21; GLfloat YZ;             };  //YAxis.Z
                union { GLfloat M02; GLfloat ZX;             };  //ZAxis.X
                union { GLfloat M12; GLfloat ZY;             };  //ZAxis.Y
                union { GLfloat M22; GLfloat ZZ; GLfloat SZ; };  //ZAxis.Z and Scale Z
            } s;
            GLfloat M[9];
    } Matrix3fT;     //A single precision floating point 3 by 3 matrix. 

    typedef union Matrix4f_t
    {
            struct
            {
                //column major
                union { GLfloat M00; GLfloat XX; GLfloat SX; };  //XAxis.X and Scale X
                union { GLfloat M10; GLfloat XY;             };  //XAxis.Y
                union { GLfloat M20; GLfloat XZ;             };  //XAxis.Z
                union { GLfloat M30; GLfloat XW;             };  //XAxis.W
                union { GLfloat M01; GLfloat YX;             };  //YAxis.X
                union { GLfloat M11; GLfloat YY; GLfloat SY; };  //YAxis.Y and Scale Y
                union { GLfloat M21; GLfloat YZ;             };  //YAxis.Z
                union { GLfloat M31; GLfloat YW;             };  //YAxis.W
                union { GLfloat M02; GLfloat ZX;             };  //ZAxis.X
                union { GLfloat M12; GLfloat ZY;             };  //ZAxis.Y
                union { GLfloat M22; GLfloat ZZ; GLfloat SZ; };  //ZAxis.Z and Scale Z
                union { GLfloat M32; GLfloat ZW;             };  //ZAxis.W
                union { GLfloat M03; GLfloat TX;             };  //Trans.X
                union { GLfloat M13; GLfloat TY;             };  //Trans.Y
                union { GLfloat M23; GLfloat TZ;             };  //Trans.Z
                union { GLfloat M33; GLfloat TW; GLfloat SW; };  //Trans.W and Scale W
            } s;
            GLfloat M[16];
    } Matrix4fT;     //A single precision floating point 4 by 4 matrix. 

    typedef Tuple2fT Point2fT;   //A 2 element point that is represented by single precision floating point x,y coordinates. 
    typedef Tuple4fT Quat4fT;   //A 4 element unit quaternion represented by single precision floating point x,y,z,w coordinates. 
    typedef Tuple2fT Vector2fT;   //A 2-element vector that is represented by single-precision floating point x,y coordinates. 
    typedef Tuple3fT Vector3fT;   //A 3-element vector that is represented by single-precision floating point x,y,z coordinates. 

    //assuming IEEE-754(GLfloat), which i believe has max precision of 7 bits
    static const float Epsilon;

    static void Point2fAdd(Point2fT* NewObj, const Tuple2fT* t1);

    /**
      * Sets the value of this tuple to the vector difference of itself and tuple t1 (this = this - t1).
      * @param t1 the other tuple
      */
    static void Point2fSub(Point2fT* NewObj, const Tuple2fT* t1);

    /**
      * Sets this vector to be the vector cross product of vectors v1 and v2.
      * @param v1 the first vector
      * @param v2 the second vector
      */
    static void Vector3fCross(Vector3fT* NewObj, const Vector3fT* v1, const Vector3fT* v2);

    /**
      * Computes the dot product of the this vector and vector v1.
      * @param  v1 the other vector
      */
    static GLfloat Vector3fDot(const Vector3fT* NewObj, const Vector3fT* v1);

    /**
      * Returns the squared length of this vector.
      * @return the squared length of this vector
      */
    static GLfloat Vector3fLengthSquared(const Vector3fT* NewObj);

    /**
      * Returns the length of this vector.
      * @return the length of this vector
      */
    static GLfloat Vector3fLength(const Vector3fT* NewObj);

    static void Matrix3fSetZero(Matrix3fT* NewObj);
    
    static void Matrix4fSetZero(Matrix4fT* NewObj);

    /**
     * Sets this Matrix3 to identity.
     */
    static void Matrix3fSetIdentity(Matrix3fT* NewObj);

    static void Matrix4fSetIdentity(Matrix4fT* NewObj);

    /**
      * Sets the value of this matrix to the matrix conversion of the
      * quaternion argument. 
      * @param q1 the quaternion to be converted 
      */
    //$hack this can be optimized some(if s == 0)
    static void Matrix3fSetRotationFromQuat4f(Matrix3fT* NewObj, const Quat4fT* q1);

    /**
     * Sets the value of this matrix to the result of multiplying itself
     * with matrix m1. 
     * @param m1 the other matrix 
     */
    static void Matrix3fMulMatrix3f(Matrix3fT* NewObj, const Matrix3fT* m1);

    static void Matrix4fSetRotationScaleFromMatrix4f(Matrix4fT* NewObj, const Matrix4fT* m1);

    /**
      * Performs SVD on this matrix and gets scale and rotation.
      * Rotation is placed into rot3, and rot4.
      * @param rot3 the rotation factor(Matrix3d). if null, ignored
      * @param rot4 the rotation factor(Matrix4) only upper 3x3 elements are changed. if null, ignored
      * @return scale factor
      */
    static GLfloat Matrix4fSVD(const Matrix4fT* NewObj, Matrix3fT* rot3, Matrix4fT* rot4);

    static void Matrix4fSetRotationScaleFromMatrix3f(Matrix4fT* NewObj, const Matrix3fT* m1);

    static void Matrix4fMulRotationScale(Matrix4fT* NewObj, GLfloat scale);

    /**
      * Sets the rotational component (upper 3x3) of this matrix to the matrix
      * values in the T precision Matrix3d argument; the other elements of
      * this matrix are unchanged; a singular value decomposition is performed
      * on this object's upper 3x3 matrix to factor out the scale, then this
      * object's upper 3x3 matrix components are replaced by the passed rotation
      * components, and then the scale is reapplied to the rotational
      * components.
      * @param m1 T precision 3x3 matrix
      */
    static void Matrix4fSetRotationFromMatrix3f(Matrix4fT* NewObj, const Matrix3fT* m1);
};

#endif
