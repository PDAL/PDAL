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

#include "ArcBallMath.hpp"

#include <cassert>
#include <cmath>
#include <gl/gl.h>
                                                

const float ArcBallMath::Epsilon = 1.0e-5f;


void ArcBallMath::Point2fAdd(Point2fT* NewObj, const Tuple2fT* t1)
{
    assert(NewObj && t1);

    NewObj->s.X += t1->s.X;
    NewObj->s.Y += t1->s.Y;
}


void ArcBallMath::Point2fSub(Point2fT* NewObj, const Tuple2fT* t1)
{
    assert(NewObj && t1);

    NewObj->s.X -= t1->s.X;
    NewObj->s.Y -= t1->s.Y;
}


void ArcBallMath::Vector3fCross(Vector3fT* NewObj, const Vector3fT* v1, const Vector3fT* v2)
{
    Vector3fT Result; //safe not to initialize

    assert(NewObj && v1 && v2);

    // store on stack once for aliasing-safty
    // i.e. safe when a.cross(a, b)

    Result.s.X = (v1->s.Y * v2->s.Z) - (v1->s.Z * v2->s.Y);
    Result.s.Y = (v1->s.Z * v2->s.X) - (v1->s.X * v2->s.Z);
    Result.s.Z = (v1->s.X * v2->s.Y) - (v1->s.Y * v2->s.X);

    //copy result back
    *NewObj = Result;
}


GLfloat ArcBallMath::Vector3fDot(const Vector3fT* NewObj, const Vector3fT* v1)
{
    assert(NewObj && v1);

    return  (NewObj->s.X * v1->s.X) +
        (NewObj->s.Y * v1->s.Y) +
        (NewObj->s.Z * v1->s.Z);
}


GLfloat ArcBallMath::Vector3fLengthSquared(const Vector3fT* NewObj)
{
    assert(NewObj);

    return  (NewObj->s.X * NewObj->s.X) +
        (NewObj->s.Y * NewObj->s.Y) +
        (NewObj->s.Z * NewObj->s.Z);
}


GLfloat ArcBallMath::Vector3fLength(const Vector3fT* NewObj)
{
    assert(NewObj);

    return sqrtf(Vector3fLengthSquared(NewObj));
}


void ArcBallMath::Matrix3fSetZero(Matrix3fT* NewObj)
{
    NewObj->s.M00 = NewObj->s.M01 = NewObj->s.M02 = 
        NewObj->s.M10 = NewObj->s.M11 = NewObj->s.M12 = 
        NewObj->s.M20 = NewObj->s.M21 = NewObj->s.M22 = 0.0f;
}


void ArcBallMath::Matrix4fSetZero(Matrix4fT* NewObj)
{
    NewObj->s.M00 = NewObj->s.M01 = NewObj->s.M02 = NewObj->s.M03 = 
        NewObj->s.M10 = NewObj->s.M11 = NewObj->s.M12 = NewObj->s.M13 = 
        NewObj->s.M20 = NewObj->s.M21 = NewObj->s.M22 =  NewObj->s.M23 =
        NewObj->s.M30 = NewObj->s.M31 = NewObj->s.M32 =  NewObj->s.M33 = 0.0f;
}


void ArcBallMath::Matrix3fSetIdentity(Matrix3fT* NewObj)
{
    Matrix3fSetZero(NewObj);

    //then set diagonal as 1
    NewObj->s.M00 = 
        NewObj->s.M11 = 
        NewObj->s.M22 = 1.0f;
}


void ArcBallMath::Matrix4fSetIdentity(Matrix4fT* NewObj)
{
    Matrix4fSetZero(NewObj);

    //then set diagonal as 1
    NewObj->s.M00 = 
        NewObj->s.M11 = 
        NewObj->s.M22 =
        NewObj->s.M33 = 1.0f;
}


void ArcBallMath::Matrix3fSetRotationFromQuat4f(Matrix3fT* NewObj, const Quat4fT* q1)
{
    GLfloat n, s;
    GLfloat xs, ys, zs;
    GLfloat wx, wy, wz;
    GLfloat xx, xy, xz;
    GLfloat yy, yz, zz;

    assert(NewObj && q1);

    n = (q1->s.X * q1->s.X) + (q1->s.Y * q1->s.Y) + (q1->s.Z * q1->s.Z) + (q1->s.W * q1->s.W);
    s = (n > 0.0f) ? (2.0f / n) : 0.0f;

    xs = q1->s.X * s;  ys = q1->s.Y * s;  zs = q1->s.Z * s;
    wx = q1->s.W * xs; wy = q1->s.W * ys; wz = q1->s.W * zs;
    xx = q1->s.X * xs; xy = q1->s.X * ys; xz = q1->s.X * zs;
    yy = q1->s.Y * ys; yz = q1->s.Y * zs; zz = q1->s.Z * zs;

    NewObj->s.XX = 1.0f - (yy + zz); NewObj->s.YX =         xy - wz;  NewObj->s.ZX =         xz + wy;
    NewObj->s.XY =         xy + wz;  NewObj->s.YY = 1.0f - (xx + zz); NewObj->s.ZY =         yz - wx;
    NewObj->s.XZ =         xz - wy;  NewObj->s.YZ =         yz + wx;  NewObj->s.ZZ = 1.0f - (xx + yy);
}


void ArcBallMath::Matrix3fMulMatrix3f(Matrix3fT* NewObj, const Matrix3fT* m1)
{
    Matrix3fT Result; //safe not to initialize

    assert(NewObj && m1);

    // alias-safe way.
    Result.s.M00 = (NewObj->s.M00 * m1->s.M00) + (NewObj->s.M01 * m1->s.M10) + (NewObj->s.M02 * m1->s.M20);
    Result.s.M01 = (NewObj->s.M00 * m1->s.M01) + (NewObj->s.M01 * m1->s.M11) + (NewObj->s.M02 * m1->s.M21);
    Result.s.M02 = (NewObj->s.M00 * m1->s.M02) + (NewObj->s.M01 * m1->s.M12) + (NewObj->s.M02 * m1->s.M22);

    Result.s.M10 = (NewObj->s.M10 * m1->s.M00) + (NewObj->s.M11 * m1->s.M10) + (NewObj->s.M12 * m1->s.M20);
    Result.s.M11 = (NewObj->s.M10 * m1->s.M01) + (NewObj->s.M11 * m1->s.M11) + (NewObj->s.M12 * m1->s.M21);
    Result.s.M12 = (NewObj->s.M10 * m1->s.M02) + (NewObj->s.M11 * m1->s.M12) + (NewObj->s.M12 * m1->s.M22);

    Result.s.M20 = (NewObj->s.M20 * m1->s.M00) + (NewObj->s.M21 * m1->s.M10) + (NewObj->s.M22 * m1->s.M20);
    Result.s.M21 = (NewObj->s.M20 * m1->s.M01) + (NewObj->s.M21 * m1->s.M11) + (NewObj->s.M22 * m1->s.M21);
    Result.s.M22 = (NewObj->s.M20 * m1->s.M02) + (NewObj->s.M21 * m1->s.M12) + (NewObj->s.M22 * m1->s.M22);

    //copy result back to this
    *NewObj = Result;
}


void ArcBallMath::Matrix4fSetRotationScaleFromMatrix4f(Matrix4fT* NewObj, const Matrix4fT* m1)
{
    assert(NewObj && m1);

    NewObj->s.XX = m1->s.XX; NewObj->s.YX = m1->s.YX; NewObj->s.ZX = m1->s.ZX;
    NewObj->s.XY = m1->s.XY; NewObj->s.YY = m1->s.YY; NewObj->s.ZY = m1->s.ZY;
    NewObj->s.XZ = m1->s.XZ; NewObj->s.YZ = m1->s.YZ; NewObj->s.ZZ = m1->s.ZZ;
}


GLfloat ArcBallMath::Matrix4fSVD(const Matrix4fT* NewObj, Matrix3fT* rot3, Matrix4fT* rot4)
{
    GLfloat s, n;

    assert(NewObj);

    // this is a simple svd.
    // Not complete but fast and reasonable.
    // See comment in Matrix3d.

    s = sqrtf(
        ( (NewObj->s.XX * NewObj->s.XX) + (NewObj->s.XY * NewObj->s.XY) + (NewObj->s.XZ * NewObj->s.XZ) + 
        (NewObj->s.YX * NewObj->s.YX) + (NewObj->s.YY * NewObj->s.YY) + (NewObj->s.YZ * NewObj->s.YZ) +
        (NewObj->s.ZX * NewObj->s.ZX) + (NewObj->s.ZY * NewObj->s.ZY) + (NewObj->s.ZZ * NewObj->s.ZZ) ) / 3.0f );

    if (rot3)   //if pointer not null
    {
        //this->getRotationScale(rot3);
        rot3->s.XX = NewObj->s.XX; rot3->s.XY = NewObj->s.XY; rot3->s.XZ = NewObj->s.XZ;
        rot3->s.YX = NewObj->s.YX; rot3->s.YY = NewObj->s.YY; rot3->s.YZ = NewObj->s.YZ;
        rot3->s.ZX = NewObj->s.ZX; rot3->s.ZY = NewObj->s.ZY; rot3->s.ZZ = NewObj->s.ZZ;

        // zero-div may occur.

        n = 1.0f / sqrtf( (NewObj->s.XX * NewObj->s.XX) +
            (NewObj->s.XY * NewObj->s.XY) +
            (NewObj->s.XZ * NewObj->s.XZ) );
        rot3->s.XX *= n;
        rot3->s.XY *= n;
        rot3->s.XZ *= n;

        n = 1.0f / sqrtf( (NewObj->s.YX * NewObj->s.YX) +
            (NewObj->s.YY * NewObj->s.YY) +
            (NewObj->s.YZ * NewObj->s.YZ) );
        rot3->s.YX *= n;
        rot3->s.YY *= n;
        rot3->s.YZ *= n;

        n = 1.0f / sqrtf( (NewObj->s.ZX * NewObj->s.ZX) +
            (NewObj->s.ZY * NewObj->s.ZY) +
            (NewObj->s.ZZ * NewObj->s.ZZ) );
        rot3->s.ZX *= n;
        rot3->s.ZY *= n;
        rot3->s.ZZ *= n;
    }

    if (rot4)   //if pointer not null
    {
        if (rot4 != NewObj)
        {
            Matrix4fSetRotationScaleFromMatrix4f(rot4, NewObj);  // private method
        }

        // zero-div may occur.

        n = 1.0f / sqrtf( (NewObj->s.XX * NewObj->s.XX) +
            (NewObj->s.XY * NewObj->s.XY) +
            (NewObj->s.XZ * NewObj->s.XZ) );
        rot4->s.XX *= n;
        rot4->s.XY *= n;
        rot4->s.XZ *= n;

        n = 1.0f / sqrtf( (NewObj->s.YX * NewObj->s.YX) +
            (NewObj->s.YY * NewObj->s.YY) +
            (NewObj->s.YZ * NewObj->s.YZ) );
        rot4->s.YX *= n;
        rot4->s.YY *= n;
        rot4->s.YZ *= n;

        n = 1.0f / sqrtf( (NewObj->s.ZX * NewObj->s.ZX) +
            (NewObj->s.ZY * NewObj->s.ZY) +
            (NewObj->s.ZZ * NewObj->s.ZZ) );
        rot4->s.ZX *= n;
        rot4->s.ZY *= n;
        rot4->s.ZZ *= n;
    }

    return s;
}


void ArcBallMath::Matrix4fSetRotationScaleFromMatrix3f(Matrix4fT* NewObj, const Matrix3fT* m1)
{
    assert(NewObj && m1);

    NewObj->s.XX = m1->s.XX; NewObj->s.YX = m1->s.YX; NewObj->s.ZX = m1->s.ZX;
    NewObj->s.XY = m1->s.XY; NewObj->s.YY = m1->s.YY; NewObj->s.ZY = m1->s.ZY;
    NewObj->s.XZ = m1->s.XZ; NewObj->s.YZ = m1->s.YZ; NewObj->s.ZZ = m1->s.ZZ;
}


void ArcBallMath::Matrix4fMulRotationScale(Matrix4fT* NewObj, GLfloat scale)
{
    assert(NewObj);

    NewObj->s.XX *= scale; NewObj->s.YX *= scale; NewObj->s.ZX *= scale;
    NewObj->s.XY *= scale; NewObj->s.YY *= scale; NewObj->s.ZY *= scale;
    NewObj->s.XZ *= scale; NewObj->s.YZ *= scale; NewObj->s.ZZ *= scale;
}


void ArcBallMath::Matrix4fSetRotationFromMatrix3f(Matrix4fT* NewObj, const Matrix3fT* m1)
{
    GLfloat scale;

    assert(NewObj && m1);

    scale = Matrix4fSVD(NewObj, NULL, NULL);

    Matrix4fSetRotationScaleFromMatrix3f(NewObj, m1);
    Matrix4fMulRotationScale(NewObj, scale);
}
