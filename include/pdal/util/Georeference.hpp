/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#pragma once

#include <pdal/pdal_export.hpp>

namespace pdal
{
namespace georeference
{


struct Xyz
{
    Xyz(double x, double y, double z)
        : X(x)
        , Y(y)
        , Z(z)
    {
    }

    double X;
    double Y;
    double Z;
};


struct RotationMatrix
{
    // Row-major
    RotationMatrix(double m00, double m01, double m02, double m10, double m11,
                   double m12, double m20, double m21, double m22)
        : m00(m00)
        , m01(m01)
        , m02(m02)
        , m10(m10)
        , m11(m11)
        , m12(m12)
        , m20(m20)
        , m21(m21)
        , m22(m22)
    {
    }

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;
};


inline RotationMatrix createIdentityMatrix()
{
    return RotationMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
}


// Returns Latitude, Longitude, Height triplet with angles in radians
PDAL_DLL Xyz georeferenceWgs84(double range, double scanAngle,
                      const RotationMatrix& boresightMatrix,
                      const RotationMatrix& imuMatrix, const Xyz& gpsPoint);
}
}
