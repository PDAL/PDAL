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

#include <pdal/util/Georeference.hpp>

#include <cmath>
#include <iostream>
#include <vector>


namespace pdal
{
namespace georeference
{


namespace
{


static const double a = 6378137.0;
static const double f = 1 / 298.257223563;
static const double e2 = 2 * f - f * f;


Xyz rotate(const Xyz& point, const RotationMatrix& matrix)
{
    return Xyz(
        matrix.m00 * point.X + matrix.m01 * point.Y + matrix.m02 * point.Z,
        matrix.m10 * point.X + matrix.m11 * point.Y + matrix.m12 * point.Z,
        matrix.m20 * point.X + matrix.m21 * point.Y + matrix.m22 * point.Z);
}


Xyz cartesianToCurvilinear(const Xyz& point, double latitude)
{
    double w = std::sqrt(1 - e2 * std::sin(latitude) * std::sin(latitude));
    double n = a / w;
    double m = a * (1 - e2) / (w * w * w);
    return Xyz(point.X / (n * std::cos(latitude)), point.Y / m, point.Z);
}
}


Xyz georeferenceWgs84(double range, double scanAngle,
                      const RotationMatrix& boresightMatrix,
                      const RotationMatrix& imuMatrix, const Xyz& gpsPoint)
{
    Xyz pSocs = Xyz(range * std::sin(scanAngle), 0, -range * std::cos(scanAngle));

    Xyz pSocsAligned = rotate(pSocs, boresightMatrix);
    Xyz pLocalLevel = rotate(pSocsAligned, imuMatrix);
    Xyz pCurvilinear = cartesianToCurvilinear(pLocalLevel, gpsPoint.Y);

    return Xyz(gpsPoint.X + pCurvilinear.X, gpsPoint.Y + pCurvilinear.Y,
               gpsPoint.Z + pCurvilinear.Z);
}
}
}
