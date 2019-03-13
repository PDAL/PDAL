/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <pdal/Dimension.hpp>

namespace pdal
{
namespace sbet
{

// This is static so as to be made local (internal linkage) in the translation
// units in which it's included.
static inline Dimension::IdList fileDimensions()
{
   // Data for each point is in the source file in the order these dimensions
    // are listed, I would suppose.  Would be really nice to have a reference
    // to the file spec.  I searched the Internet and found that it is from
    // some company called Applanix (Trimble), but I can't find anything
    // describing the file format on their website.

    using namespace Dimension;
    return { Id::GpsTime, Id::Y, Id::X, Id::Z, Id::XVelocity, Id::YVelocity,
        Id::ZVelocity, Id::Roll, Id::Pitch, Id::Azimuth, Id::WanderAngle,
        Id::XBodyAccel, Id::YBodyAccel, Id::ZBodyAccel, Id::XBodyAngRate,
        Id::YBodyAngRate, Id::ZBodyAngRate };
}

static inline bool isAngularDimension(Dimension::Id dimension) {
    using namespace Dimension;
    switch (dimension) {
        case Id::X:
        case Id::Y:
        case Id::Roll:
        case Id::Pitch:
        case Id::Azimuth:
        case Id::WanderAngle:
        case Id::XBodyAngRate:
        case Id::YBodyAngRate:
        case Id::ZBodyAngRate:
            return true;
        default:
            return false;
    }
};

} // namespace sbet
} // namespace pdal
