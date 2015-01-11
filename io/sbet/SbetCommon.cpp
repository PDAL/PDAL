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

#include "SbetCommon.hpp"

namespace pdal
{

Dimension::IdList fileDimensions()
{
    Dimension::IdList ids;

    // Data for each point is in the source file in the order these dimensions
    // are listed, I would suppose.  Would be really nice to have a reference
    // to the file spec.  I searched the Internet and found that it is from
    // some company called Applanix (Trimble), but I can't find anything
    // describing the file format on their website.

    using namespace Dimension;
    ids.push_back(Id::GpsTime);
    ids.push_back(Id::Y);
    ids.push_back(Id::X);
    ids.push_back(Id::Z);
    ids.push_back(Id::XVelocity);
    ids.push_back(Id::YVelocity);
    ids.push_back(Id::ZVelocity);
    ids.push_back(Id::Roll);
    ids.push_back(Id::Pitch);
    ids.push_back(Id::PlatformHeading);
    ids.push_back(Id::WanderAngle);
    ids.push_back(Id::XBodyAccel);
    ids.push_back(Id::YBodyAccel);
    ids.push_back(Id::ZBodyAccel);
    ids.push_back(Id::XBodyAngRate);
    ids.push_back(Id::YBodyAngRate);
    ids.push_back(Id::ZBodyAngRate);

    return ids;
}

} // namespace pdal
