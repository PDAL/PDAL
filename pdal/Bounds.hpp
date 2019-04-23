/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/util/Box.hpp>

namespace pdal
{

/**
  Wrapper for BOX3D and BOX2D to allow extraction as either.  Typically used
  to facilitate streaming either a BOX2D or BOX3D
*/
class PDAL_DLL Bounds
{
public:
    Bounds()
    {}

    explicit Bounds(const BOX3D& box);
    explicit Bounds(const BOX2D& box);

    BOX3D to3d() const;
    BOX2D to2d() const;
    bool is3d() const;

    friend PDAL_DLL std::istream& operator >> (std::istream& in,
        Bounds& bounds);
    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const Bounds& bounds);

private:
    BOX3D m_box;

    void set(const BOX3D& box);
    void set(const BOX2D& box);
};

PDAL_DLL std::istream& operator >> (std::istream& in, Bounds& bounds);
PDAL_DLL std::ostream& operator << (std::ostream& in, const Bounds& bounds);

} // namespace pdal

