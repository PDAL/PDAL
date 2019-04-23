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
*     * Neither the name of Hobu, Inc. names of its contributors may be
*       used to endorse or promote products derived from this software
*       without specific prior written permission.
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

#include <pdal/Bounds.hpp>

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

namespace pdal
{

Bounds::Bounds(const BOX3D& box) : m_box(box)
{}


Bounds::Bounds(const BOX2D& box) : m_box(box)
{
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
}


// We don't allow implicit conversion from a BOX2D to BOX3D.  Use the explicit
// BOX3D ctor that takes a BOX2D if that's what you want.
BOX3D Bounds::to3d() const
{
    if (!is3d())
        return BOX3D();
    return m_box;
}


BOX2D Bounds::to2d() const
{
    return m_box.to2d();
}


bool Bounds::is3d() const
{
    return (m_box.minz != HIGHEST || m_box.maxz != LOWEST);
}


void Bounds::set(const BOX3D& box)
{
    m_box = box;
}


void Bounds::set(const BOX2D& box)
{
    m_box = BOX3D(box);
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
}

std::istream& operator>>(std::istream& in, Bounds& bounds)
{
    std::streampos start = in.tellg();
    BOX3D b3d;
    in >> b3d;
    if (in.fail())
    {
        in.clear();
        in.seekg(start);
        BOX2D b2d;
        in >> b2d;
        if (!in.fail())
            bounds.set(b2d);
    }
    else
        bounds.set(b3d);
    return in;
}

std::ostream& operator<<(std::ostream& out, const Bounds& bounds)
{
    if (bounds.is3d())
        out << bounds.to3d();
    else
        out << bounds.to2d();
    return out;
}

} // namespace pdal
