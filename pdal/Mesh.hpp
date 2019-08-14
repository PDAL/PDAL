/******************************************************************************
* Copyright (c) 2017, Hobu Inc.
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

#include <array>
#include <deque>

namespace pdal
{

class PDAL_DLL Triangle
{
public:
    Triangle(PointId a, PointId b, PointId c) : m_a(a), m_b(b), m_c(c)
    {}

    PointId m_a;
    PointId m_b;
    PointId m_c;

    friend bool operator == (const Triangle& a, const Triangle& b);
};

inline bool operator == (const Triangle& a, const Triangle& b)
{
    std::array<PointId, 3> aa { {a.m_a, a.m_b, a.m_c} };
    std::array<PointId, 3> bb { {b.m_a, b.m_b, b.m_c} };
    std::sort(aa.begin(), aa.end());
    std::sort(bb.begin(), bb.end());
    return aa == bb;
}

/**
  A mesh is a way to represent a set of points connected by edges.  Point
  indices are into a point view.
*/
class Mesh
{};


/**
  A mesh where the faces are triangles.
*/
class TriangularMesh : public Mesh
{
public:
    using const_iterator = std::deque<Triangle>::const_iterator;

    PDAL_DLL TriangularMesh()
    {}

    size_t PDAL_DLL size() const
        { return m_index.size(); }
    void PDAL_DLL add(PointId a, PointId b, PointId c)
        { m_index.emplace_back(a, b, c); }
    const PDAL_DLL Triangle& operator[](PointId id) const
        { return m_index[id]; }
    const_iterator begin() const
        { return m_index.begin(); }
    const_iterator end() const
        { return m_index.end(); }

protected:
    std::deque<Triangle> m_index;
};

} // namespace pdal
