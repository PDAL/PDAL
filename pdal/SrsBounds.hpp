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

#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

class PDAL_DLL SrsBounds : public Bounds
{
public:
    SrsBounds()
    {}

    explicit SrsBounds(const BOX3D& box);
    explicit SrsBounds(const BOX3D& box, const SpatialReference& srs);
    explicit SrsBounds(const BOX2D& box);
    explicit SrsBounds(const BOX2D& box, const SpatialReference& srs);

    void parse(const std::string& s, std::string::size_type& pos);
    SpatialReference spatialReference()
        { return m_srs; }

    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const SrsBounds& bounds);

private:
    SpatialReference m_srs;
};

namespace Utils
{
    template<>
    inline StatusWithReason fromString(const std::string& s,
        SrsBounds& srsBounds)
    {
        std::string::size_type pos(0);
        srsBounds.parse(s, pos);
        return true;
    }
}

PDAL_DLL std::ostream& operator << (std::ostream& out, const Bounds& bounds);

} // namespace pdal

