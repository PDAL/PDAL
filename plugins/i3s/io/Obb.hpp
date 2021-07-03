/******************************************************************************
* Copyright (c) 2020, Hobu Inc. (info@hobu.co)
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

#include <Eigen/Geometry>
#include <pdal/JsonFwd.hpp>
#include <pdal/util/Utils.hpp>

#include "EsriUtil.hpp"

namespace pdal
{
class SrsTransform;

namespace i3s
{
using Segment = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

class Obb
{
    FRIEND_TEST(ObbTest, obb);
public:
    // Can throw EsriError.
    Obb();
    Obb(const NL::json& spec);
    void parse(NL::json spec);
    bool intersect(Obb clip);
    void transform(const SrsTransform& xform);
    bool valid() const;
    Eigen::Vector3d center() const;
    Eigen::Quaterniond quat() const;
    BOX3D bounds() const;

private:
    void verifyArray(const NL::json& spec, const std::string& name, size_t cnt);
    bool intersectNormalized(const Segment& seg) const;
    Eigen::Vector3d corner(size_t pos);
    static bool halfIntersect(const Obb& a, Obb b);
    Segment segment(size_t pos);
    // Test support
    void setCenter(const Eigen::Vector3d& center);

    bool m_valid;
    Eigen::Vector3d m_p;
    double m_hx;
    double m_hy;
    double m_hz;    
    Eigen::Quaterniond m_quat;

    friend std::ostream& operator<<(std::ostream&, const Obb&);
};

} //namespace i3s

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::i3s::Obb& obb)
{
    NL::json spec;
    try
    {
        spec = NL::json::parse(from);
    }
    catch (const NL::json::exception& ex)
    {
        return { -1, ex.what() };
    }
    try
    {
        obb.parse(spec);
    }
    catch (const i3s::EsriError& err)
    {
        return { -1, err.what() };
    }
    return true;
}

} // namespace Utils
} // namespace pdal 
