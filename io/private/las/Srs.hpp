/******************************************************************************
* Copyright (c) 2021, Hobu Inc.
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "Vlr.hpp"

#include  <pdal/Log.hpp>
#include  <pdal/util/Algorithm.hpp>

namespace pdal
{
namespace las
{

enum class SrsType
{
    Wkt1,
    Geotiff,
    Proj,
    Wkt2
};

/**
inline std::istream& operator>>(std::istream& in, SrsType& type)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "wkt1")
        type = SrsType::Wkt1;
    else if (s == "geotiff")
        type = SrsType::Geotiff;
    else if (s == "proj")
        type = SrsType::Proj;
    else if (s == "wkt2")
        type = SrsType::Wkt2;
    else
        throw pdal_error("Invalid SRS type '" + s + "'. Must be one of 'wkt1', 'geotiff', "
            "'proj' or 'wk2'.");
    return in;
}
**/

inline std::ostream& operator<<(std::ostream& out, SrsType type)
{
    switch (type)
    {
    case SrsType::Wkt1:
        out << "wkt1";
        break;
    case SrsType::Geotiff:
        out << "geotiff";
        break;
    case SrsType::Proj:
        out << "proj";
        break;
    case SrsType::Wkt2:
        out << "wkt2";
        break;
    }
    return out;
}

class Srs
{
public:
    void init(const VlrList& vlrs, std::vector<SrsType> srsOrder, bool useWkt, LogPtr log);
    SpatialReference get() const;
    std::string geotiffString() const;

private:
    void extractGeotiff(const Vlr *vlr, const VlrList& vlrs, LogPtr log);
    void extractWkt(const Vlr *vlr);

    std::string m_geotiffString;
    SpatialReference m_srs;
};

} // namespace las

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    std::vector<las::SrsType>& srsOrder)
{
    using namespace las;

     static const std::map<std::string, SrsType> typemap =
        { { "wkt2", SrsType::Wkt2 },
          { "wkt1", SrsType::Wkt1 },
          { "proj", SrsType::Proj },
          { "geotiff", SrsType::Geotiff } };

    StringList srsTypes = Utils::split2(from, ',');
    std::transform(srsTypes.cbegin(), srsTypes.cend(), srsTypes.begin(),
        [](std::string s){ Utils::trim(s); return Utils::tolower(s); });

    for (std::string& stype : srsTypes)
    {
        auto it = typemap.find(stype);
        if (it == typemap.end())
            return { -1, "Invalid SRS type '" + stype + "'." };
        SrsType type = it->second;
        if (Utils::contains(srsOrder, type))
            return { -1,
                "Duplicate SRS type '" + stype + "' in 'vlr_srs_order'" };
        srsOrder.push_back(type);
    }
    return 0;
}

} // namespace Utils

} // namespace pdal
