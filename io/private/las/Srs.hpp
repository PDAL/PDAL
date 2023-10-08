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
    Wkt1 = 0,
    Geotiff,
    Proj,
    Wkt2
};

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
        out << "projjson";
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
    const SpatialReference& get() const;
    std::string geotiffString() const;

private:
    void extractGeotiff(const Vlr *vlr, const VlrList& vlrs, LogPtr log);
    void extractWkt(const Vlr *vlr);

    std::string m_geotiffString;
    SpatialReference m_srs;
};

} // namespace las
} // namespace pdal
