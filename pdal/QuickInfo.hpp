/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <vector>

#include <pdal/util/Bounds.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal
{

struct QuickInfo
{
public:
    BOX3D m_bounds;
    SpatialReference m_srs;
    point_count_t m_pointCount;
    StringList m_dimNames;
    MetadataNode m_metadata;

    bool m_valid;

    QuickInfo() : m_pointCount(0), m_valid(false)
        {}

    bool valid() const
        { return m_valid; }

    MetadataNode dumpSummary() const
    {
        MetadataNode summary;
        summary.add("num_points", this->m_pointCount);
        if (this->m_srs.valid())
        {
            MetadataNode srs = this->m_srs.toMetadata();
            summary.add(srs);
        }
        if (this->m_bounds.valid())
        {
            MetadataNode bounds = Utils::toMetadata(this->m_bounds);
            summary.add(bounds.clone("bounds"));
        }

        std::string dims;
        auto di = this->m_dimNames.begin();
        while (di != this->m_dimNames.end())
        {
            dims += *di;
            ++di;
            if (di != this->m_dimNames.end())
               dims += ", ";
        }
        if (dims.size())
            summary.add("dimensions", dims);

        if (!this->m_metadata.empty() && this->m_metadata.valid())
        {
            summary.add(this->m_metadata.clone("metadata"));
        }

        return summary;
    }
};

} // namespace pdal

