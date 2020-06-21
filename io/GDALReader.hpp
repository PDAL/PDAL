/******************************************************************************
* Copyright (c) 2015, Howard Butler <howard@hobu.co>
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

#include <string>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

namespace gdal { class Raster; }

typedef std::map<std::string, Dimension::Id> DimensionMap;

class PDAL_DLL GDALReader : public Reader , public Streamable
{
public:
    std::string getName() const;

    GDALReader();
    ~GDALReader();

private:
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual void done(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual QuickInfo inspect();
    virtual void addArgs(ProgramArgs& args);

    std::unique_ptr<gdal::Raster> m_raster;
    std::vector<Dimension::Type> m_bandTypes;
    std::vector<Dimension::Id> m_bandIds;
    std::string m_header;
    int m_width;
    int m_height;
    bool m_useMemoryCopy;
    point_count_t m_index;
    int m_row;
    int m_col;

    BOX3D m_bounds;
    StringList m_dimNames;
};

} // namespace pdal

