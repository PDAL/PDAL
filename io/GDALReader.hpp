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

class PDAL_EXPORT GDALReader : public Reader , public Streamable
{
public:
    std::string getName() const;

    GDALReader();
    ~GDALReader();

private:
    class BlockReader
    {
    public:
        BlockReader(GDALReader& reader);
        void initialize();
        point_count_t processBlock(PointViewPtr view);
        bool processOne(PointRef& point);

    private:
        struct Block
        {
            std::vector<std::vector<double>> m_data;
            int m_blockCol;
            int m_blockRow;
        };

        bool readBlock();

        GDALReader& m_reader;
        int m_blockRow;
        int m_blockCol;
        int m_blockWidth;
        int m_blockHeight;
        int m_numBlocksX;
        int m_numBlocksY;

        Block m_currentBlock;
        bool m_needsRead;
        int m_colInBlock;
        int m_rowInBlock;
        std::array<double, 2> m_coords;
    };

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
    pdal::StringList m_GDAL_metadata;
    std::string m_header;
    int m_width;
    int m_height;
    bool m_useMemoryCopy;

    BlockReader m_blockReader;

    BOX3D m_bounds;
    StringList m_dimNames;
    StringList m_options;
};

} // namespace pdal

