/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <map>
#include <vector>

#include <pdal/drivers/oci/common.hpp>
#include <pdal/ReaderIterator.hpp>

namespace pdal
{

class Dimension;

namespace drivers
{
namespace oci
{
namespace iterators
{
namespace sequential
{

class OciSeqIterator : public ReaderSequentialIterator
{
public:
    OciSeqIterator(Statement stmt, BlockPtr block,
        std::vector<Dimension *> dims, bool normalizeXYZ, bool setPointSource) :
        m_stmt(stmt), m_block(block), m_dims(dims),
        m_normalizeXYZ(normalizeXYZ), m_setPointSourceId(setPointSource), m_atEnd(false)
    {
        for (size_t i = 0; i < m_dims.size(); ++i)
        {
            if (m_dims[i]->getName() == "X")
                m_dimX = m_dims[i];
            if (m_dims[i]->getName() == "Y")
                m_dimY = m_dims[i];
            if (m_dims[i]->getName() == "Z")
                m_dimZ = m_dims[i];
        }
    }

protected:
    point_count_t readBufferImpl(PointBuffer& buffer)
    {
        return readImpl(buffer, (std::numeric_limits<point_count_t>::max)());
    }
    point_count_t readImpl(PointBuffer& buffer, point_count_t count);
    uint64_t skipImpl(uint64_t count);
    bool atEndImpl() const
        { return m_atEnd; }

private:
    void readBlob(Statement stmt, BlockPtr block);
    point_count_t read(PointBuffer& buffer, BlockPtr block,
        point_count_t numPts);
    point_count_t readDimMajor(PointBuffer& buffer, BlockPtr block,
        point_count_t numPts);
    point_count_t readPointMajor(PointBuffer& buffer, BlockPtr block,
        point_count_t numPts);
    char *seekDimMajor(size_t dimIdx, BlockPtr block);
    char *seekPointMajor(BlockPtr block);
    void normalize(PointBuffer& buffer, BlockPtr block, PointId begin,
        PointId end);
    void setpointids(PointBuffer& buffer, BlockPtr block,
            PointId begin, PointId end);
    bool readOci(Statement stmt, BlockPtr block);
    Schema *findSchema(Statement stmt, BlockPtr block);
    pdal::Bounds<double> getBounds(Statement stmt, BlockPtr block);

    Statement m_stmt;
    BlockPtr m_block;
    std::vector<Dimension *> m_dims;
    bool m_normalizeXYZ;
    bool m_setPointSourceId;
    bool m_atEnd;
    Dimension *m_dimX;
    Dimension *m_dimY;
    Dimension *m_dimZ;
    std::map<int32_t, Schema> m_schemas;
};

} // namespace sequential
} // namespace iterators
} // namespace oci
} // namespace driver
} // namespace pdal

