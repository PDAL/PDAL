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
    OciSeqIterator(Statement stmt, BlockPtr block) :
        m_stmt(stmt), m_block(block), m_atEnd(false)
    {}

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
    char *seekDimMajor(const schema::DimInfo& d, BlockPtr block);
    char *seekPointMajor(BlockPtr block);
    bool readOci(Statement stmt, BlockPtr block);
    schema::XMLSchema *findSchema(Statement stmt, BlockPtr block);
    pdal::Bounds<double> getBounds(Statement stmt, BlockPtr block);

    Statement m_stmt;
    BlockPtr m_block;
    bool m_atEnd;
    std::map<int32_t, schema::XMLSchema> m_schemas;
};

} // namespace sequential
} // namespace iterators
} // namespace oci
} // namespace driver
} // namespace pdal

