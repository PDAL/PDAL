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

#include <memory>
#include <vector>

#include "pdal/Dimension.hpp"
#include "pdal/Schema.hpp"

namespace pdal
{

/// This class provides a place to store the point data.
class RawPtBuf
{
public:
    RawPtBuf(SchemaPtr schema) : m_numPts{0}, m_allocPts{0},
        m_schema{schema}
    {}

    PointId addPoint()
    {
        if (m_numPts >= m_allocPts)
        {
            m_allocPts += 100000;
            m_buf.resize(pointsToBytes(m_allocPts));
        }
        return m_numPts++;
    }

    void setField(const Dimension& dim, PointId idx, void *value)
    {
        size_t offset = pointsToBytes(idx) + dim.getByteOffset();
        memcpy(m_buf.data() + offset, value, dim.getByteSize());
    }

    template <typename T>
    T getField(const Dimension& dim, PointId idx);

private:
    std::vector<char> m_buf;
    point_count_t m_numPts;
    point_count_t m_allocPts;
    SchemaPtr m_schema;
    
    size_t pointsToBytes(point_count_t numPts)
        { return m_schema->getByteSize() * numPts; }

};
typedef std::shared_ptr<RawPtBuf> RawPtBufPtr;


template <typename T>
T RawPtBuf::getField(const Dimension& dim, PointId idx)
{
    T t;

    size_t offset = pointsToBytes(idx) + dim.getByteOffset();
    memcpy(&t, m_buf.data() + offset, sizeof(T));
    return t;
}


} // namespace pdal

