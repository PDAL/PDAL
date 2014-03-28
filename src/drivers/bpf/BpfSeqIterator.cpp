/******************************************************************************
* Copyright (c) 2014, Andrew Bell
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

#include <vector>

#include <pdal/PointBuffer.hpp>
#include <pdal/Schema.hpp>

#include "BpfSeqIterator.hpp"
#include "BpfReader.hpp"

namespace pdal
{

BpfSeqIterator::BpfSeqIterator(PointBuffer& data, boost::uint32_t numPoints,
        ILeStream& stream) :
    ReaderSequentialIterator(data), m_numPoints(numPoints),
    m_stream(stream), m_index(0)
{}

boost::uint32_t BpfSeqIterator::readBufferImpl(PointBuffer& data)
{
    std::vector<Dimension> dims;

    const Schema& schema = data.getSchema();
    for (size_t i = 0; i < schema.numDimensions(); ++i)
    {
        Dimension d = schema.getDimension(i);
        if (d.getNamespace() == "bpf")
            dims.push_back(d);
    }

    return readPointMajor(data, dims);
}

boost::uint64_t BpfSeqIterator::skipImpl(boost::uint64_t)
{
    //ABELL - Fix
    return 0;
}

bool BpfSeqIterator::atEndImpl() const
{
    return m_index >= m_numPoints;
}

boost::uint32_t BpfSeqIterator::readPointMajor(PointBuffer& data,
    const std::vector<Dimension>& dims)
{
    uint32_t capacity = data.getCapacity();
    uint32_t idx = data.getNumPoints();
    skipImpl(idx);
    while (idx < capacity && idx < m_numPoints)
    {
        for (size_t d = 0; d < dims.size(); ++d)
        {
            float f;

            m_stream >> f;
            data.setField<float>(dims[d], idx, f);
        }
        idx++;
    }
    return idx - data.getNumPoints();
}

boost::uint32_t BpfSeqIterator::readDimMajor(PointBuffer& data,
    const std::vector<Dimension>& dims)
{
    uint32_t capacity = data.getCapacity();

    uint32_t idx;
    for (size_t d = 0; d < dims.size(); ++d)
    {
       idx = data.getNumPoints();
       skipDimMajor(d, idx);
       for (; idx < capacity && idx < m_numPoints; idx++)
       {
           float f;

           m_stream >> f;
           data.setField<float>(dims[d], idx, f);
           idx++;
        }
    }
    return idx - data.getNumPoints();
}

boost::uint32_t BpfSeqIterator::readByteMajor(PointBuffer& data,
    const std::vector<Dimension>& dims)
{
    uint32_t idx;
    uint32_t capacity = data.getCapacity();

    for (size_t d = 0; d < dims.size(); ++d)
    {
        for (size_t b = 0; b < sizeof(float); ++b)
        {
            idx = data.getNumPoints();
            skipByteMajor(d, idx);
            for (;idx < capacity && idx < m_numPoints; idx++)
            {
                union 
                {
                    float f;
                    uint32_t u32;
                } u;

                u.u32 = 0;

                if (b)
                {
                    u.f = data.getField<float>(dims[d], idx);
                }
                uint8_t u8;
                m_stream >> u8;
                u.u32 |= ((uint32_t)u8 << b);
                data.setField<float>(dims[d], idx, u.f);
            }
        }
    }
    return idx - data.getNumPoints();
}


void BpfSeqIterator::skipDimMajor(size_t dimIdx, uint32_t ptIdx)
{
    (void)dimIdx;
    (void)ptIdx;
}


void BpfSeqIterator::skipByteMajor(size_t dimIdx, uint32_t ptIdx)
{
    (void)dimIdx;
    (void)ptIdx;
}

} //namespace
