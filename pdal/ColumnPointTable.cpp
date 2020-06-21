/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include <pdal/PointTable.hpp>

namespace pdal
{

ColumnPointTable::~ColumnPointTable()
{
    for (DimBlockList& l : m_blocks)
        for (auto ptr : l)
            delete [] ptr;
}


void ColumnPointTable::finalize()
{
    m_layoutRef.orderDimensions();
    m_blocks.resize(m_layoutRef.dims().size());
}


PointId ColumnPointTable::addPoint()
{
    if (m_numPts % m_blockPtCnt == 0)
    {
        for (Dimension::Id id : m_layoutRef.dims())
        {
            const Dimension::Detail *detail = m_layoutRef.dimDetail(id);

            // Make a block that holds m_blockPtCnt values of a dimension.
            size_t size = m_blockPtCnt * Dimension::size(detail->type());
            char *buf = new char[size];
            memset(buf, 0, size);
            DimBlockList& dimBlocks = m_blocks[detail->order()];
            dimBlocks.push_back(buf);
        }
    }
    return m_numPts++;
}

namespace
{

void copy(const char *src, char *dst, Dimension::Type type)
{
    switch (type)
    {
    case Dimension::Type::Double:
        *reinterpret_cast<double *>(dst) =
            *reinterpret_cast<const double *>(src);
        break;
    case Dimension::Type::Float:
        *reinterpret_cast<float *>(dst) =
            *reinterpret_cast<const float*>(src);
        break;
    case Dimension::Type::Signed8:
        *reinterpret_cast<int8_t *>(dst) =
            *reinterpret_cast<const int8_t *>(src);
        break;
    case Dimension::Type::Signed16:
        *reinterpret_cast<int16_t *>(dst) =
            *reinterpret_cast<const int16_t *>(src);
        break;
    case Dimension::Type::Signed32:
        *reinterpret_cast<int32_t *>(dst) =
            *reinterpret_cast<const int32_t *>(src);
        break;
    case Dimension::Type::Signed64:
        *reinterpret_cast<int64_t *>(dst) =
            *reinterpret_cast<const int64_t *>(src);
        break;
    case Dimension::Type::Unsigned8:
        *reinterpret_cast<uint8_t *>(dst) =
            *reinterpret_cast<const uint8_t *>(src);
        break;
    case Dimension::Type::Unsigned16:
        *reinterpret_cast<uint16_t *>(dst) =
            *reinterpret_cast<const uint16_t *>(src);
        break;
    case Dimension::Type::Unsigned32:
        *reinterpret_cast<uint32_t *>(dst) =
            *reinterpret_cast<const uint32_t *>(src);
        break;
    case Dimension::Type::Unsigned64:
        *reinterpret_cast<uint64_t *>(dst) =
            *reinterpret_cast<const uint64_t *>(src);
        break;
    case Dimension::Type::None:
    default:
        break;
    }
}

} // unnamed namespace


void ColumnPointTable::setFieldInternal(Dimension::Id dim,
    PointId idx, const void *src)
{
    const Dimension::Detail *d = m_layoutRef.dimDetail(dim);
    const DimBlockList& dimBlocks = m_blocks[d->order()];
    char *buf = dimBlocks[idx / m_blockPtCnt];
    char *dst = buf + (Dimension::size(d->type()) * (idx % m_blockPtCnt));

    copy (reinterpret_cast<const char *>(src), dst, d->type());
}


void ColumnPointTable::getFieldInternal(Dimension::Id dim,
    PointId idx, void *dst) const
{
    const Dimension::Detail *d = m_layoutRef.dimDetail(dim);
    const DimBlockList& dimBlocks = m_blocks[d->order()];
    const char *buf = dimBlocks[idx / m_blockPtCnt];
    const char *src = buf + (Dimension::size(d->type()) * (idx % m_blockPtCnt));

    copy(src, reinterpret_cast<char *>(dst), d->type());
}

char *ColumnPointTable::getDimension(const Dimension::Detail *d, PointId idx)
{
    DimBlockList& dimBlocks = m_blocks[d->order()];
    char *buf = dimBlocks[idx / m_blockPtCnt];
    return buf + (Dimension::size(d->type()) * (idx % m_blockPtCnt));
}


const char *ColumnPointTable::getDimension(const Dimension::Detail *d,
    PointId idx) const
{
    ColumnPointTable *ncThis = const_cast<ColumnPointTable *>(this);
    return ncThis->getDimension(d, idx);
}

} // namespace pdal

