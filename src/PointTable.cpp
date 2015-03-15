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

#include <pdal/PointTable.hpp>

namespace pdal
{

PointTable::PointTable()
    : m_layout(new PointLayout())
    , m_metadata(new Metadata())
{}

PointTable::PointTable(PointLayoutPtr layout)
    : m_layout(layout)
    , m_metadata(new Metadata())
{}

PointTable::~PointTable()
{}

PointLayoutPtr PointTable::layout()
{
    return m_layout;
}

const PointLayoutPtr PointTable::layout() const
{
    return m_layout;
}

MetadataNode PointTable::metadata()
{
    return m_metadata->getNode();
}

SpatialReference PointTable::spatialRef() const
{
    MetadataNode m = m_metadata->m_private.findChild("spatialreference");
    SpatialReference sref;
    sref.setWKT(m.value());
    return sref;
}

void PointTable::setSpatialRef(const SpatialReference& sref)
{
    MetadataNode mp = m_metadata->m_private;
    mp.addOrUpdate("spatialreference", sref.getRawWKT());
}



DefaultPointTable::DefaultPointTable()
    : PointTable()
    , m_blocks()
    , m_numPts(0)
{}

DefaultPointTable::DefaultPointTable(PointLayoutPtr layout)
    : PointTable(layout)
    , m_blocks()
    , m_numPts(0)
{}

DefaultPointTable::~DefaultPointTable()
{
    for (auto vi = m_blocks.begin(); vi != m_blocks.end(); ++vi)
        delete [] *vi;
}

PointId DefaultPointTable::addPoint()
{
    if (m_numPts % m_blockPtCnt == 0)
    {
        char *buf = new char[pointsToBytes(m_blockPtCnt)];
        m_blocks.push_back(buf);
    }
    return m_numPts++;
}

char *DefaultPointTable::getPoint(PointId idx)
{
    char *buf = m_blocks[idx / m_blockPtCnt];
    return buf + pointsToBytes(idx % m_blockPtCnt);
}

void DefaultPointTable::setField(
        const Dimension::Detail *d,
        PointId idx,
        const void *value)
{
    std::memcpy(getDimension(d, idx), value, d->size());
}

void DefaultPointTable::getField(
        const Dimension::Detail *d,
        PointId idx,
        void *value)
{
    std::memcpy(value, getDimension(d, idx), d->size());
}

} // namespace pdal

