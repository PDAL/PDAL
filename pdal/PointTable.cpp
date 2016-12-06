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

MetadataNode BasePointTable::privateMetadata(const std::string& name)
{
    MetadataNode mp = m_metadata->m_private;
    MetadataNode m = mp.findChild(name);
    if (!m.valid())
        m = mp.add(name);
    return m;
}


void SimplePointTable::setFieldInternal(Dimension::Id id, PointId idx,
    const void *value)
{
    const Dimension::Detail *d = m_layoutRef.dimDetail(id);
    const char *src  = (const char *)value;
    char *dst = getDimension(d, idx);
    std::copy(src, src + d->size(), dst);
}


void SimplePointTable::getFieldInternal(Dimension::Id id, PointId idx,
    void *value) const
{
    const Dimension::Detail *d = m_layoutRef.dimDetail(id);
    const char *src = getDimension(d, idx);
    char *dst = (char *)value;
    std::copy(src, src + d->size(), dst);
}


PointTable::~PointTable()
{
    for (auto vi = m_blocks.begin(); vi != m_blocks.end(); ++vi)
        delete [] *vi;
}

PointId PointTable::addPoint()
{
    if (m_numPts % m_blockPtCnt == 0)
    {
        size_t size = pointsToBytes(m_blockPtCnt);
        char *buf = new char[size];
        memset(buf, 0, size);
        m_blocks.push_back(buf);
    }
    return m_numPts++;
}


char *PointTable::getPoint(PointId idx)
{
    char *buf = m_blocks[idx / m_blockPtCnt];
    return buf + pointsToBytes(idx % m_blockPtCnt);
}


MetadataNode BasePointTable::toMetadata() const
{
    const PointLayoutPtr l(layout());
    MetadataNode root;

    for (const auto& id : l->dims())
    {
        MetadataNode dim("dimensions");
        dim.add("name", l->dimName(id));
        Dimension::Type t = l->dimType(id);
        dim.add("type", Dimension::toName(Dimension::base(t)));
        dim.add("size", l->dimSize(id));
        root.addList(dim);
    }

    return root;
}

} // namespace pdal

