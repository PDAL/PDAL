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

#include <pdal/ArtifactManager.hpp>
#include <pdal/PointTable.hpp>

namespace pdal
{

BasePointTable::BasePointTable(PointLayout& layout) :
    m_metadata(new Metadata()), m_layoutRef(layout)
{}


BasePointTable::~BasePointTable()
{}


MetadataNode BasePointTable::privateMetadata(const std::string& name)
{
    MetadataNode mp = m_metadata->m_private;
    MetadataNode m = mp.findChild(name);
    if (!m.valid())
        m = mp.add(name);
    return m;
}


void BasePointTable::addSpatialReference(const SpatialReference& spatialRef)
{
    auto it = std::find(m_spatialRefs.begin(), m_spatialRefs.end(), spatialRef);

    // If not found, add to the beginning.
    if (it == m_spatialRefs.end())
        m_spatialRefs.push_front(spatialRef);
    // If not the first element, move the found element to the front.
    else if (it != m_spatialRefs.begin())
        m_spatialRefs.splice(m_spatialRefs.begin(), m_spatialRefs, it);
}


ArtifactManager& BasePointTable::artifactManager()
{
    if (!m_artifactManager)
        m_artifactManager.reset(new ArtifactManager);

    return *m_artifactManager;
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


RowPointTable::~RowPointTable()
{
    for (auto vi = m_blocks.begin(); vi != m_blocks.end(); ++vi)
        delete [] *vi;
}

PointId RowPointTable::addPoint()
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


char *RowPointTable::getPoint(PointId idx)
{
    char *buf = m_blocks[idx / m_blockPtCnt];
    return buf + pointsToBytes(idx % m_blockPtCnt);
}


MetadataNode BasePointTable::toMetadata() const
{
    return layout()->toMetadata();
}

} // namespace pdal

