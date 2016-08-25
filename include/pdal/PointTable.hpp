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

#include <set>
#include <vector>

#include "pdal/SpatialReference.hpp"
#include "pdal/Dimension.hpp"
#include "pdal/PointContainer.hpp"
#include "pdal/PointLayout.hpp"
#include "pdal/Metadata.hpp"

namespace pdal
{

class PDAL_DLL BasePointTable : public PointContainer
{
    friend class PointView;

protected:
    BasePointTable(PointLayout& layout) : m_metadata(new Metadata()),
        m_layoutRef(layout)
    {}

public:
    virtual ~BasePointTable()
        {}

    // Layout operations.
    virtual PointLayoutPtr layout() const
        { return &m_layoutRef; }

    // Metadata operations.
    MetadataNode metadata()
        { return m_metadata->getNode(); }
    virtual void finalize()
        { m_layoutRef.finalize(); }
    void setSpatialReference(const SpatialReference& srs)
    {
        clearSpatialReferences();
        addSpatialReference(srs);
    }
    void clearSpatialReferences()
        { m_spatialRefs.clear(); }
    void addSpatialReference(const SpatialReference& srs)
        { m_spatialRefs.insert(srs); }
    bool spatialReferenceUnique() const
        { return m_spatialRefs.size() == 1; }
    SpatialReference spatialReference() const
    {
        return spatialReferenceUnique() ? anySpatialReference() :
            SpatialReference();
    }
    SpatialReference anySpatialReference() const
    {
        return m_spatialRefs.size() ?
            *m_spatialRefs.begin() : SpatialReference();
    }
    virtual bool supportsView() const
        { return false; }
    MetadataNode privateMetadata(const std::string& name);
    MetadataNode toMetadata() const;

private:
    // Point data operations.
    virtual PointId addPoint() = 0;

protected:
    virtual char *getPoint(PointId idx) = 0;

protected:
    MetadataPtr m_metadata;
    std::set<SpatialReference> m_spatialRefs;
    PointLayout& m_layoutRef;
};
typedef BasePointTable& PointTableRef;
typedef BasePointTable const & ConstPointTableRef;

class PDAL_DLL SimplePointTable : public BasePointTable
{

protected:
    SimplePointTable(PointLayout& layout) : BasePointTable(layout)
        {}

protected:
    std::size_t pointsToBytes(point_count_t numPts) const
        { return m_layoutRef.pointSize() * numPts; }

private:
    virtual void setFieldInternal(Dimension::Id id, PointId idx,
        const void *value);
    virtual void getFieldInternal(Dimension::Id id, PointId idx,
        void *value) const;

    // The number of points in each memory block.
    char *getDimension(const Dimension::Detail *d, PointId idx)
        { return getPoint(idx) + d->offset(); }

    const char *getDimension(const Dimension::Detail *d, PointId idx) const
    {
        SimplePointTable *ncThis = const_cast<SimplePointTable *>(this);
        return ncThis->getPoint(idx) + d->offset();
    }
};

// This provides a context for processing a set of points and allows the library
// to be used to process multiple point sets simultaneously.
class PDAL_DLL PointTable : public SimplePointTable
{
private:
    // Point storage.
    std::vector<char *> m_blocks;
    point_count_t m_numPts;
    static const point_count_t m_blockPtCnt = 65536;

public:
    PointTable() : SimplePointTable(m_layout), m_numPts(0)
        {}
    virtual ~PointTable();
    virtual bool supportsView() const
        { return true; }

protected:
    virtual char *getPoint(PointId idx);

private:
    // Point data operations.
    virtual PointId addPoint();

    PointLayout m_layout;
};

/// A StreamPointTable must provide storage for point data up to its capacity.
/// It must implement getPoint() which returns a pointer to a buffer of
/// sufficient size to contain a point's data.  The minimum size required
/// is constant and can be determined by calling pointsToBytes(1) in the
/// finalize() method.
class PDAL_DLL StreamPointTable : public SimplePointTable
{
protected:
    StreamPointTable(PointLayout& layout) : SimplePointTable(layout)
    {}

public:
    /// Called when a new point should be added.  Probably a no-op for
    /// streaming.
    virtual PointId addPoint()
    { return 0; }
    /// Called when execute() is started.  Typically used to set buffer size
    /// when all dimensions are known.
    virtual void finalize()
    {}
    /// Called when the contents of StreamPointTable have been consumed and
    /// the point data will be potentially overwritten.
    virtual void reset()
    {}
    virtual point_count_t capacity() const = 0;
};

class PDAL_DLL FixedPointTable : public StreamPointTable
{
public:
    FixedPointTable(point_count_t capacity) : StreamPointTable(m_layout),
        m_capacity(capacity)
    {}

    virtual void finalize()
    {
        if (!m_layout.finalized())
        {
            BasePointTable::finalize();
            m_buf.resize(pointsToBytes(m_capacity + 1));
        }
    }

    point_count_t capacity() const
        { return m_capacity; }
protected:
    virtual char *getPoint(PointId idx)
        { return m_buf.data() + pointsToBytes(idx); }

private:
    std::vector<char> m_buf;
    point_count_t m_capacity;
    PointLayout m_layout;
};

} //namespace

