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

#include <map>
#include <memory>
#include <vector>

#include "pdal/Dimension.hpp"
#include "pdal/PointLayout.hpp"
#include "pdal/Metadata.hpp"

namespace pdal
{

namespace plang
{
    class BufferedInvocation;
}

class PointTable
{
public:
    PointTable();
    PointTable(PointLayoutPtr layout);
    virtual ~PointTable();

    // Point data operations.
    virtual PointId addPoint() = 0;
    virtual char *getPoint(PointId idx) = 0;
    virtual void setField(const Dimension::Detail *d, PointId idx,
        const void *value) = 0;
    virtual void getField(const Dimension::Detail *d, PointId idx,
        void *value) = 0;

    // Layout operations.
    PointLayoutPtr layout();
    const PointLayoutPtr layout() const;

    // Metadata operations.
    MetadataNode metadata();
    SpatialReference spatialRef() const;
    void setSpatialRef(const SpatialReference& sref);

protected:
    PointLayoutPtr m_layout;
    MetadataPtr m_metadata;
};




// This provides a context for processing a set of points and allows the library
// to be used to process multiple point sets simultaneously.
class DefaultPointTable : public PointTable
{
    friend class PointView;
    friend class plang::BufferedInvocation;
private:
    // Point storage.
    std::vector<char *> m_blocks;
    point_count_t m_numPts;

public:
    DefaultPointTable();
    DefaultPointTable(PointLayoutPtr layout);
    virtual ~DefaultPointTable();

private:
    // Point data operations.
    virtual PointId addPoint();
    virtual char *getPoint(PointId idx);
    virtual void setField(const Dimension::Detail *d, PointId idx,
        const void *value);
    virtual void getField(const Dimension::Detail *d, PointId idx,
        void *value);

    // The number of points in each memory block.
    static const point_count_t m_blockPtCnt = 65536;

    char *getDimension(const Dimension::Detail *d, PointId idx)
        { return getPoint(idx) + d->offset(); }

    std::size_t pointsToBytes(point_count_t numPts)
        { return m_layout->pointSize() * numPts; }
};

typedef std::shared_ptr<PointTable> PointTablePtr;

} //namespace

