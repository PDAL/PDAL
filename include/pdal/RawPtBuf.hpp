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

#include "pdal/pdal_internal.hpp"

namespace pdal
{

class RawPtBuf
{
public:
    virtual PointId addPoint() = 0;
    virtual char *getPoint(PointId idx) = 0;
    virtual void setField(Dimension::Detail *d, PointId idx,
        const void *value) = 0;
    virtual void getField(Dimension::Detail *d, PointId idx, void *value) = 0;
    virtual bool update(Dimension::DetailList& detail, Dimension::Detail *cur,
        const std::string& name) = 0;
};

/// This class provides a place to store the point data.
class DefaultRawPtBuf : public RawPtBuf
{
public:
    DefaultRawPtBuf() : m_numPts(0)
    {}

    ~DefaultRawPtBuf()
    {
        for (auto vi = m_blocks.begin(); vi != m_blocks.end(); ++vi)
            delete [] *vi;
    }

    PointId addPoint()
    {
        if (m_numPts % m_blockPtCnt == 0)
        {
            char *buf = new char[pointsToBytes(m_blockPtCnt)];
            m_blocks.push_back(buf);
        }
        return m_numPts++;
    }

    char *getPoint(PointId idx)
    {
        char *buf = m_blocks[idx / m_blockPtCnt];
        return buf + pointsToBytes(idx % m_blockPtCnt);
    }

    void setField(Dimension::Detail *d, PointId idx, const void *value)
       { memcpy(getDimension(d, idx), value, d->size()); }

    void getField(Dimension::Detail *d, PointId idx, void *value)
       { memcpy(value, getDimension(d, idx), d->size()); }

    bool update(Dimension::DetailList& detail, Dimension::Detail *cur,
        const std::string& name)
    {
        auto sorter = [this](const Dimension::Detail& d1,
                const Dimension::Detail& d2) -> bool
        {
            if (d1.size() > d2.size())
                return true;
            if (d1.size() < d2.size())
                return false;
            return d1.id() < d2.id();
        };

        if (m_numPts != 0)
            throw pdal_error("Can't update dimensions after points have "
                "been added.");

        int offset = 0;
        std::sort(detail.begin(), detail.end(), sorter);
        for (auto& d : detail)
        {
            d.setOffset(offset);
            offset += (int)d.size();
        }
        //NOTE - I tried forcing all points to be aligned on 8-byte boundaries
        // in case this would matter to the optimized memcpy, but it made
        // no difference.  No sense wasting space for no difference.
        m_pointSize = (size_t)offset;
        return true;
    }

private:
    std::vector<char *> m_blocks;
    point_count_t m_numPts;
    size_t m_pointSize;

    // The number of points in each memory block.
    static const point_count_t m_blockPtCnt = 65536;

    char *getDimension(Dimension::Detail *d, PointId idx)
        { return getPoint(idx) + d->offset(); }
    
    std::size_t pointsToBytes(point_count_t numPts)
        { return m_pointSize * numPts; }
};
typedef std::shared_ptr<RawPtBuf> RawPtBufPtr;

} // namespace pdal

