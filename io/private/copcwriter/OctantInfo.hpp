/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include "VoxelKey.hpp"
#include <pdal/PointView.hpp>

namespace pdal
{
namespace copcwriter
{

class OctantInfo
{
public:
    OctantInfo() : m_mustWrite(false)
    {}

    OctantInfo(const VoxelKey& key) : m_mustWrite(false)
        { m_key = key; }

    void movePoints(OctantInfo& oi)
    {
        PointViewPtr& source = oi.source();
        if (!source)
            return;
        if (!m_source)
            m_source = source;
        else
            m_source->append(*source);
        source = m_source->makeNew(); // Make an empty point view for the source.
    }

    PointViewPtr& source()
    {
        return m_source;
    }

    size_t numPoints() const
    {
        return m_source ? m_source->size() : 0;
    }

    VoxelKey key() const
        { return m_key; }
    void setKey(VoxelKey k)
        { m_key = k; }
    bool mustWrite() const
        { return m_mustWrite; }
    void setMustWrite(bool mustWrite)
        { m_mustWrite = mustWrite; }

private:
    PointViewPtr m_source;
    VoxelKey m_key;
    bool m_mustWrite;
};

} // namespace copcwriter
} // namespace pdal
