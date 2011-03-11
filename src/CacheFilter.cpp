/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "libpc/CacheFilter.hpp"

namespace libpc
{


CacheFilter::CacheFilter(Stage& prevStage)
    : Filter(prevStage)
    , m_currentPointIndex(0)
    , m_storedPointIndex(0)
    , m_storedPointData(NULL)
    , m_numPointsRequested(0)
    , m_numPointsRead(0)
{
    return;
}


CacheFilter::~CacheFilter()
{
    delete m_storedPointData;
}


const std::string& CacheFilter::getName() const
{
    static std::string name("Cache Filter");
    return name;
}


boost::uint64_t CacheFilter::getCurrentPointIndex() const
{
    return m_currentPointIndex;
}


void CacheFilter::seekToPoint(boost::uint64_t index)
{
    m_currentPointIndex = index;
    m_prevStage.seekToPoint(index);
}


boost::uint64_t CacheFilter::getNumPointsRequested() const
{
    return m_numPointsRequested;
}


boost::uint64_t CacheFilter::getNumPointsRead() const
{
    return m_numPointsRead;
}


boost::uint32_t CacheFilter::readBuffer(PointData& data)
{
    if (data.getNumPoints() != 1)
    {
        const boost::uint32_t numRead = m_prevStage.read(data);
        m_currentPointIndex += numRead;
        return numRead;
    }

    m_numPointsRequested += data.getNumPoints();

    boost::uint64_t pointToRead = getCurrentPointIndex();

    // do we meet the cache-checking criteria?
    if (m_storedPointData != NULL)
    {
        // do we have the point we want in the cache?
        if (pointToRead >= m_storedPointIndex && 
            pointToRead < m_storedPointIndex + m_storedPointData->getNumPoints())
        {
            // the cache has the data we want, copy out the point we want
            boost::uint32_t index = (boost::uint32_t)(pointToRead - m_storedPointIndex);
            data.copyPointFast(0, index, *m_storedPointData);
            m_currentPointIndex += 1;
            return 1;
        }
    }

    // not cached, so read a full block and replace cache with that new block

    delete m_storedPointData;

    int chunkSize = 1024;
    m_storedPointData = new PointData(data.getSchemaLayout(), chunkSize);

    m_prevStage.read(*m_storedPointData);
    m_numPointsRead += chunkSize;

    data.copyPointFast(0, 0, *m_storedPointData);

    m_storedPointIndex = getCurrentPointIndex();

    m_currentPointIndex += 1;

    return 1;
}

}
