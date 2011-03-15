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

#include <libpc/CacheFilter.hpp>

#include <libpc/PointDataCache.hpp>

namespace libpc
{


// cache block size is measured in Points, not bytes
CacheFilter::CacheFilter(Stage& prevStage, boost::uint32_t maxCacheBlocks, boost::uint32_t cacheBlockSize)
    : Filter(prevStage)
    , m_numPointsRequested(0)
    , m_numPointsRead(0)
    , m_cache(NULL)
    , m_maxCacheBlocks(maxCacheBlocks)
    , m_cacheBlockSize(cacheBlockSize)
{
    resetCache();
    return;
}


CacheFilter::~CacheFilter()
{
    delete m_cache;
}


const std::string& CacheFilter::getName() const
{
    static std::string name("Cache Filter");
    return name;
}


void CacheFilter::getCacheStats(boost::uint64_t& numCacheLookupMisses,
                                boost::uint64_t& numCacheLookupHits,
                                boost::uint64_t& numCacheInsertMisses,
                                boost::uint64_t& numCacheInsertHits) const
{
    m_cache->getCacheStats(numCacheLookupMisses,
                           numCacheLookupHits,
                           numCacheInsertMisses,
                           numCacheInsertHits);
}


void CacheFilter::resetCache(boost::uint32_t maxCacheBlocks, boost::uint32_t cacheBlockSize)
{
    m_maxCacheBlocks = maxCacheBlocks;
    m_cacheBlockSize = cacheBlockSize;
    resetCache();
}


void CacheFilter::resetCache()
{
    delete m_cache;
    m_cache = new PointDataCache(m_maxCacheBlocks);
}


void CacheFilter::seekToPoint(boost::uint64_t index)
{
    setCurrentPointIndex(index);
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
    const boost::uint64_t currentPointIndex = getCurrentPointIndex();

    // for now, we only read from the cache if they are asking for one point
    // (this avoids the problem of an N-point request needing more than one
    // cached block to satisfy it)
    if (data.getNumPoints() != 1)
    {
        const boost::uint32_t numRead = m_prevStage.read(data);

        // if they asked for a full block and we got a full block,
        // and the block we got is properly aligned and not already cached,
        // then let's cache it!
        if (data.getNumPoints() == m_cacheBlockSize && numRead == m_cacheBlockSize && 
            (currentPointIndex % m_cacheBlockSize == 0) &&
            m_cache->lookup(currentPointIndex) == NULL)
        {
            PointData* block = new PointData(data.getSchemaLayout(), m_cacheBlockSize);
            block->copyPointsFast(0, 0, data, m_cacheBlockSize);
            m_cache->insert(currentPointIndex, block);
        }

        incrementCurrentPointIndex(numRead);

        m_numPointsRead += data.getNumPoints();
        m_numPointsRequested += data.getNumPoints();

        return numRead;
    }

    // they asked for just one point -- first, check Mister Cache
    const boost::uint64_t blockNum = currentPointIndex / m_cacheBlockSize;
    PointData* block = m_cache->lookup(blockNum);
    if (block != NULL)
    {
        // A hit! A palpable hit!
        data.copyPointFast(0,  currentPointIndex % m_cacheBlockSize, *block);
        
        m_numPointsRead += 0;
        m_numPointsRequested += 1;
        incrementCurrentPointIndex(1);

        m_prevStage.seekToPoint(getCurrentPointIndex());
        return 1;
    }

    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = m_prevStage.read(data);
    m_numPointsRead += numRead;
    m_numPointsRequested += numRead;
    incrementCurrentPointIndex(numRead);

    return numRead;
}

}
