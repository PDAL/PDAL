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

#include <libpc/filters/CacheFilter.hpp>
#include <libpc/filters/CacheFilterIterator.hpp>

#include <libpc/PointBufferCache.hpp>
#include <libpc/exceptions.hpp>

namespace libpc { namespace filters {


// cache block size is measured in Points, not bytes
CacheFilter::CacheFilter(const Stage& prevStage, boost::uint32_t maxCacheBlocks, boost::uint32_t cacheBlockSize)
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


boost::uint32_t CacheFilter::getCacheBlockSize() const
{
    return m_cacheBlockSize;
}


void CacheFilter::addToCache(boost::uint64_t pointIndex, const PointBuffer& data) const
{
    PointBuffer* block = new PointBuffer(data.getSchemaLayout(), m_cacheBlockSize);
    block->copyPointsFast(0, 0, data, m_cacheBlockSize);
    
    m_cache->insert(pointIndex, block);

    return;
}


const PointBuffer* CacheFilter::lookupInCache(boost::uint64_t pointIndex) const
{
    const boost::uint64_t blockNum = pointIndex / m_cacheBlockSize;

    const PointBuffer* block = m_cache->lookup(blockNum);

    return block;
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
    m_cache = new PointBufferCache(m_maxCacheBlocks);
}


boost::uint64_t CacheFilter::getNumPointsRequested() const
{
    return m_numPointsRequested;
}


boost::uint64_t CacheFilter::getNumPointsRead() const
{
    return m_numPointsRead;
}


void CacheFilter::updateStats(boost::uint64_t numRead, boost::uint64_t numRequested) const
{
    m_numPointsRead += numRead;
    m_numPointsRequested += numRequested;
}


libpc::SequentialIterator* CacheFilter::createSequentialIterator() const
{
    return new CacheFilterSequentialIterator(*this);
}


libpc::RandomIterator* CacheFilter::createRandomIterator() const
{
    return new CacheFilterRandomIterator(*this);
}


} } // namespaces
