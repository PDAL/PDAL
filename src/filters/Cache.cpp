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

#include <pdal/filters/Cache.hpp>

#include <pdal/filters/PointBufferCache.hpp>

namespace pdal
{
namespace filters
{


Cache::Cache(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_numPointsRequested(0)
    , m_numPointsRead(0)
    , m_cache(NULL)
    , m_maxCacheBlocks(options.getValueOrDefault<boost::uint32_t>("max_cache_blocks", 1))
    , m_cacheBlockSize(options.getValueOrDefault<boost::uint32_t>("cache_block_size", 0))
{
    return;
}


// cache block size is measured in Points, not bytes
Cache::Cache(   Stage& prevStage, 
                boost::uint32_t maxCacheBlocks, 
                boost::uint32_t cacheBlockSize)
    : Filter(prevStage, Options::none())
    , m_numPointsRequested(0)
    , m_numPointsRead(0)
    , m_cache(NULL)
    , m_maxCacheBlocks(maxCacheBlocks)
    , m_cacheBlockSize(cacheBlockSize)
{
    return;
}


Cache::~Cache()
{
    delete m_cache;
}


void Cache::initialize()
{
    Filter::initialize();
	
    resetCache();
    return;
}


Options Cache::getDefaultOptions()
{
    Options options;
    Option max_cache_blocks("max_cache_blocks", 1);
    Option cache_block_size("cache_block_size", 32768);
    options.add(max_cache_blocks);
    options.add(cache_block_size);
    return options;
}


void Cache::addToCache(boost::uint64_t pointIndex, const PointBuffer& data) const
{
    PointBuffer* block = new PointBuffer(data.getSchema(), m_cacheBlockSize);
    block->copyPointsFast(0, 0, data, m_cacheBlockSize);

    m_cache->insert(pointIndex, block);

    return;
}


const PointBuffer* Cache::lookupInCache(boost::uint64_t pointIndex) const
{
    const boost::uint64_t blockNum = pointIndex / m_cacheBlockSize;

    const PointBuffer* block = m_cache->lookup(blockNum);

    return block;
}


void Cache::getCacheStats(boost::uint64_t& numCacheLookupMisses,
                          boost::uint64_t& numCacheLookupHits,
                          boost::uint64_t& numCacheInsertMisses,
                          boost::uint64_t& numCacheInsertHits) const
{
    m_cache->getCacheStats(numCacheLookupMisses,
                           numCacheLookupHits,
                           numCacheInsertMisses,
                           numCacheInsertHits);
}


void Cache::resetCache(boost::uint32_t maxCacheBlocks, boost::uint32_t cacheBlockSize)
{
    m_maxCacheBlocks = maxCacheBlocks;
    m_cacheBlockSize = cacheBlockSize;
    resetCache();
}


void Cache::resetCache()
{
    delete m_cache;
    m_cache = new PointBufferCache(m_maxCacheBlocks);
}


boost::uint64_t Cache::getNumPointsRequested() const
{
    return m_numPointsRequested;
}


boost::uint64_t Cache::getNumPointsRead() const
{
    return m_numPointsRead;
}


void Cache::updateStats(    boost::uint64_t numRead, 
                            boost::uint64_t numRequested) const
{
    m_numPointsRead += numRead;
    m_numPointsRequested += numRequested;
}


pdal::StageSequentialIterator* Cache::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Cache(*this, buffer);
}


pdal::StageRandomIterator* Cache::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::random::Cache(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Cache::Cache(const pdal::filters::Cache& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_filter(filter)
{
	// reset the cache to the point count if no block size 
	// was specified. Yes we're doing dirt here, but nuking the cache at
	// the creation of an iterator shouldn't be too catastrophic
	if (filter.getCacheBlockSize() == 0)
	{
		filters::Cache* f = const_cast<filters::Cache*>(&filter);
		f->resetCache(f->getMaxCacheBlocks(), f->getPrevStage().getNumPoints());
	}
    return;
}


boost::uint64_t Cache::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Cache::atEndImpl() const
{
    return getPrevIterator().atEnd();
}


boost::uint32_t Cache::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t cacheBlockSize = m_filter.getCacheBlockSize();

    const boost::uint64_t currentPointIndex = getIndex();

    // for now, we only read from the cache if they are asking for one point
    // (this avoids the problem of an N-point request needing more than one
    // cached block to satisfy it)
    if (data.getCapacity() != 1)
    {
        const boost::uint32_t numRead = getPrevIterator().read(data);

        // if they asked for a full block and we got a full block,
        // and the block we got is properly aligned and not already cached,
        // then let's cache it!
        const bool isCacheable = (data.getCapacity() == cacheBlockSize) &&
                                 (numRead == cacheBlockSize) &&
                                 (currentPointIndex % cacheBlockSize == 0);
        if (isCacheable && (m_filter.lookupInCache(currentPointIndex) == NULL))
        {
            m_filter.addToCache(currentPointIndex, data);
        }

        m_filter.updateStats(numRead, data.getCapacity());

        return numRead;
    }

    // they asked for just one point -- first, check Mister Cache
    const PointBuffer* block = m_filter.lookupInCache(currentPointIndex);
    if (block != NULL)
    {
        // A hit! A palpable hit!
        data.copyPointFast(0,  currentPointIndex % cacheBlockSize, *block);

        m_filter.updateStats(0, 1);

        return 1;
    }

    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);
    m_filter.updateStats(numRead, numRead);

    return numRead;
}
}
} // iterators::sequential


namespace iterators
{
namespace random
{



Cache::Cache(const pdal::filters::Cache& filter, PointBuffer& buffer)
    : pdal::FilterRandomIterator(filter, buffer)
    , m_filter(filter)
{
    if (filter.getCacheBlockSize() == 0)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);
        f->resetCache(f->getMaxCacheBlocks(), f->getPrevStage().getNumPoints());
    }
	return;
}


boost::uint64_t Cache::seekImpl(boost::uint64_t count)
{
    return getPrevIterator().seek(count);
}


// BUG: this duplicates the code above
boost::uint32_t Cache::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t cacheBlockSize = m_filter.getCacheBlockSize();

    const boost::uint64_t currentPointIndex = getIndex();

    // for now, we only read from the cache if they are asking for one point
    // (this avoids the problem of an N-point request needing more than one
    // cached block to satisfy it)
    if (data.getCapacity() != 1)
    {
        const boost::uint32_t numRead = getPrevIterator().read(data);

        // if they asked for a full block and we got a full block,
        // and the block we got is properly aligned and not already cached,
        // then let's cache it!
        const bool isCacheable = (data.getCapacity() == cacheBlockSize) &&
                                 (numRead == cacheBlockSize) &&
                                 (currentPointIndex % cacheBlockSize == 0);
        if (isCacheable && (m_filter.lookupInCache(currentPointIndex) == NULL))
        {
            m_filter.addToCache(currentPointIndex, data);
        }

        m_filter.updateStats(numRead, data.getCapacity());

        return numRead;
    }

    // they asked for just one point -- first, check Mister Cache
    const PointBuffer* block = m_filter.lookupInCache(currentPointIndex);
    if (block != NULL)
    {
        PointBuffer::copyLikeDimensions(*block, data, currentPointIndex, 0, 1);

        return 1;
    }

    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);
    m_filter.updateStats(numRead, numRead);

    return numRead;
}


}
} // iterators::random




}
} // namespaces
