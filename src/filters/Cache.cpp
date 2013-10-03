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
Cache::Cache(Stage& prevStage,
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


void Cache::addToCache(boost::uint64_t blockPosition, const PointBuffer& data) const
{
    m_cache->insert(blockPosition, new PointBuffer(data));
    return;
}

bool Cache::isCached(boost::uint64_t pointPosition, boost::uint32_t count) const
{
    boost::uint32_t startingBlockNumber = pointPosition / getCacheBlockSize();
    boost::uint32_t endingBlockNumber = (pointPosition + count - 1) / getCacheBlockSize();

    if (startingBlockNumber == endingBlockNumber)
    {
        PointBuffer const* block = m_cache->lookup(startingBlockNumber);
        if (!block)
            return false;
    }
    else
    {
        for (boost::uint32_t i(startingBlockNumber); i <= endingBlockNumber; ++i)
        {
            PointBuffer const* block = m_cache->lookup(i);
            // Every block must be in the cache, or we return nothing
            if (!block)
            {
                return false;

            }
        }
    }
    return true;
}

void Cache::lookup(boost::uint64_t pointPosition, boost::uint32_t count) const
{

    m_blocks.clear();

    boost::uint32_t startingBlockNumber = pointPosition / getCacheBlockSize();
    boost::uint32_t endingBlockNumber = (pointPosition + count - 1) / getCacheBlockSize();

#ifdef DEBUG    // log output slows things down a lot when doing per-point lookups
    bool logOutput = log()->getLevel() > logDEBUG3;

    if (logOutput)
        log()->get(logDEBUG4) << "Checking for blocks from " << startingBlockNumber
                              << ".." << endingBlockNumber << std::endl;
#endif

    if (startingBlockNumber == endingBlockNumber)
    {
        PointBuffer const* block = m_cache->lookup(startingBlockNumber);
        if (!block)
        {
            m_blocks.clear();
            return;
        }

        m_blocks.push_back(block);
    }
    else
    {
        for (boost::uint32_t i(startingBlockNumber); i <= endingBlockNumber; ++i)
        {

#ifdef DEBUG
            if (logOutput)
                log()->get(logDEBUG4) << "Searching for block " << i << std::endl;
#endif

            PointBuffer const* block = m_cache->lookup(i);


            // Every block must be in the cache, or we return nothing
            if (!block)
            {
#ifdef DEBUG
                if (logOutput)
                    log()->get(logDEBUG4) << "Did not find block " << i << std::endl;
#endif
                m_blocks.clear();
                return;

            }

            m_blocks.push_back(block);
        }

    }


#ifdef DEBUG

    if (logOutput)
        log()->get(logDEBUG2) << "found " << m_blocks.size()
                              << " block(s) for position "
                              << pointPosition << " with size "
                              << count << std::endl;
#endif
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

Metadata Cache::toMetadata() const
{
    Metadata metadata;


    metadata.addMetadata<boost::uint32_t>("max_cache_blocks",
                                          getMaxCacheBlocks(),
                                          "Number of cache entries");
    metadata.addMetadata<boost::uint32_t>("cache_block_size",
                                          getCacheBlockSize(),
                                          "Cache entry size");

    boost::uint64_t lookup_misses(0);
    boost::uint64_t lookup_hits(0);
    boost::uint64_t insert_misses(0);
    boost::uint64_t insert_hits(0);

    getCacheStats(lookup_misses, lookup_hits, insert_misses, insert_hits);
    metadata.addMetadata<boost::uint64_t>("lookup_misses",
                                          lookup_misses,
                                          "Cache lookup misses");

    metadata.addMetadata<boost::uint64_t>("lookup_hits",
                                          lookup_hits,
                                          "Cache lookup hits");

    metadata.addMetadata<boost::uint64_t>("insert_misses",
                                          insert_misses,
                                          "Cache insert misses");

    metadata.addMetadata<boost::uint64_t>("insert_hits",
                                          insert_hits,
                                          "Cache insert hits");

    return metadata;
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


boost::uint64_t Cache::getNumPointsCached() const
{
    return m_cache->size() * getCacheBlockSize();
}

void Cache::updateStats(boost::uint64_t numRead,
                        boost::uint64_t numRequested) const
{
    m_numPointsRead += numRead;
    m_numPointsRequested += numRequested;
}

boost::uint32_t Cache::calculateNumberOfBlocks(boost::uint32_t CacheBlockSize,
        boost::uint64_t totalNumberOfPoints) const
{

    boost::uint64_t num_blocks = totalNumberOfPoints / static_cast<boost::uint64_t>(CacheBlockSize);

    boost::uint64_t extra = totalNumberOfPoints % static_cast<boost::uint64_t>(CacheBlockSize);
    if (extra)
    {
        num_blocks = num_blocks + 1;
    }

    return num_blocks;
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

namespace cache
{

IteratorBase::IteratorBase(pdal::filters::Cache const& filter, PointBuffer& buffer)
    : m_cache_filter(filter)
    , m_mapped_buffer(&buffer)
{

}

IteratorBase::~IteratorBase()
{
    if (m_dimension_maps.size())
    {
        typedef std::map<PointBuffer const*, pointbuffer::DimensionMap const*>::iterator MapIterator;
        for (MapIterator i = m_dimension_maps.begin(); i != m_dimension_maps.end(); ++i)
        {
            delete i->second;
        }
    }
}

boost::uint32_t IteratorBase::copyCachedBlocks(std::vector<PointBuffer const*> const& blocks,
        PointBuffer& data,
        boost::uint32_t currentPointIndex) const
{

    if (!blocks.size()) return 0;

    boost::uint32_t cacheBlockSize = m_cache_filter.getCacheBlockSize();

    boost::uint32_t blockNumber = currentPointIndex / cacheBlockSize;

    typedef std::vector<PointBuffer const*>::const_iterator ConstIterator;
    boost::int32_t readEndPosition(currentPointIndex + data.getCapacity());
    boost::int32_t currentPosition(currentPointIndex);
    boost::int32_t userBufferStartingPosition(0);
    boost::uint32_t startingBlockNumber = currentPointIndex / m_cache_filter.getCacheBlockSize();
    boost::uint32_t endingBlockNumber = (currentPointIndex + data.getCapacity()) / m_cache_filter.getCacheBlockSize();

    boost::uint32_t numPointsCopied(0);

#ifdef DEBUG
    bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
#endif
    if (m_mapped_buffer != &data)
    {
        m_dimension_maps.clear();
        m_mapped_buffer = &data;

#ifdef DEBUG
        if (logOutput)
            m_cache_filter.log()->get(logDEBUG2) << "Wiping dimension map because mapped buffer != current buffer!" << std::endl;
#endif

    }
    assert(m_mapped_buffer == &data);

    for (ConstIterator i = blocks.begin(); i != blocks.end(); ++i)
    {
        PointBuffer const* b = *i;
        pointbuffer::DimensionMap const* dim_map(0);

        typedef std::map<PointBuffer const*, pointbuffer::DimensionMap const*>::iterator MapIterator;
        MapIterator it = m_dimension_maps.find(b);
        if (it == m_dimension_maps.end())
        {
            pointbuffer::DimensionMap* d = PointBuffer::mapDimensions(*b, *m_mapped_buffer);
            std::pair<PointBuffer const*, pointbuffer::DimensionMap const*> p(b, d);
            m_dimension_maps.insert(p);
            dim_map = d;
#ifdef DEBUG
            if (logOutput)
                m_cache_filter.log()->get(logDEBUG2) << "Inserting dimension map entry for block" << std::endl;
#endif
        }
        else
        {
            dim_map = it->second;
        }



        boost::int32_t blockStartingPosition = (blockNumber) * m_cache_filter.getCacheBlockSize();
        boost::int32_t blockEndingPosition = (blockNumber+1) * m_cache_filter.getCacheBlockSize();

        boost::int64_t blockHowMany = std::min(blockEndingPosition, readEndPosition) - currentPosition;

        boost::int64_t blockBufferStartingPosition =  currentPosition - blockStartingPosition ;

#ifdef DEBUG
        if (logOutput)
        {
            m_cache_filter.log()->get(logDEBUG4) << "readEndPosition: " << readEndPosition << std::endl;
            m_cache_filter.log()->get(logDEBUG4) << "blockStartingPosition: " << blockStartingPosition
                                                 << " blockEndingPosition: " << blockEndingPosition << std::endl;
            m_cache_filter.log()->get(logDEBUG4) << "blockBufferStartingPosition: " << blockBufferStartingPosition
                                                 << " blockHowMany: " << blockHowMany << std::endl;
            m_cache_filter.log()->get(logDEBUG4) << "currentPosition: " << currentPosition
                                                 << " userBufferStartingPosition: " << userBufferStartingPosition << std::endl;
        }
#endif

        // This is dirty.
        if (data.getSchema().getByteSize() == b->getSchema().getByteSize())
        {

            data.copyPointsFast(userBufferStartingPosition, blockBufferStartingPosition, *b, blockHowMany);
        }
        else
        {
            PointBuffer::copyLikeDimensions(*b, data,
                                            *dim_map,
                                            blockBufferStartingPosition,
                                            userBufferStartingPosition,
                                            blockHowMany);
        }

        blockNumber++;
        numPointsCopied = numPointsCopied + blockHowMany;
        currentPosition = currentPosition + blockHowMany;
        userBufferStartingPosition = userBufferStartingPosition + blockHowMany;
    }

    m_cache_filter.updateStats(data.getNumPoints(), data.getCapacity());

    data.setNumPoints(numPointsCopied);


    return data.getNumPoints();
}

} // namespace cache

namespace sequential
{


Cache::Cache(const pdal::filters::Cache& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , cache::IteratorBase(filter, buffer)
{
    // reset the cache to the point count if no block size
    // was specified. Yes we're doing dirt here, but nuking the cache at
    // the creation of an iterator shouldn't be too catastrophic
    // If we're set to defaults, just cache the whole thing.
    bool derive_blocks = filter.getOptions().getValueOrDefault<bool>("derive_blocks", false);
    if (filter.getMaxCacheBlocks() == 1 && filter.getCacheBlockSize() == 0)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);

        boost::uint64_t total_count = f->getPrevStage().getNumPoints();
        boost::uint32_t num_blocks = (total_count / filter.getMaxCacheBlocks()) + 1;

        f->resetCache(1, buffer.getCapacity());
    }
    else if (filter.getMaxCacheBlocks() == 1 && derive_blocks)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);

        boost::uint64_t total_count = f->getPrevStage().getNumPoints();
        boost::uint32_t num_blocks = f->calculateNumberOfBlocks(filter.getCacheBlockSize(), total_count);

        f->resetCache(num_blocks, filter.getCacheBlockSize());

    }

#ifdef DEBUG
    bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
    if (logOutput)
        filter.log()->get(logDEBUG) << "create sequential iterator: creating cache of "
                                    << filter.getMaxCacheBlocks() << " blocks of capacity "
                                    << filter.getCacheBlockSize() << std::endl;
#endif
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

void Cache::readBufferEndImpl(PointBuffer& buffer)
{
    pdal::Metadata& metadata = buffer.getMetadataRef();
    pdal::Metadata stats = m_cache_filter.toMetadata();
    stats.setName(m_cache_filter.getName());
    metadata.setMetadata(stats);

}

boost::uint32_t Cache::readBufferImpl(PointBuffer& data)
{
    boost::uint32_t cacheBlockSize = m_cache_filter.getCacheBlockSize();

    boost::uint64_t currentPointIndex = getIndex();

    boost::uint32_t blockNumber = currentPointIndex / cacheBlockSize;

#ifdef DEBUG
    bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
#endif

    m_cache_filter.lookup(currentPointIndex, data.getCapacity());
    std::vector<PointBuffer const*> const& blocks = m_cache_filter.getCachedBlocks();

    if (blocks.size())
    {
#ifdef DEBUG
        if (logOutput)
            m_cache_filter.log()->get(logDEBUG3) << "sequential read had cache hit for block "
                                                 << blockNumber << " with at point index "
                                                 << currentPointIndex << std::endl;
#endif
        // lookup will only return a list of blocks if things are
        // entirely contained in the cache.
        boost::uint32_t copied = copyCachedBlocks(blocks, data, currentPointIndex);
        if (copied != 0)
        {
            return data.getNumPoints();
        }
    }


    const bool isCacheable = (data.getCapacity() == cacheBlockSize) &&
                             (currentPointIndex % cacheBlockSize == 0) &&
                             !blocks.size();

    if (isCacheable)
    {

        const boost::uint32_t numRead = getPrevIterator().read(data);

        m_cache_filter.addToCache(blockNumber, data);
        m_cache_filter.updateStats(numRead, data.getCapacity());

        return numRead;
    }



    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);
    m_cache_filter.updateStats(numRead, data.getCapacity());

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
    , cache::IteratorBase(filter, buffer)
{

#ifdef DEBUG
    bool logOutput = filter.log()->getLevel() > logDEBUG3;

    if (logOutput)
        filter.log()->get(logDEBUG) << "create random iterator: creating cache of "
                                    << filter.getMaxCacheBlocks() << " blocks of capacity "
                                    << filter.getCacheBlockSize() << std::endl;
#endif
    return;
}


boost::uint64_t Cache::seekImpl(boost::uint64_t position)
{
    bool bCached = m_cache_filter.isCached(position, 1);
    if (bCached)
    {
        if (position > m_cache_filter.getNumPoints())
            throw invalid_seek_error("seeked past the end!");
        return position;
    }
    else
    {
        return getPrevIterator().seek(position);
    }
}


boost::uint32_t Cache::readBufferImpl(PointBuffer& data)
{
    boost::uint32_t cacheBlockSize = m_cache_filter.getCacheBlockSize();
    boost::uint32_t numBlocks = m_cache_filter.getMaxCacheBlocks();

    boost::uint64_t currentPointIndex = getIndex();


#ifdef DEBUG
    bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
#endif

    m_cache_filter.lookup(currentPointIndex, data.getCapacity());
    std::vector<PointBuffer const*> const& blocks = m_cache_filter.getCachedBlocks();
    if (blocks.size())
    {
#ifdef DEBUG
        if (logOutput)

            m_cache_filter.log()->get(logDEBUG3) << "random read had cache hit for block with at point index "
                                                 << currentPointIndex << std::endl;
#endif
        // lookup will only return a list of blocks if things are
        // entirely contained in the cache.
        boost::uint32_t copied(0);
        copied  = copyCachedBlocks(blocks, data, currentPointIndex);
        if (copied)
            return data.getNumPoints();
    }

    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);


    return numRead;
}


}
} // iterators::random




}
} // namespaces
