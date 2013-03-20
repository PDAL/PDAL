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


void Cache::addToCache(boost::uint64_t blockPosition, const PointBuffer& data) const
{
    m_cache->insert(blockPosition, new PointBuffer(data));
    return;
}


std::vector<PointBuffer const*> Cache::lookup(boost::uint64_t pointPosition, boost::uint32_t count) const
{
    
    std::vector<PointBuffer const*> empty;
    std::vector<PointBuffer const*> output;
    
    boost::uint32_t startingBlockNumber = pointPosition / getCacheBlockSize();
    boost::uint32_t endingBlockNumber = (pointPosition + count - 1) / getCacheBlockSize();
    
    bool logOutput = log()->getLevel() > logDEBUG3;
    
    if (logOutput)
        log()->get(logDEBUG4) << "Checking for blocks from " << startingBlockNumber 
                              << ".." << endingBlockNumber << std::endl;
    
    for (boost::uint32_t i(startingBlockNumber); i <= endingBlockNumber; ++i)
    {
        if (logOutput)
            log()->get(logDEBUG4) << "Searching for block " << i << std::endl;
        PointBuffer const* block = m_cache->lookup(i);

        
        // Every block must be in the cache, or we return nothing
        if (!block)
        {
            if (logOutput)
                log()->get(logDEBUG4) << "Did not find block " << i << std::endl;
            return empty;
            
        }

        output.push_back(block);
    }
    
    if (logOutput)
    log()->get(logDEBUG2) << "found " << output.size() 
                                << " block(s) for position " 
                                << pointPosition << " with size " 
                                << count << std::endl;
    return output;
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


boost::uint64_t Cache::getNumPointsCached() const
{
    return m_cache->size() * getCacheBlockSize();
}

void Cache::updateStats(    boost::uint64_t numRead, 
                            boost::uint64_t numRequested) const
{
    m_numPointsRead += numRead;
    m_numPointsRequested += numRequested;
}

boost::uint32_t Cache::calculateNumberOfBlocks (boost::uint32_t CacheBlockSize, 
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

namespace cache {
    
IteratorBase::IteratorBase(pdal::filters::Cache const& filter, PointBuffer& buffer)
: m_cache_filter(filter)
, m_dimension_map(0) 
{

}

IteratorBase::~IteratorBase()
{
    if (m_dimension_map)
        delete m_dimension_map;
}

boost::uint32_t IteratorBase::copyCachedBlocks(     std::vector<PointBuffer const*> const& blocks, 
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
    for (ConstIterator i = blocks.begin(); i != blocks.end(); ++i)
    {
        PointBuffer const* b = *i;
        // if (!m_dimension_map)
            m_dimension_map = PointBuffer::mapDimensions(*b, data);
            
    
        boost::int32_t blockStartingPosition = (blockNumber) * m_cache_filter.getCacheBlockSize();
        boost::int32_t blockEndingPosition =   (blockNumber+1) * m_cache_filter.getCacheBlockSize();
    
        boost::int64_t blockHowMany = std::min(blockEndingPosition, readEndPosition) - currentPosition;

        boost::int64_t blockBufferStartingPosition =  currentPosition - blockStartingPosition ;

        bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
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

        PointBuffer::copyLikeDimensions(*b, data, 
                                        *m_dimension_map, 
                                        blockBufferStartingPosition, 
                                        userBufferStartingPosition, 
                                        blockHowMany);

        blockNumber++;
        numPointsCopied = numPointsCopied + blockHowMany;
        currentPosition = currentPosition + blockHowMany;
        userBufferStartingPosition = userBufferStartingPosition + blockHowMany;         
    }

    m_cache_filter.updateStats( data.getNumPoints(), data.getCapacity());  

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
        
            f->resetCache(num_blocks, buffer.getCapacity());        
        }
    else if (filter.getMaxCacheBlocks() == 1 && derive_blocks)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);
        
        boost::uint64_t total_count = f->getPrevStage().getNumPoints();
        boost::uint32_t num_blocks = f->calculateNumberOfBlocks(filter.getCacheBlockSize(), total_count);
        
        f->resetCache(num_blocks, filter.getCacheBlockSize());
    
    } 

    filter.log()->get(logDEBUG) << "create sequential iterator: creating cache of " 
                                << filter.getMaxCacheBlocks() << " blocks of capacity " 
                                << filter.getCacheBlockSize() << std::endl;
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
    boost::uint32_t cacheBlockSize = m_cache_filter.getCacheBlockSize();

    boost::uint64_t currentPointIndex = getIndex();
    
    boost::uint32_t blockNumber = currentPointIndex / cacheBlockSize;

    bool logOutput = m_cache_filter.log()->getLevel() > logDEBUG3;
    

    std::vector<PointBuffer const*> blocks = m_cache_filter.lookup(currentPointIndex, data.getCapacity());

    if (blocks.size())
    {
        m_cache_filter.log()->get(logDEBUG3) << "sequential read had cache hit for block " 
                                    << blockNumber << " with at point index " 
                                    << currentPointIndex << std::endl;
        
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
    m_cache_filter.updateStats(numRead, numRead);

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
    // If we're set to defaults, just cache the whole thing.
    if (filter.getMaxCacheBlocks() == 1 && filter.getCacheBlockSize() == 0)
        {
            filters::Cache* f = const_cast<filters::Cache*>(&filter);
        
            boost::uint64_t total_count = f->getPrevStage().getNumPoints();
            boost::uint32_t num_blocks = (total_count / filter.getMaxCacheBlocks()) + 1;
        
            f->resetCache(1, total_count);        
        }
    else if (filter.getMaxCacheBlocks() == 1 && filter.getCacheBlockSize() != 0)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);
        
        boost::uint64_t total_count = f->getPrevStage().getNumPoints();
        boost::uint32_t num_blocks = f->calculateNumberOfBlocks(filter.getCacheBlockSize(), total_count);
        
        f->resetCache(num_blocks, filter.getCacheBlockSize());
    
    } else if (filter.getCacheBlockSize() != 0 && filter.getMaxCacheBlocks() == 1)
    {
        filters::Cache* f = const_cast<filters::Cache*>(&filter);
        
        boost::uint64_t total_count = f->getPrevStage().getNumPoints();
        boost::uint32_t num_blocks = (total_count / filter.getMaxCacheBlocks()) + 1;
        
        f->resetCache(num_blocks, filter.getCacheBlockSize());
    
    } 


    filter.log()->get(logDEBUG) << "create random iterator: creating cache of " 
                                << filter.getMaxCacheBlocks() << " blocks of capacity " 
                                << filter.getCacheBlockSize() << std::endl;

	return;
}


boost::uint64_t Cache::seekImpl(boost::uint64_t position)
{
    boost::uint32_t blockPosition(0);
    if (position != 0)
    {
        blockPosition = m_cache_filter.getCacheBlockSize() / position;
    }

    std::vector<PointBuffer const*> blocks = m_cache_filter.lookup(position, 1);
    if (blocks.size())
    {
        if (position > m_cache_filter.getNumPoints())
            throw pdal_error("seeked past the end!");
        return position;
    } else 
    {
        return getPrevIterator().seek(position);
    }
}


boost::uint32_t Cache::readBufferImpl(PointBuffer& data)
{
    boost::uint32_t cacheBlockSize = m_cache_filter.getCacheBlockSize();
    boost::uint32_t numBlocks = m_cache_filter.getMaxCacheBlocks();

    boost::uint64_t currentPointIndex = getIndex();
    
    boost::uint32_t blockNumber(0);
    if (currentPointIndex != 0)
    {
        blockNumber = cacheBlockSize / currentPointIndex;
    }

    std::vector<PointBuffer const*> blocks = m_cache_filter.lookup(currentPointIndex, data.getCapacity());

    if (blocks.size())
    {
        m_cache_filter.log()->get(logDEBUG3) << "random read had cache hit for block " 
                                    << blockNumber << " with at point index " 
                                    << currentPointIndex << std::endl;
        
        // lookup will only return a list of blocks if things are 
        // entirely contained in the cache.
        boost::uint32_t copied(0);
        copied  = copyCachedBlocks(blocks, data, currentPointIndex);
        if (copied)
            return data.getNumPoints();
    }

    // We're cacheable if we're on the block boundary
    const bool isCacheable = (data.getCapacity() == cacheBlockSize) &&
                             (data.getNumPoints() == 0) &&
                             (currentPointIndex % cacheBlockSize == 0) &&
                             !blocks.size();
    if (isCacheable)
    {
        const boost::uint32_t numRead = getPrevIterator().read(data);
        
        if (numRead != cacheBlockSize)
        {
            throw pdal_error("We did not read the same number of points as the cache size!");
        }

        m_cache_filter.addToCache(blockNumber, data);
        m_cache_filter.updateStats(numRead, data.getCapacity());

        return numRead;
    }


    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);
    m_cache_filter.updateStats(numRead, numRead);

    return numRead;
}


}
} // iterators::random




}
} // namespaces
