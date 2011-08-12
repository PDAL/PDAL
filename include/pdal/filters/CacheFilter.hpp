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

#ifndef INCLUDED_FILTER_CACHEFILTER_HPP
#define INCLUDED_FILTER_CACHEFILTER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Filter.hpp>
//#include <pdal/FilterIterator.hpp>

namespace pdal {

class PointBufferCache;
class PointBuffer;
    
namespace filters {

//
// This is just a very simple MRU cache filter. It has the following constraints:
//   - up to 'numBlocks' will be cached
//   - we cache blocks of points, of size 'blockSize'
//   - we only cache full blocks
//   - we only cache on 'blockSize' boundaries
//   - we only look into the cache if 1 point is being requested
//
class PDAL_DLL CacheFilter : public Filter
{
public:
    SET_STAGE_NAME("filters.cache", "Cache Filter")

    CacheFilter(Stage& prevStage, const Options&);
    CacheFilter(Stage& prevStage, boost::uint32_t numBlocks, boost::uint32_t blockSize);
    ~CacheFilter();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    boost::uint32_t getCacheBlockSize() const;

    // this is const only because the m_cache itself is mutable
    void addToCache(boost::uint64_t pointIndex, const PointBuffer& data) const;

    const PointBuffer* lookupInCache(boost::uint64_t pointIndex) const;

    // clear cache (but leave cache params unchanged)
    void resetCache();

    // clear cache, and change cache params too
    void resetCache(boost::uint32_t numBlocks, boost::uint32_t blockSize);

    // number of points requested from this filter via read()
    boost::uint64_t getNumPointsRequested() const;

    // num points this filter read from the previous stage
    boost::uint64_t getNumPointsRead() const;

    void updateStats(boost::uint64_t numRead, boost::uint64_t numRequested) const;

    void getCacheStats(boost::uint64_t& numCacheLookupMisses,
                       boost::uint64_t& numCacheLookupHits,
                       boost::uint64_t& numCacheInsertMisses,
                       boost::uint64_t& numCacheInsertHits) const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator() const;
    pdal::StageRandomIterator* createRandomIterator() const;

private:
    // these are mutable to allow const-ness for updating stats
    // BUG: need to make thread-safe
    mutable boost::uint64_t m_numPointsRequested;
    mutable boost::uint64_t m_numPointsRead;

    // these is mutable to allow const-ness for updating stats
    // BUG: need to make thread-safe
    mutable PointBufferCache* m_cache;

    boost::uint32_t m_maxCacheBlocks;
    boost::uint32_t m_cacheBlockSize;

    CacheFilter& operator=(const CacheFilter&); // not implemented
    CacheFilter(const CacheFilter&); // not implemented
};


} } // namespaces

#endif
