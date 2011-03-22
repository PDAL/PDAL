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


CacheFilterIterator::CacheFilterIterator(const CacheFilter& filter)
    : libpc::FilterIterator(filter)
    , m_stageAsDerived(filter)
{
    return;
}


void CacheFilterIterator::seekToPoint(boost::uint64_t index)
{
    setCurrentPointIndex(index);
    getPrevIterator().seekToPoint(index);
}


boost::uint32_t CacheFilterIterator::readBuffer(PointBuffer& data)
{
    CacheFilter& filter = const_cast<CacheFilter&>(m_stageAsDerived);       // BUG BUG BUG

    const boost::uint64_t currentPointIndex = getCurrentPointIndex();

    // for now, we only read from the cache if they are asking for one point
    // (this avoids the problem of an N-point request needing more than one
    // cached block to satisfy it)
    if (data.getCapacity() != 1)
    {
        const boost::uint32_t numRead = getPrevIterator().read(data);

        // if they asked for a full block and we got a full block,
        // and the block we got is properly aligned and not already cached,
        // then let's cache it!
        if (data.getCapacity() == filter.m_cacheBlockSize && numRead == filter.m_cacheBlockSize && 
            (currentPointIndex % filter.m_cacheBlockSize == 0) &&
            filter.m_cache->lookup(currentPointIndex) == NULL)
        {
            PointBuffer* block = new PointBuffer(data.getSchemaLayout(), filter.m_cacheBlockSize);
            block->copyPointsFast(0, 0, data, filter.m_cacheBlockSize);
            filter.m_cache->insert(currentPointIndex, block);
        }

        incrementCurrentPointIndex(numRead);

        filter.m_numPointsRead += numRead;
        filter.m_numPointsRequested += data.getCapacity();

        return numRead;
    }

    // they asked for just one point -- first, check Mister Cache
    const boost::uint64_t blockNum = currentPointIndex / filter.m_cacheBlockSize;
    PointBuffer* block = filter.m_cache->lookup(blockNum);
    if (block != NULL)
    {
        // A hit! A palpable hit!
        data.copyPointFast(0,  currentPointIndex % filter.m_cacheBlockSize, *block);
        
        filter.m_numPointsRead += 0;
        filter.m_numPointsRequested += 1;
        incrementCurrentPointIndex(1);

        getPrevIterator().seekToPoint(getCurrentPointIndex());
        return 1;
    }

    // Not in the cache, so do a normal read :-(
    const boost::uint32_t numRead = getPrevIterator().read(data);
    filter.m_numPointsRead += numRead;
    filter.m_numPointsRequested += numRead;
    incrementCurrentPointIndex(numRead);

    return numRead;
}

} } // namespaces
