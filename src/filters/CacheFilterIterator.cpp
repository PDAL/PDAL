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


CacheFilterSequentialIterator::CacheFilterSequentialIterator(const CacheFilter& filter)
    : libpc::FilterSequentialIterator(filter)
    , m_filter(filter)
{
    return;
}


boost::uint64_t CacheFilterSequentialIterator::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool CacheFilterSequentialIterator::atEndImpl() const
{
    return getPrevIterator().atEnd();
}


boost::uint32_t CacheFilterSequentialIterator::readImpl(PointBuffer& data)
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



CacheFilterRandomIterator::CacheFilterRandomIterator(const CacheFilter& filter)
    : libpc::FilterRandomIterator(filter)
    , m_filter(filter)
{
    return;
}


boost::uint64_t CacheFilterRandomIterator::seekImpl(boost::uint64_t count)
{
    return getPrevIterator().seek(count);
}


// BUG: this duplicates the code above
boost::uint32_t CacheFilterRandomIterator::readImpl(PointBuffer& data)
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

} } // namespaces
