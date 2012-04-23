/*
 * Copyright (c) 2010, Tim Day <timday@timday.com>
 * Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

// This PointBuffer-specific code derived from the templated code at
// http://www.bottlenose.demon.co.uk/article/lru.htm.  It is under an
// Internet Systems Consortium (ISC) license (an OSI-approved BSD-alike license).

#ifndef INCLUDED_PDAL_POINTBUFFERCACHE_HPP
#define INCLUDED_PDAL_POINTBUFFERCACHE_HPP

#include <pdal/pdal_internal.hpp>


#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/bimap.hpp>
#include <boost/bimap/list_of.hpp>
//#include <boost/bimap/set_of.hpp>
//#include <boost/function.hpp>
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

#include <pdal/PointBuffer.hpp>


namespace pdal
{


class PDAL_DLL PointBufferCache
{
public:

    typedef int dummy_type;

    // Bimap with key access on left view, key access
    // history on right view, and associated value.
    typedef boost::bimaps::bimap<
    boost::bimaps::set_of<boost::uint64_t>,
          boost::bimaps::list_of<dummy_type>,
          boost::bimaps::with_info<PointBuffer*>
          > cache_type;

    // Constuctor specifies the cached function and
    // the maximum number of records to be stored.
    PointBufferCache(size_t c)
        :_capacity(c)
        , m_numCacheLookupMisses(0)
        , m_numCacheLookupHits(0)
        , m_numCacheInsertMisses(0)
        , m_numCacheInsertHits(0)
    {
        assert(_capacity!=0);
    }

    ~PointBufferCache()
    {
        for (cache_type::left_iterator it =_cache.left.begin(); it != _cache.left.end(); ++it)
        {
            PointBuffer* data = it->info;
            delete data;
        }
        return;
    }

    PointBuffer* lookup(boost::uint64_t k)
    {
        // Attempt to find existing record
        const cache_type::left_iterator it =_cache.left.find(k);

        if (it==_cache.left.end())
        {
            // We don't have it:
            ++m_numCacheLookupMisses;
            return NULL;
        }
        else
        {
            // We do have it: update the access record view and return it
            _cache.right.relocate(
                _cache.right.end(),
                _cache.project_right(it)
            );
            ++m_numCacheLookupHits;
            return it->info;
        }
    }

    // When something is inserted into the cache, the cache
    // takes ownership (deletion responsibility) for it.
    PointBuffer* insert(boost::uint64_t k, PointBuffer* v)
    {
        // Attempt to find existing record
        const cache_type::left_iterator it =_cache.left.find(k);

        if (it==_cache.left.end())
        {
            // We don't have it: insert it
            insertx(k,v);
            ++m_numCacheInsertMisses;
            return v;
        }
        else
        {
            // We do have it: update the access record view and return it
            // note we don't support "replacing" the value (i.e. if v != it->info)
            _cache.right.relocate(
                _cache.right.end(),
                _cache.project_right(it)
            );
            assert(it->info == v);
            ++m_numCacheInsertHits;
            return it->info;
        }
    }

    // Obtain the cached keys, most recently used element
    // at head, least recently used at tail.
    // This method is provided purely to support testing.
    template <typename IT> void get_keys(IT dst) const
    {
        typename cache_type::right_const_reverse_iterator src
            =_cache.right.rbegin();
        while (src!=_cache.right.rend())
        {
            dst=(*src).second;
            ++src;
            ++dst;
        }
    }

    void getCacheStats(boost::uint64_t& numCacheLookupMisses,
                       boost::uint64_t& numCacheLookupHits,
                       boost::uint64_t& numCacheInsertMisses,
                       boost::uint64_t& numCacheInsertHits) const
    {
        numCacheLookupMisses = m_numCacheLookupMisses;
        numCacheLookupHits = m_numCacheLookupHits;
        numCacheInsertMisses = m_numCacheInsertMisses;
        numCacheInsertHits = m_numCacheInsertHits;
    }

private:

    void insertx(boost::uint64_t k, PointBuffer* v)
    {
        assert(_cache.size()<=_capacity);

        // If necessary, make space
        if (_cache.size()==_capacity)
        {
            // by purging the least-recently-used element

            cache_type::right_iterator iter =_cache.right.begin();
            PointBuffer *old = iter->info;
            delete old;

            _cache.right.erase(_cache.right.begin());
        }

        // Create a new record from the key, a dummy and the value
        _cache.insert(
            cache_type::value_type(
                k,0,v
            )
        );
    }

    const size_t _capacity;
    cache_type _cache;

    boost::uint64_t m_numCacheLookupMisses;
    boost::uint64_t m_numCacheLookupHits;
    boost::uint64_t m_numCacheInsertMisses;
    boost::uint64_t m_numCacheInsertHits;

    PointBufferCache& operator=(const PointBufferCache&); // not implemented
    PointBufferCache(const PointBufferCache&); // not implemented
};


} // namespace

#endif
