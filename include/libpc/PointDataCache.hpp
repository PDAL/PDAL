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

// This PointData-specific code derived from the templated code at
// http://www.bottlenose.demon.co.uk/article/lru.htm.  It is under an
// Internet Systems Consortium (ISC) license (an OSI-approved BSD-alike license).

#ifndef INCLUDED_LIBPC_POINTDATACACHE_HPP
#define INCLUDED_LIBPC_POINTDATACACHE_HPP


#ifdef _MSC_VER
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/bimap.hpp>
#include <boost/bimap/list_of.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/function.hpp>
#ifdef _MSC_VER
#  pragma warning(pop)
#endif

namespace libpc
{

class PointData;


class PointDataCache
{
public:

    typedef int dummy_type;

    // Bimap with key access on left view, key access
    // history on right view, and associated value.
    typedef boost::bimaps::bimap<
    boost::bimaps::set_of<boost::uint32_t>,
          boost::bimaps::list_of<dummy_type>,
          boost::bimaps::with_info<PointData*>
          > cache_type;

    // Constuctor specifies the cached function and
    // the maximum number of records to be stored.
    PointDataCache(size_t c)
        :_capacity(c)
    {
        assert(_capacity!=0);
    }

    ~PointDataCache()
    {
        for (cache_type::left_iterator it =_cache.left.begin(); it != _cache.left.end(); ++it)
        {
           PointData* data = it->info;
           delete data;
        }
        return;
    }

    PointData* lookup(boost::uint32_t k)
    {
        // Attempt to find existing record
        const cache_type::left_iterator it =_cache.left.find(k);

        if (it==_cache.left.end())
        {
            // We don't have it:
            return NULL;
        }
        else
        {
            // We do have it:
            return it->info;
        }
    }

    // Obtain value of the cached function for k
    PointData* insert(boost::uint32_t k, PointData* v)
    {
        // Attempt to find existing record
        const cache_type::left_iterator it =_cache.left.find(k);

        if (it==_cache.left.end())
        {
            // We don't have it: insert it
            insertx(k,v);
            return v;
        }
        else
        {
            // We do have it:
            // Update the access record view.
            _cache.right.relocate(
                _cache.right.end(),
                _cache.project_right(it)
            );

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

private:

    void insertx(boost::uint32_t k, PointData* v)
    {
        assert(_cache.size()<=_capacity);

        // If necessary, make space
        if (_cache.size()==_capacity)
        {
            // by purging the least-recently-used element
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

    PointDataCache& operator=(const PointDataCache&); // not implemented
    PointDataCache(const PointDataCache&); // not implemented
};


} // namespace

#endif
