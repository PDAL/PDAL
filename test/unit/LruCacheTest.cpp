/*
 * Copyright (c) 2010, Tim Day <timday@timday.com>
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

// This code is from http://www.bottlenose.demon.co.uk/article/lru.htm.  It is
// under a Internet Systems Consortium (ISC) license (an OSI-approved BSD-alike license).

#include <boost/test/unit_test.hpp>

#include "libpc/LruCache.hpp"

using namespace libpc;

static size_t count_evaluations=0; 
 
// Dummy function we want to cache 
std::string fn(const std::string& s) 
{ 
  ++count_evaluations; 
  std::string r; 
  std::copy(s.rbegin(),s.rend(),std::back_inserter(r)); 
  return r; 
} 

BOOST_AUTO_TEST_SUITE(LruCacheTest)

BOOST_AUTO_TEST_CASE(test1)
{
    count_evaluations=0; 
 
    LruCache<std::string,std::string> lru(fn,5); 
 
    // Some initial accesses to prime state 
    BOOST_CHECK_EQUAL(lru("first"),"tsrif"); 
    BOOST_CHECK_EQUAL(lru("second"),"dnoces"); 
    BOOST_CHECK_EQUAL(lru("third"),"driht"); 
    BOOST_CHECK_EQUAL(lru("fourth"),"htruof"); 
    BOOST_CHECK_EQUAL(lru("fifth"),"htfif"); 
    BOOST_CHECK_EQUAL(count_evaluations,5); 
    BOOST_CHECK_EQUAL(lru("sixth"),"htxis"); 
    BOOST_CHECK_EQUAL(count_evaluations,6); 

    // This should be retrieved from cache 
    BOOST_CHECK_EQUAL(lru("second"),"dnoces"); 
    BOOST_CHECK_EQUAL(count_evaluations,6); 

    // This will have been evicted 
    BOOST_CHECK_EQUAL(lru("first"),"tsrif"); 
    BOOST_CHECK_EQUAL(count_evaluations,7); 
 
    // Cache contents by access time 
    // (most recent to least recent) 
    // should now be: 
    // first,second,sixth,fifth,fourth 
    { 
        std::vector<std::string> expected; 
        expected.push_back("first"); 
        expected.push_back("second"); 
        expected.push_back("sixth"); 
        expected.push_back("fifth"); 
        expected.push_back("fourth"); 
        std::vector<std::string> actual; 
        lru.get_keys(std::back_inserter(actual)); 
        BOOST_CHECK(actual==expected); 
    } 

    // So check fourth is retrieved 
    BOOST_CHECK_EQUAL(lru("fourth"),"htruof"); 
    BOOST_CHECK_EQUAL(count_evaluations,7); 

    // That will have moved up "fourth" to the head 
    // so this will evict fifth 
    BOOST_CHECK_EQUAL(lru("seventh"),"htneves"); 
    BOOST_CHECK_EQUAL(count_evaluations,8); 

    // Check fifth was evicted as expected 
    BOOST_CHECK_EQUAL(lru("fifth"),"htfif"); 
    BOOST_CHECK_EQUAL(count_evaluations,9);     
    
    return;
}


class MyItem
{
public:
    MyItem(int index, double data) : m_index(index), m_data(data) {}
    MyItem(const MyItem& other)
    {
        m_index = other.m_index;
        m_data = other.m_data;
    }

    MyItem& operator=(MyItem const& rhs)
    {
        if (&rhs != this)
        {
            m_index = rhs.m_index;
            m_data = rhs.m_data;
        }
        return *this;
    }

    bool operator==(const MyItem& other) const
    {
        if (m_index == other.m_index) return true;
        return false;
    }

    int m_index;
    double m_data;
};

// Dummy function we want to cache 
int cache_func(MyItem* item)
{ 
    ++count_evaluations; 
    return item->m_index;
}

BOOST_AUTO_TEST_CASE(test2)
{
    MyItem* item1 = new MyItem(1,11);
    MyItem* item11 = new MyItem(1,11);
    MyItem* item2 = new MyItem(2,22);
    MyItem* item3 = new MyItem(3,33);
    MyItem* item33 = new MyItem(3,33);

    count_evaluations = 0;

    LruCache<MyItem*,int> lru(cache_func,2);

    // bunch of insertions/lookups
    lru(item1);
    lru(item1);
    lru(item2);
    lru(item2);
    lru(item1);
    lru(item1);
    lru(item3);
    lru(item3);
    lru(item1);
     
    BOOST_CHECK(count_evaluations == 3);

    { 
        std::vector<MyItem*> actual; 
        lru.get_keys(std::back_inserter(actual)); 
        BOOST_CHECK(actual.size() == 2); 
        BOOST_CHECK(actual[0]->m_index == 1); 
        BOOST_CHECK(actual[1]->m_index == 3); 
    }

    lru(item33);
    lru(item11);
     
    BOOST_CHECK(count_evaluations == 5);

    { 
        std::vector<MyItem*> actual; 
        lru.get_keys(std::back_inserter(actual)); 
        BOOST_CHECK(actual.size() == 2); 
        BOOST_CHECK(actual[0]->m_index == 1); 
        BOOST_CHECK(actual[1]->m_index == 3); 
    }

    return;
}


BOOST_AUTO_TEST_SUITE_END()
