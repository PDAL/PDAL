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

#include <pdal/PointBuffer.hpp>
#include <pdal/filters/PointBufferCache.hpp>

using namespace pdal;


BOOST_AUTO_TEST_SUITE(PointBufferCacheTest)

BOOST_AUTO_TEST_CASE(test1)
{
    Schema schema;

    Dimension d1("X", dimension::SignedInteger, 4);
    schema.appendDimension(d1);

    PointBuffer* item0 = new PointBuffer(schema, 10);
    pdal::Dimension const& dimX0 = item0->getSchema().getDimension("X");
    PointBuffer* item1 = new PointBuffer(schema, 10);
    pdal::Dimension const& dimX1 = item1->getSchema().getDimension("X");

    PointBuffer* item2 = new PointBuffer(schema, 10);
    pdal::Dimension const& dimX2 = item2->getSchema().getDimension("X");
    //PointBuffer* item3 = new PointBuffer(schema, 10);
    PointBuffer* item4 = new PointBuffer(schema, 10);
    pdal::Dimension const& dimX4 = item4->getSchema().getDimension("X");
    //PointBuffer* item5 = new PointBuffer(schema, 10);

    // write the data into the buffer
    for (int i=0; i<10; i++)
    {
        item0->setField(dimX0, i, i);
        item1->setField(dimX1, i, i+10);
        item2->setField(dimX2, i, i+20);
        //item3->setField(i, 0, i+30);
        item4->setField(dimX4, i, i+40);
        //item5->setField(i, 0, i+50);
    }

    PointBufferCache lru(2);

    lru.insert(0, item0);         // insert miss
    lru.insert(10, item1);        // insert miss
    lru.insert(20, item2);        // insert miss

    BOOST_CHECK(lru.lookup(0) == NULL);      // lookup miss
    BOOST_CHECK(lru.lookup(10) == item1);    // lookup hit
    BOOST_CHECK(lru.lookup(20) == item2);    // lookup hit

    {
        std::vector<boost::uint64_t> actual;
        lru.get_keys(std::back_inserter(actual));
        BOOST_CHECK(actual.size() == 2);
        BOOST_CHECK(actual[0] == 20);
        BOOST_CHECK(actual[1] == 10);
    }

    lru.insert(40,item4);        // insert miss

    BOOST_CHECK(lru.lookup(0) == NULL);    // lookup miss
    BOOST_CHECK(lru.lookup(10) == NULL);   // lookup miss
    BOOST_CHECK(lru.lookup(20) == item2);  // lookup hit
    BOOST_CHECK(lru.lookup(40) == item4);  // lookup hit
    BOOST_CHECK(lru.lookup(40) == item4);  // lookup hit

    boost::uint64_t lookupHits, lookupMisses, insertHits, insertMisses;
    lru.getCacheStats(lookupMisses, lookupHits, insertMisses, insertHits);
    BOOST_CHECK(lookupMisses == 3);
    BOOST_CHECK(lookupHits == 5);
    BOOST_CHECK(insertMisses == 4);
    BOOST_CHECK(insertHits == 0);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
