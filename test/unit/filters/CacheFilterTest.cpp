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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Cache.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(CacheFilterTest)


BOOST_AUTO_TEST_CASE(CacheFilterTest_test_options)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader(srcBounds, 10000, pdal::drivers::faux::Reader::Constant);
    
    boost::uint32_t cache_size(355);
    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 9, "");
    pdal::Option max_cache_blocks("max_cache_blocks", 3);
    pdal::Option cache_block_size("cache_block_size", cache_size);
    options.add(max_cache_blocks);
    options.add(cache_block_size);
    // options.add(debug);
    // options.add(verbose);


    pdal::filters::Cache cache(reader, options);

    BOOST_CHECK(cache.getDescription() == "Cache Filter");
    cache.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer dataBig(schema, cache_size);
    PointBuffer dataSmall(schema, 293);

    StageSequentialIterator* iter1 = cache.createSequentialIterator(dataBig);

    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), 0); // No data in there yet
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), 0);

    iter1->read(dataBig);
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), cache_size);

    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), cache_size);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), cache_size);

    iter1->read(dataBig);
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), cache_size* 2); 
    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), cache_size * 1);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), cache_size * 2);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), cache_size * 2);

    iter1->read(dataBig);
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), cache_size* 3); 

    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), cache_size*2);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), cache_size * 3);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), cache_size * 3);
    

    StageSequentialIterator* iter2 = cache.createSequentialIterator(dataSmall);

    boost::uint32_t skip_position(648);
    iter2->skip(skip_position);
    iter2->read(dataSmall);
    BOOST_CHECK_EQUAL(dataSmall.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), skip_position);

    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), cache_size*2+skip_position);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), cache_size*3);

    delete iter1;
    delete iter2;

    return;
}



BOOST_AUTO_TEST_CASE(CacheFilterTest_test_use_counts)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader(srcBounds, 10000, pdal::drivers::faux::Reader::Constant);

    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 9, "");
    // pdal::Option max_cache_blocks("max_cache_blocks", 1);
    // pdal::Option cache_block_size("cache_block_size", cache_size);
    // options.add(max_cache_blocks);
    // options.add(cache_block_size);
    // options.add(debug);
    // options.add(verbose);


    pdal::filters::Cache cache(reader, options);

    BOOST_CHECK_EQUAL(cache.getDescription(), "Cache Filter");
    cache.initialize();

    const Schema& schema = reader.getSchema();
    
    boost::uint32_t buffer_size(1024);
    PointBuffer dataBig(schema, buffer_size);
    PointBuffer dataSmall(schema, 293);

    StageSequentialIterator* iter1 = cache.createSequentialIterator(dataBig);

    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), 0); // No data in there yet
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), 0);

    iter1->read(dataBig);
    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), buffer_size);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), buffer_size);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), buffer_size);

    iter1->read(dataBig);
    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), 1024);
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), buffer_size*2);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), buffer_size*2);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), buffer_size*2);
    
    BOOST_CHECK_EQUAL(cache.getNumPointsCached(), buffer_size* 2);
    
    StageSequentialIterator* iter2 = cache.createSequentialIterator(dataSmall);

    boost::uint32_t skip_position(1045);
    iter2->skip(skip_position);
    boost::uint32_t numRead = iter2->read(dataSmall);
    BOOST_CHECK_EQUAL(numRead, dataSmall.getCapacity());
    BOOST_CHECK_EQUAL(dataSmall.getField<boost::uint64_t>(dataSmall.getSchema().getDimension("Time"), 0), skip_position);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(),buffer_size*2+dataSmall.getCapacity());
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), buffer_size*2);

    delete iter1;
    delete iter2;

    return;
}



BOOST_AUTO_TEST_CASE(CacheFilterTest_test_random)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    
    boost::uint32_t numPoints(10000);
    pdal::drivers::faux::Reader reader(srcBounds, numPoints, pdal::drivers::faux::Reader::Constant);

    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 9, "");
    // pdal::Option max_cache_blocks("max_cache_blocks", 1);
    // pdal::Option cache_block_size("cache_block_size", cache_size);
    // options.add(max_cache_blocks);
    // options.add(cache_block_size);
    // options.add(debug);
    // options.add(verbose);

    pdal::filters::Cache cache(reader, options);

    BOOST_CHECK_EQUAL(cache.getDescription(), "Cache Filter");
    cache.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer dataBig(schema, reader.getNumPoints());
    PointBuffer dataSmall(schema, 1);

    StageSequentialIterator* sequential = cache.createSequentialIterator(dataBig);
    StageRandomIterator* random = cache.createRandomIterator(dataSmall);

    //BOOST_CHECK(cache.getIndex() == 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), 0);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), 0);

    sequential->read(dataBig);
    BOOST_CHECK_EQUAL(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0), 0);

    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), numPoints);
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), numPoints);
    
    boost::uint32_t seek_position(42);
    random->seek(seek_position);
    random->read(dataSmall);
    BOOST_CHECK_EQUAL(dataSmall.getField<boost::uint64_t>(dataSmall.getSchema().getDimension("Time"), 0), seek_position);    
    BOOST_CHECK_EQUAL(cache.getNumPointsRequested(), numPoints + dataSmall.getCapacity());
    BOOST_CHECK_EQUAL(cache.getNumPointsRead(), numPoints);
    
    delete sequential;
    delete random;

    return;
}


BOOST_AUTO_TEST_CASE(test_two_iters_with_cache)
{
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");

    boost::uint32_t block_size(355);
    boost::uint32_t num_blocks(3);
    boost::uint32_t num_points(1065);
    
    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 9, "");
    pdal::Option max_cache_blocks("max_cache_blocks", num_blocks);
    pdal::Option cache_block_size("cache_block_size", block_size);
    options.add(max_cache_blocks);
    options.add(cache_block_size);
    // options.add(debug);
    // options.add(verbose);


    pdal::filters::Cache cache(reader, options);

    cache.initialize();
    

    BOOST_CHECK_EQUAL(reader.getNumPoints(), num_points);
    BOOST_CHECK_EQUAL(block_size * num_blocks, num_points);

    BOOST_CHECK_EQUAL(cache.getNumPoints(), num_points);

    const Schema& schema = cache.getSchema();

    PointBuffer data(schema, block_size);

    boost::uint32_t numRead;
    
    {
        pdal::StageSequentialIterator* iter = cache.createSequentialIterator(data);
        BOOST_CHECK_EQUAL(iter->getIndex(), 0);
    
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, block_size);
        BOOST_CHECK_EQUAL(iter->getIndex(), block_size);
    
        Support::check_p0_p1_p2(data);
    
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, block_size);
        BOOST_CHECK_EQUAL(iter->getIndex(), block_size*2);
    
        Support::check_p355_p356_p357(data);
    
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, block_size);
        BOOST_CHECK_EQUAL(iter->getIndex(), num_points);
            
        Support::check_p710_p711_p712(data);
            
        delete iter;
    }

    {
        pdal::StageRandomIterator* iter = cache.createRandomIterator(data);
        BOOST_CHECK_EQUAL(iter->getIndex(), 0);
   
        // read the middle third
        iter->seek(355);
        BOOST_CHECK_EQUAL(iter->getIndex(), 355);
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iter->getIndex(), 710);
   
        Support::check_p355_p356_p357(data);
        
        // read the first third
        iter->seek(0);
        BOOST_CHECK_EQUAL(iter->getIndex(), 0);
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iter->getIndex(), 355);
        
        Support::check_p0_p1_p2(data);
        
        // read the first third again
        iter->seek(0);
        BOOST_CHECK_EQUAL(iter->getIndex(), 0);
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iter->getIndex(), 355);
        
        Support::check_p0_p1_p2(data);
        
        // read the last third
        iter->seek(710);
        BOOST_CHECK_EQUAL(iter->getIndex(), 710);
        numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iter->getIndex(), 1065);
        
        Support::check_p710_p711_p712(data);
   
        delete iter;
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
