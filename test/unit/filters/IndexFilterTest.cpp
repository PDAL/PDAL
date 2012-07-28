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

#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/filters/Index.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(IndexFilterTest)

BOOST_AUTO_TEST_CASE(test_3d)
{
    pdal::Option option("filename", Support::datapath("pipeline/pipeline_index.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();

    pdal::filters::Index const* filter = static_cast<pdal::filters::Index const*>(reader.getManager().getStage());
    pdal::Options opt = filter->getOptions();
    // std::cout << "filter ops: " << opt << std::endl;

    const pdal::Schema& schema = filter->getSchema();
    pdal::PointBuffer data(schema, 20);

    pdal::StageSequentialIterator* it = filter->createSequentialIterator(data);

    boost::uint32_t numRead = it->read(data);
    BOOST_CHECK(numRead == 20);
    
#ifdef PDAL_HAVE_FLANN   
    pdal::filters::iterators::sequential::Index* iter = dynamic_cast<pdal::filters::iterators::sequential::Index*>(it);
    
    unsigned k = 8;
    
    // If the query distance is 0, just return the k nearest neighbors
    std::vector<boost::uint32_t> ids = iter->query(636199, 849238, 428.05, 0.0, k);
    
    BOOST_CHECK_EQUAL(ids.size(), k);
    BOOST_CHECK_EQUAL(ids[0], 8u);
    BOOST_CHECK_EQUAL(ids[4], 10u);

    std::vector<boost::uint32_t> dist_ids = iter->query(636199, 849238, 428.05, 100.0, k);
    
    BOOST_CHECK_EQUAL(dist_ids.size(), 3u);
    BOOST_CHECK_EQUAL(dist_ids[0], 8u);
#endif

    return;
}

BOOST_AUTO_TEST_CASE(test_2d)
{
    pdal::Option option("filename", Support::datapath("pipeline/pipeline_index.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);



    reader.initialize();
    pdal::filters::Index* filter = static_cast<pdal::filters::Index*>(reader.getManager().getStage());
    
    filter->setDimensions(2);

    // std::cout << "filter ops: " << opt << std::endl;

    const pdal::Schema& schema = filter->getSchema();
    pdal::PointBuffer data(schema, 20);

    pdal::StageSequentialIterator* it = filter->createSequentialIterator(data);

    boost::uint32_t numRead = it->read(data);
    BOOST_CHECK(numRead == 20);
    
#ifdef PDAL_HAVE_FLANN   
    pdal::filters::iterators::sequential::Index* iter = dynamic_cast<pdal::filters::iterators::sequential::Index*>(it);
    
    unsigned k = 8;
    
    // If the query distance is 0, just return the k nearest neighbors
    std::vector<boost::uint32_t> ids = iter->query(636199, 849238, 0.0, 0.0, k);
    
    BOOST_CHECK_EQUAL(ids.size(), k);
    BOOST_CHECK_EQUAL(ids[0], 8u);
    BOOST_CHECK_EQUAL(ids[4], 10u);

    std::vector<boost::uint32_t> dist_ids = iter->query(636199, 849238, 0.0, 100.0, k);
    
    BOOST_CHECK_EQUAL(dist_ids.size(), 3u);
    BOOST_CHECK_EQUAL(dist_ids[0], 8u);
#endif

    return;
}




BOOST_AUTO_TEST_SUITE_END()
