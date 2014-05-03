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

#include "Support.hpp"
#include <pdal/StageIterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Mosaic.hpp>

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(MosaicFilterTest)

BOOST_AUTO_TEST_CASE(basic_test)
{

    Bounds<double> bounds1(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader1(bounds1, 100, pdal::drivers::faux::Reader::Constant);

    Bounds<double> bounds2(100.0, 100.0, 100.0, 200.0, 200.0, 200.0);
    pdal::drivers::faux::Reader reader2(bounds2, 100, pdal::drivers::faux::Reader::Constant);

    Bounds<double> bounds3(200.0, 200.0, 200.0, 300.0, 300.0, 300.0);
    pdal::drivers::faux::Reader reader3(bounds3, 100, pdal::drivers::faux::Reader::Constant);

    std::vector<Stage*> vec;
    vec.push_back(&reader1);
    vec.push_back(&reader2);
    vec.push_back(&reader3);

    Options options;
    // Option debug("debug", "true", "debug");
    // options.add(debug);
    // 
    // Option verbose("verbose", 7, "verbose");
    // options.add(verbose);

    pdal::filters::Mosaic mosaic(options);
    mosaic.setInput(vec);
    BOOST_CHECK(mosaic.getDescription() == "Mosaic Filter");
    mosaic.initialize();

    const Schema& schema = mosaic.getSchema();

    PointBuffer data(schema, 300);

    StageSequentialIterator* iter = mosaic.createSequentialIterator(data);
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK(numRead == 300);


    Schema const& buffer_schema = data.getSchema();
    Dimension const& dimX = buffer_schema.getDimension("X");
    Dimension const& dimY = buffer_schema.getDimension("Y");
    Dimension const& dimZ = buffer_schema.getDimension("Z");
    Dimension const& dimTime = buffer_schema.getDimension("Time");

    for (boost::uint32_t i=0; i<10; i++)
    {
        double x = data.getField<double>(dimX, i);
        double y = data.getField<double>(dimY, i);
        double z = data.getField<double>(dimZ, i);
        boost::uint64_t t = data.getField<boost::uint64_t>(dimTime, i);

        if (i<100)
        {
            BOOST_CHECK_CLOSE(x, 0.0, 0.001);
            BOOST_CHECK_CLOSE(y, 0.0, 0.001);
            BOOST_CHECK_CLOSE(z, 0.0, 0.001);
        }
        else if (i<200)
        {
            BOOST_CHECK_CLOSE(x, 100.0, 0.001);
            BOOST_CHECK_CLOSE(y, 100.0, 0.001);
            BOOST_CHECK_CLOSE(z, 100.0, 0.001);
        }
        else
        {
            BOOST_CHECK_CLOSE(x, 200.0, 0.001);
            BOOST_CHECK_CLOSE(y, 200.0, 0.001);
            BOOST_CHECK_CLOSE(z, 200.0, 0.001);

        }
        BOOST_CHECK_EQUAL(t, i % 100);
    }

    delete iter;
}


// pipeline/mosaic.xml uses filters.programmable, which depends on Python
#ifdef PDAL_HAVE_PYTHON
BOOST_AUTO_TEST_CASE(pipeline_mosaic)
{

    pdal::PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(Support::datapath("pipeline/mosaic.xml"));
    Stage* stage = manager.getStage();
    BOOST_CHECK(stage != NULL);
    stage->initialize();

    const Schema& schema = stage->getSchema();
    PointBuffer data(schema, 2*1065);
    StageSequentialIterator* iter = stage->createSequentialIterator(data);
    boost::uint32_t np = iter->read(data);
    BOOST_CHECK_EQUAL(np, 2*1065u);


    Dimension const& dimX = data.getSchema().getDimension("X1");
    // std::cout << data.getSchema() << std::endl;
    for (boost::uint32_t i=0; i<1065; i++)
    {
        boost::int32_t x = data.getField<boost::int32_t>(dimX, i);
        BOOST_CHECK_EQUAL(x, 314);
    }

    // boost::int32_t x2 = data.getField<boost::int32_t>(dimX, 1066);
    // BOOST_CHECK_EQUAL(x2, 0); // Past 1066, is the other data set due to mosaic filter
    //         
    delete iter;
}
#endif

BOOST_AUTO_TEST_SUITE_END()
