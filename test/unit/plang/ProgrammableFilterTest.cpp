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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include "Support.hpp"
#include <boost/test/unit_test.hpp>

#include <pdal/filters/Programmable.hpp>
#include <pdal/filters/Programmable.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>
#include <pdal/drivers/pipeline/Reader.hpp>


#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>

BOOST_AUTO_TEST_SUITE(ProgrammableFilterTest)

using namespace pdal;


BOOST_AUTO_TEST_CASE(ProgrammableFilterTest_test1)
{
    Bounds<double> bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    pdal::drivers::faux::Reader reader(bounds, 10, pdal::drivers::faux::Reader::Ramp);

    const pdal::Option source("source",
                              "import numpy as np\n"
                              "def myfunc(ins,outs):\n"
                              "  X = ins['X']\n"
                              "  Y = ins['Y']\n"
                              "  Z = ins['Z']\n"
                              "  #print ins['X']\n"
                              "  X = X + 10.0\n"
                              "  # Y: leave as-is, don't export back out\n"
                              "  # Z: goofiness to make it a numpy array of a constant\n"
                              "  Z = np.zeros(X.size) + 3.14\n"
                              "  outs['X'] = X\n"
                              "  #print outs['X']\n"
                              "  outs['Z'] = Z\n"
                              "  return True\n"
                             );
    const pdal::Option module("module", "MyModule");
    const pdal::Option function("function", "myfunc");
    pdal::Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    pdal::filters::Programmable filter(opts);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Programmable Filter");
    pdal::drivers::faux::Writer writer;
    writer.setInput(&filter);
    writer.initialize();

    boost::uint64_t numWritten = writer.write(10);

    BOOST_CHECK(numWritten == 10);

    const double minX = writer.getMinX();
    const double maxX = writer.getMaxX();
    const double minY = writer.getMinY();
    const double maxY = writer.getMaxY();
    const double minZ = writer.getMinZ();
    const double maxZ = writer.getMaxZ();

    BOOST_CHECK_CLOSE(minX, 10.0, 0.001);
    BOOST_CHECK_CLOSE(maxX, 11.0, 0.001);

    BOOST_CHECK_CLOSE(minY, 0.0, 0.001);
    BOOST_CHECK_CLOSE(maxY, 1.0, 0.001);

    BOOST_CHECK_CLOSE(minZ, 3.14, 0.001);
    BOOST_CHECK_CLOSE(maxZ, 3.14, 0.001);
}

BOOST_AUTO_TEST_CASE(pipeline)
{

    pdal::PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(Support::datapath("pipeline/create-dimension.xml"));
    Stage* stage = manager.getStage();
    BOOST_CHECK(stage != NULL);
    stage->initialize();

    const Schema& schema = stage->getSchema();
    PointBuffer data(schema, 2*1065);
    StageSequentialIterator* iter = stage->createSequentialIterator(data);
    boost::uint32_t np = iter->read(data);
    BOOST_CHECK_EQUAL(np, 1065u);


    Dimension const& dimX = data.getSchema().getDimension("Y");
    // std::cout << data.getSchema() << std::endl;
    for (boost::uint32_t i=0; i<10; i++)
    {
        boost::int32_t x = data.getField<boost::int32_t>(dimX, i);
        BOOST_CHECK_EQUAL(x, 314);
    }
            
    delete iter;
}

BOOST_AUTO_TEST_SUITE_END()
#endif
