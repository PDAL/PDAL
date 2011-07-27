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

#include <boost/property_tree/xml_parser.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PipelineManagerTest)

BOOST_AUTO_TEST_CASE(PipelineManagerTest_test1)
{
    PipelineManager mgr;

    Options optsR;
    optsR.add("filename", Support::datapath("1.2-with-color.las"));
    Reader* reader = mgr.addReader("drivers.las.reader", optsR);

    Options optsF;
    optsF.add("bounds", Bounds<double>(0,0,0,1000000,1000000,1000000));
    Filter* filter = mgr.addFilter("filters.crop", *reader, optsF);

    Options optsW;
    optsW.add("filename", "temp.las", "file to write to");
    Writer* writer = mgr.addWriter("drivers.las.writer", *filter, optsW);

    const boost::uint64_t np = writer->write( reader->getNumPoints() );
    BOOST_CHECK(np == 1065);

    return;
}


BOOST_AUTO_TEST_CASE(PipelineManagerTest_test2)
{
    PipelineManager mgr;

    const Stage& stage = mgr.readReaderPipeline(Support::datapath("pipeline_read.xml"));

    {
        const Schema& schema = stage.getSchema();
        SchemaLayout layout(schema);
        PointBuffer data(layout, 2048);
        StageSequentialIterator* iter = stage.createSequentialIterator();
        boost::uint32_t np = iter->read(data);
        BOOST_CHECK(np == 1065);

        delete iter;
    }

    return;
}


BOOST_AUTO_TEST_CASE(PipelineManagerTest_test3)
{
    PipelineManager mgr;

    Writer& writer = mgr.readWriterPipeline(Support::datapath("pipeline_write.xml"));

    const boost::uint64_t np = writer.write();
    BOOST_CHECK(np == 1065);

    return;
}


BOOST_AUTO_TEST_CASE(PipelineManagerTest_test4)
{
    PipelineManager mgr;

    mgr.readWriterPipeline(Support::datapath("pipeline_write.xml"));

    mgr.writeWriterPipeline("test.xml");

    bool filesSame = Support::compare_files("test.xml", Support::datapath("pipeline_write_out.xml"));
    BOOST_CHECK(filesSame);

    //if (filesSame)
    {
        Utils::deleteFile("test.xml");
    }

    return;
}


BOOST_AUTO_TEST_SUITE_END()
