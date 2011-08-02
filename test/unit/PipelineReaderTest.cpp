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

#include "Support.hpp"

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PipelineReaderTest)


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test1)
{
    PipelineManager manager;
    PipelineReader reader(manager);

    reader.readReaderPipeline(Support::datapath("pipeline_read.xml"));
    Stage* stage = manager.getStage();
    BOOST_CHECK(stage != NULL);
    stage->initialize();

    {
        const Schema& schema = stage->getSchema();
        SchemaLayout layout(schema);
        PointBuffer data(layout, 2048);
        StageSequentialIterator* iter = stage->createSequentialIterator();
        boost::uint32_t np = iter->read(data);
        BOOST_CHECK(np == 1065);

        delete iter;
    }

    return;
}


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test2)
{
    FileUtils::deleteFile("out.las");

    {
        PipelineManager manager;
        PipelineReader reader(manager);

        reader.readWriterPipeline(Support::datapath("pipeline_write.xml"));
        Writer* writerStage = manager.getWriter();
        writerStage->initialize();

        const boost::uint64_t np = writerStage->write();
        BOOST_CHECK(np == 1065);
    }

    FileUtils::deleteFile("out.las");

    return;
}


BOOST_AUTO_TEST_CASE(PipelineReaderTest_test3)
{
    // missing Type
    PipelineManager manager01;
    PipelineReader reader01(manager01);
    BOOST_CHECK_THROW( reader01.readWriterPipeline(Support::datapath("pipeline_bad01.xml")), pipeline_xml_error);

    // missing child of filter
    PipelineManager manager02;
    PipelineReader reader02(manager02);
    BOOST_CHECK_THROW( reader02.readWriterPipeline(Support::datapath("pipeline_bad02.xml")), pipeline_xml_error);

    // missing child of multifilter
    PipelineManager manager03;
    PipelineReader reader03(manager03);
    BOOST_CHECK_THROW( reader03.readWriterPipeline(Support::datapath("pipeline_bad03.xml")), pipeline_xml_error);

    // missing child of writer
    PipelineManager manager04;
    PipelineReader reader04(manager04);
    BOOST_CHECK_THROW( reader04.readWriterPipeline(Support::datapath("pipeline_bad04.xml")), pipeline_xml_error);

    // extra child of filter
    PipelineManager manager05;
    PipelineReader reader05(manager05);
    BOOST_CHECK_THROW( reader05.readWriterPipeline(Support::datapath("pipeline_bad05.xml")), pipeline_xml_error);

    // extra child of writer
    PipelineManager manager06;
    PipelineReader reader06(manager06);
    BOOST_CHECK_THROW( reader06.readWriterPipeline(Support::datapath("pipeline_bad06.xml")), pipeline_xml_error);

    // child of reader
    PipelineManager manager07;
    PipelineReader reader07(manager07);
    BOOST_CHECK_THROW( reader07.readWriterPipeline(Support::datapath("pipeline_bad07.xml")), pipeline_xml_error);

    // unknown element
    PipelineManager manager08;
    PipelineReader reader08(manager08);
    BOOST_CHECK_THROW( reader08.readWriterPipeline(Support::datapath("pipeline_bad08.xml")), pipeline_xml_error);

    // no Pipeline for writer xml
    PipelineManager manager09;
    PipelineReader reader09(manager09);
    BOOST_CHECK_THROW( reader08.readWriterPipeline(Support::datapath("pipeline_bad09.xml")), pipeline_xml_error);

    // no Pipeline for reader xml
    PipelineManager manager10;
    PipelineReader reader10(manager10);
    BOOST_CHECK_THROW( reader10.readReaderPipeline(Support::datapath("pipeline_bad10.xml")), pipeline_xml_error);

    // try to make a reader pipeline from a writer pipeline xml file
    PipelineManager managerW;
    PipelineReader readerW(managerW);
    BOOST_CHECK_THROW( readerW.readReaderPipeline(Support::datapath("pipeline_write.xml")), pipeline_xml_error);

    // try to make a writer pipeline from a reader pipeline xml file
    PipelineManager managerR;
    PipelineReader readerR(managerR);
    BOOST_CHECK_THROW( readerR.readWriterPipeline(Support::datapath("pipeline_read.xml")), pipeline_xml_error);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
