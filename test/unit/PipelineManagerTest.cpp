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

#include <pdal/PipelineManager.hpp>
#include <pdal/FileUtils.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PipelineManagerTest)


BOOST_AUTO_TEST_CASE(PipelineManagerTest_test1)
{
    FileUtils::deleteFile("temp.las");

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

        writer->initialize();

        const boost::uint64_t np = writer->write( reader->getNumPoints() );
        BOOST_CHECK(np == 1065);
    }

    FileUtils::deleteFile("temp.las");

    return;
}


BOOST_AUTO_TEST_CASE(PipelineManagerTest_test2)
{
#ifndef PDAL_SRS_ENABLED
    // because mosaicking needs to compare SRSs
    return;
#endif

    FileUtils::deleteFile("temp.las");

    {
        PipelineManager mgr;

        Options optsR1;
        optsR1.add("filename", Support::datapath("1.2-with-color.las"));
        Reader* reader1 = mgr.addReader("drivers.las.reader", optsR1);

        Options optsR2;
        optsR2.add("filename", Support::datapath("1.2-with-color.las"));
        Reader* reader2 = mgr.addReader("drivers.las.reader", optsR2);

        Options optsMF;
        std::vector<Stage*> vec;
        vec.push_back(reader1);
        vec.push_back(reader2);
        MultiFilter* multifilter = mgr.addMultiFilter("filters.mosaic", vec, optsMF);

        Options optsF;
        optsF.add("bounds", Bounds<double>(0,0,0,1000000,1000000,1000000));
        Filter* filter = mgr.addFilter("filters.crop", *multifilter, optsF);

        Options optsW;
        optsW.add("filename", "temp.las", "file to write to");
        Writer* writer = mgr.addWriter("drivers.las.writer", *filter, optsW);

        writer->initialize();

        // check all the prev/next stage linkages

        const boost::uint64_t np = writer->write( 0 );
        BOOST_CHECK(np == 1065 * 2);

        std::vector<StageBase*> reader1_inputs = reader1->getInputs();
        std::vector<StageBase*> reader2_inputs = reader2->getInputs();
        std::vector<StageBase*> multifilter_inputs = multifilter->getInputs();
        std::vector<StageBase*> filter_inputs = filter->getInputs();
        std::vector<StageBase*> writer_inputs = writer->getInputs();

        std::vector<StageBase*> reader1_outputs = reader1->getOutputs();
        std::vector<StageBase*> reader2_outputs = reader2->getOutputs();
        std::vector<StageBase*> multifilter_outputs = multifilter->getOutputs();
        std::vector<StageBase*> filter_outputs = filter->getOutputs();
        std::vector<StageBase*> writer_outputs = writer->getOutputs();
        
        BOOST_CHECK(reader1_inputs.size() == 0);
        BOOST_CHECK(reader2_inputs.size() == 0);
        BOOST_CHECK(multifilter_inputs.size() == 2);
        BOOST_CHECK(multifilter_inputs[0] == reader1);
        BOOST_CHECK(multifilter_inputs[1] == reader2);
        BOOST_CHECK(filter_inputs.size() == 1);
        BOOST_CHECK(filter_inputs[0] == multifilter);
        BOOST_CHECK(writer_inputs.size() == 1);
        BOOST_CHECK(writer_inputs[0] == filter);

        BOOST_CHECK(reader1_outputs.size() == 1);
        BOOST_CHECK(reader1_outputs[0] == multifilter);
        BOOST_CHECK(reader2_outputs.size() == 1);
        BOOST_CHECK(reader2_outputs[0] == multifilter);
        BOOST_CHECK(multifilter_outputs.size() == 1);
        BOOST_CHECK(multifilter_outputs[0] == filter);
        BOOST_CHECK(filter_outputs.size() == 1);
        BOOST_CHECK(filter_outputs[0] == writer);
        BOOST_CHECK(writer_outputs.size() == 0);
    }

    FileUtils::deleteFile("temp.las");

    return;
}

BOOST_AUTO_TEST_SUITE_END()
