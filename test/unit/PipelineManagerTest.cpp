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
        optsR.add("filename", Support::datapath("las/1.2-with-color.las"));
        Reader* reader = mgr.addReader("drivers.las.reader");
        reader->setOptions(optsR);

        Options optsF;
        optsF.add("bounds", BOX3D(0,0,0,1000000,1000000,1000000));
        Filter* filter = mgr.addFilter("filters.crop", reader);
        filter->setOptions(optsF);

        Options optsW;
        optsW.add("filename", "temp.las", "file to write to");
        Writer* writer = mgr.addWriter("drivers.las.writer", filter);
        writer->setOptions(optsW);

        point_count_t np = mgr.execute();
        BOOST_CHECK(np == 1065);
    }

    FileUtils::deleteFile("temp.las");
}


//ABELL - Mosaic
/**
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
        Filter* filter = mgr.addFilter("filters.crop", multifilter, optsF);

        Options optsW;
        optsW.add("filename", "temp.las", "file to write to");
        Writer* writer = mgr.addWriter("drivers.las.writer", *filter, optsW);
        point_count_t np = mgr.execute();

        BOOST_CHECK(np == 1065 * 2);

        std::vector<Stage *> reader1_inputs = reader1->getInputs();
        std::vector<Stage *> reader2_inputs = reader2->getInputs();
        std::vector<Stage *> multifilter_inputs = multifilter->getInputs();
        std::vector<Stage *> filter_inputs = filter->getInputs();
        std::vector<Stage *> writer_inputs = writer->getInputs();

        BOOST_CHECK(reader1_inputs.size() == 0);
        BOOST_CHECK(reader2_inputs.size() == 0);
        BOOST_CHECK(multifilter_inputs.size() == 2);
        BOOST_CHECK(multifilter_inputs[0] == reader1);
        BOOST_CHECK(multifilter_inputs[1] == reader2);
        BOOST_CHECK(filter_inputs.size() == 1);
        BOOST_CHECK(filter_inputs[0] == multifilter);
        BOOST_CHECK(writer_inputs.size() == 1);
        BOOST_CHECK(writer_inputs[0] == filter);
    }

    FileUtils::deleteFile("temp.las");
}
**/

BOOST_AUTO_TEST_SUITE_END()
