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

#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

TEST(PipelineManagerTest, basic)
{
    const char * outfile = "temp.las";
    FileUtils::deleteFile(outfile);

    PipelineManager mgr;

    std::cout << "R" << std::endl;
    Options optsR;
    optsR.add("filename", Support::datapath("las/1.2-with-color.las"));
    Stage& reader = mgr.addReader("readers.las");
    reader.setOptions(optsR);

    std::cout << "W" << std::endl;
    Options optsW;
    optsW.add("filename", outfile, "file to write to");
    Stage& writer = mgr.addWriter("writers.las");
    writer.setInput(reader);
    writer.setOptions(optsW);

    std::cout << "E" << std::endl;
    point_count_t np = mgr.execute();
    std::cout << "Done" << std::endl;
    EXPECT_TRUE(np == 1065U);

    EXPECT_TRUE(!std::ifstream(outfile).fail());
    FileUtils::deleteFile(outfile);
}


//ABELL - Mosaic
/**
TEST(PipelineManagerTest, PipelineManagerTest_test2)
{
    FileUtils::deleteFile("temp.las");

    {
        PipelineManager mgr;

        Options optsR1;
        optsR1.add("filename", Support::datapath("1.2-with-color.las"));
        std::shared_ptr<Stage> reader1(mgr.addReader("readers.las", optsR1));

        Options optsR2;
        optsR2.add("filename", Support::datapath("1.2-with-color.las"));
        std::shared_ptr<Stage> reader2(mgr.addReader("readers.las", optsR2));

        Options optsMF;
        std::vector<std::shared_ptr<Stage> > vec;
        vec.push_back(reader1);
        vec.push_back(reader2);
        MultiFilter* multifilter = mgr.addMultiFilter("filters.mosaic", vec, optsMF);

        Options optsF;
        optsF.add("bounds", Bounds<double>(0,0,0,1000000,1000000,1000000));
        Filter* filter = mgr.addFilter("filters.crop", multifilter, optsF);

        Options optsW;
        optsW.add("filename", "temp.las", "file to write to");
        std::shared_ptr<Stage> writer(mgr.addWriter("writers.las", *filter, optsW));
        point_count_t np = mgr.execute();

        EXPECT_TRUE(np == 1065 * 2);

        std::vector<std::shared_ptr<Stage> > reader1_inputs = reader1->getInputs();
        std::vector<std::shared_ptr<Stage> > reader2_inputs = reader2->getInputs();
        std::vector<std::shared_ptr<Stage> > multifilter_inputs = multifilter->getInputs();
        std::vector<std::shared_ptr<Stage> > filter_inputs = filter->getInputs();
        std::vector<std::shared_ptr<Stage> > writer_inputs = writer->getInputs();

        EXPECT_TRUE(reader1_inputs.size() == 0);
        EXPECT_TRUE(reader2_inputs.size() == 0);
        EXPECT_TRUE(multifilter_inputs.size() == 2);
        EXPECT_TRUE(multifilter_inputs[0] == reader1);
        EXPECT_TRUE(multifilter_inputs[1] == reader2);
        EXPECT_TRUE(filter_inputs.size() == 1);
        EXPECT_TRUE(filter_inputs[0] == multifilter);
        EXPECT_TRUE(writer_inputs.size() == 1);
    }

    FileUtils::deleteFile("temp.las");
}
**/
