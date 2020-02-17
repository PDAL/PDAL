/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <nlohmann/json.hpp>
#include "Support.hpp"

using namespace pdal;

std::string getFilePath()
{
    return Support::datapath("hdf/autzen.h5");
}

TEST(HdfReaderTest, testRead)
{
    //setup
    StageFactory f;
    Stage* reader(f.createStage("readers.hdf"));
    EXPECT_TRUE(reader);

    Option filename("filename", getFilePath());

    NL::json j = {
        // double types
        {"X", "autzen/X"},
        {"Y", "autzen/Y"},
        {"Z", "autzen/Z"},
        {"GpsTime", "autzen/GpsTime"},
        // int types
        {"Classification", "autzen/Classification"},
        {"Intensity", "autzen/Intensity"},
        {"Red", "autzen/Red"},
        {"Green", "autzen/Green"},
        {"Blue", "autzen/Blue"},
    };
    Option dataset("dimensions", j.dump());

    Options options(filename);
    options.add(dataset);
    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    //size equality
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1065u);
    //point equality
    Support::check_p0_p1_p2(*view);
    PointViewPtr view2 = view->makeNew();
    view2->appendPoint(*view, 100);
    view2->appendPoint(*view, 101);
    view2->appendPoint(*view, 102);
    Support::check_p100_p101_p102(*view2);
    //color and GPS time equality
    Support::check_pN(*view, 99, 636699.44, 849829.23, 420.8,
         246504.03026135918, 50, 65, 65);
}

TEST(HdfReaderTest, testOptions)
{
    StageFactory f;
    Stage* reader(f.createStage("readers.hdf"));
    EXPECT_TRUE(reader);

    Option filename("filename", getFilePath());

    NL::json j = {{ "X" ,"autzen/X"}, {"Y" , 1234}};
    Option dataset("dimensions", j.dump());

    Options options(filename);
    options.add(dataset);
    reader->setOptions(options);

    PointTable table;
    ASSERT_THROW(reader->prepare(table), pdal_error);
    ASSERT_TRUE(reader->getSpatialReference().empty());
}
