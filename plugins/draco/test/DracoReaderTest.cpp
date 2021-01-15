/******************************************************************************
* Copyright (c) 2020 Hobu, Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#define NOMINMAX

#include <pdal/Filter.hpp>
#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include "../io/DracoReader.hpp"
#include "../io/DracoWriter.hpp"


namespace pdal
{

void checkPoint(const PointViewPtr& view, point_count_t idx,
        double x, double y, double z)
{
    EXPECT_DOUBLE_EQ(x, view->getFieldAs<double>(Dimension::Id::X, idx));
    EXPECT_DOUBLE_EQ(y, view->getFieldAs<double>(Dimension::Id::Y, idx));
    EXPECT_DOUBLE_EQ(z, view->getFieldAs<double>(Dimension::Id::Z, idx));
}


TEST(DracoReader, Constructor)
{
    DracoReader reader1;

    StageFactory f;
    Stage* reader2(f.createStage("readers.draco"));
    EXPECT_TRUE(reader2);
}



TEST(DracoReaderTest, test_sequential)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("draco/1.2-with-color.drc"));
    DracoReader reader;
    reader.setOptions(ops1);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    Support::check_p0_p1_p2(*view);

}

TEST(DracoReaderTest, accuracy)
{
    //create pipeline that reads draco file and transforms it via
    //filters.transformation. Then apply the offset from RTC_CENTER from
    //the pnts file. Check that the results of this fall within the bounds of
    //red-rocks.laz

    //  RTC_CENTER: [ -0.015410084428367554, -0.35363949998281896, 92.70944035355933 ]
    PipelineManager pipeline;

    Options readOptions;
    std::string path = Support::datapath("draco/redrocks.drc");
    Stage& reader = pipeline.makeReader(path, "readers.draco");

    Options filterOptions;
    filterOptions.add("matrix", "0.9649933973123795 -0.26227417551774335 0 0 0.16741023360918053 0.6159575938289551 0.7697857210207032 0 -0.20189491530603648 -0.742838138130328 0.6383023920624428 0 -1289846.4516338364 -4745771.507684133 4050624.605121021 1");
    Stage& filter = pipeline.makeFilter("filters.transformation", reader);
    filter.setOptions(filterOptions);

    point_count_t count = pipeline.execute();
    std::cout << "Point count " << count << std::endl;

}

}
