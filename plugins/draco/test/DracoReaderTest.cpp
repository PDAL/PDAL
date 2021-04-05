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


void check_pN(const pdal::PointView& data, PointId index,
    double xref, double yref, double zref, double tolerance)
{
    float x0 = data.getFieldAs<float>(Dimension::Id::X, index);
    float y0 = data.getFieldAs<float>(Dimension::Id::Y, index);
    float z0 = data.getFieldAs<float>(Dimension::Id::Z, index);

    EXPECT_TRUE(Utils::compare_approx(x0, static_cast<float>(xref), tolerance));
    EXPECT_TRUE(Utils::compare_approx(y0, static_cast<float>(yref), tolerance));
    EXPECT_TRUE(Utils::compare_approx(z0, static_cast<float>(zref), tolerance));
}

void check_p0_p1_p2(const pdal::PointView& data, double tolerance)
{
    check_pN(data, 0, 637012.240000, 849028.310000, 431.660000, tolerance);
    check_pN(data, 1, 636896.330000, 849087.700000, 446.390000, tolerance);
    check_pN(data, 2, 636784.740000, 849106.660000, 426.710000, tolerance);
}

TEST(DracoReaderTest, test_sequential)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("draco/1.2-with-color-test.drc"));
    DracoReader reader;
    reader.setOptions(ops1);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    check_p0_p1_p2(*view, 0.001);
}


TEST(DracoReaderTest, accuracy)
{
    //create pipeline that reads draco file and transforms it via
    //filters.transformation. Then apply the offset from RTC_CENTER from
    //the pnts file. Check that the results of this fall within the bounds of
    //red-rocks.laz

    PipelineManager pipeline;
    //bounding box for original red-rocks.laz this file was created from
    BOX3D bounds(482060.5, 4390187.5, 1843.98, 482763.78, 4391071, 2029.41);
    bounds.grow(1);

    Options readOptions;
    std::string path = Support::datapath("draco/redrocks.drc");
    Stage& reader = pipeline.makeReader(path, "readers.draco");

    //transform by RTC center
    std::string xOff = "-0.015410084428367554";
    std::string yOff = "-0.35363949998281896";
    std::string zOff = "92.70944035355933";
    Options rtcOptions;
    rtcOptions.add("matrix", "1 0 0 "+xOff+" 0 1 0 "+yOff+" 0 0 1 "+zOff+" 0 0 0 1");
    Stage& rtcTransformationFilter = pipeline.makeFilter("filters.transformation", reader);
    rtcTransformationFilter.setOptions(rtcOptions);

    //transform by tileset from cesium
    Options tilesetOptions;
    tilesetOptions.add("matrix", "0.9649933973123795 0.16741023360918053 -0.20189491530603648 -1289846.4516338364 -0.26227417551774335 0.6159575938289551 -0.742838138130328 -4745771.507684133 0 0.7697857210207032 0.6383023920624428 4050624.605121021 0 0 0 1");
    Stage& tilesetTransformationFilter = pipeline.makeFilter("filters.transformation", rtcTransformationFilter);
    tilesetTransformationFilter.setOptions(tilesetOptions);


    //set up projection so we can use it against our bounding box
    Options reprojectionOptions;
    reprojectionOptions.add("in_srs", "EPSG:4978");
    reprojectionOptions.add("out_srs", "EPSG:26913");
    Stage& reprojectionFilter = pipeline.makeFilter("filters.reprojection", tilesetTransformationFilter);
    reprojectionFilter.setOptions(reprojectionOptions);

    point_count_t count = pipeline.execute();

    PointViewSet set = pipeline.views();
    PointViewPtr v = *set.begin();

    for (PointId i = 0; i < count; i += 1) {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        double y = v->getFieldAs<double>(Dimension::Id::Y, i);
        double z = v->getFieldAs<double>(Dimension::Id::Z, i);
        EXPECT_TRUE(bounds.contains(x, y, z));
    }

}

}
