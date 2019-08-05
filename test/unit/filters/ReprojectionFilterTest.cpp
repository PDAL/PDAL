/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <pdal/SpatialReference.hpp>
#include <pdal/PointView.hpp>
#include <io/LasReader.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

void getPoint(const PointView& data, double& x, double& y, double& z)
{
    x = data.getFieldAs<double>(Dimension::Id::X, 0);
    y = data.getFieldAs<double>(Dimension::Id::Y, 0);
    z = data.getFieldAs<double>(Dimension::Id::Z, 0);
}

} // unnamed namespace


// Test reprojecting UTM 15 to DD with a filter
TEST(ReprojectionFilterTest, ReprojectionFilterTest_test_1)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    PointTable table;

    const double postX = -93.351563;
    const double postY = 41.577148;
    // proj 4 transformed to 16 exactly, but proj 5 will consider
    // +ellps=GRS80 +towgs84=0,0,0 to be slighly different than +datum=WGS84
    // and return 15.999954
    const double postZ = 16.000000;

    {
        const SpatialReference out_ref(epsg4326_wkt);

        Options ops1;
        ops1.add("filename", Support::datapath("las/utm15.las"));
        LasReader reader;
        reader.setOptions(ops1);

        Options options;
        options.add("out_srs", out_ref.getWKT());

        ReprojectionFilter reprojectionFilter;
        reprojectionFilter.setOptions(options);
        reprojectionFilter.setInput(reader);

        reprojectionFilter.prepare(table);
        PointViewSet viewSet = reprojectionFilter.execute(table);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();

        double x, y, z;
        getPoint(*view.get(), x, y, z);

        EXPECT_NEAR(x, postX, .000001);
        EXPECT_NEAR(y, postY, .000001);
        EXPECT_NEAR(z, postZ, 5e-5);
    }
}

// Test reprojecting UTM 15 to DD with a filter
TEST(ReprojectionFilterTest, stream_test_1)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";


    Options ops1;
    ops1.add("filename", Support::datapath("las/utm15.las"));
    LasReader reader;
    reader.setOptions(ops1);

    Options options;
    options.add("out_srs", epsg4326_wkt);

    ReprojectionFilter reprojectionFilter;
    reprojectionFilter.setOptions(options);
    reprojectionFilter.setInput(reader);

    auto cb = [](PointRef& point)
    {
        static int i = 0;
        const double x = -93.351563;
        const double y = 41.577148;
        // proj 4 transformed to 16 exactly, but proj 5 will consider
        // +ellps=GRS80 +towgs84=0,0,0 to be slighly different than +datum=WGS84
        // and return 15.999954
        const double z = 16.000000;

        if (i == 0)
        {
            EXPECT_NEAR(point.getFieldAs<float>(Dimension::Id::X), x, .0001);
            EXPECT_NEAR(point.getFieldAs<float>(Dimension::Id::Y), y, .0001);
            EXPECT_NEAR(point.getFieldAs<float>(Dimension::Id::Z), z, 5e-5);
        }
        ++i;
        return true;
    };

    StreamCallbackFilter stream;
    stream.setCallback(cb);
    stream.setInput(reprojectionFilter);

    FixedPointTable table(20);

    stream.prepare(table);
    stream.execute(table);
}


// Test reprojecting UTM 16 AND UTM 17 to DD
TEST(ReprojectionFilterTest, stream_test_2)
{
    // Fill table 1 with UTM17 data.
    Options ops1;
    ops1.add("filename", Support::datapath("las/test_utm17.las"));
    LasReader reader1;
    reader1.setOptions(ops1);

    Options ops1a;
    ReprojectionFilter repro1;
    ops1a.add("out_srs", "EPSG:4326");
    repro1.setInput(reader1);
    repro1.setOptions(ops1a);

    PointTable table1;
    repro1.prepare(table1);
    PointViewSet s1 = repro1.execute(table1);
    PointViewPtr v1 = *(s1.begin());

    // Fill table 2 with UTM16 data.
    Options ops2;
    ops2.add("filename", Support::datapath("las/test_utm16.las"));
    LasReader reader2;
    reader2.setOptions(ops2);

    Options ops2a;
    ReprojectionFilter repro2;
    ops2a.add("out_srs", "EPSG:4326");
    repro2.setInput(reader2);
    repro2.setOptions(ops2a);

    PointTable table2;
    repro2.prepare(table2);
    PointViewSet s2 = repro2.execute(table2);
    PointViewPtr v2 = *(s2.begin());

    // Put both inputs through the first repro filter.
    repro1.setInput(reader2);

    // Make sure the data looks the same coming from the stream and
    // non-stream operations.
    auto cb = [&](PointRef& point)
    {
        static size_t i = 0;
        EXPECT_TRUE(i < v1->size() + v2->size());
        PointId idx;
        if (i < v1->size())
        {
            idx = i;
            EXPECT_EQ(v1->getFieldAs<double>(Dimension::Id::X, idx),
                point.getFieldAs<double>(Dimension::Id::X));
            EXPECT_EQ(v1->getFieldAs<double>(Dimension::Id::Y, idx),
                point.getFieldAs<double>(Dimension::Id::Y));
            EXPECT_EQ(v1->getFieldAs<double>(Dimension::Id::Z, idx),
                point.getFieldAs<double>(Dimension::Id::Z));
        }
        else if (i < v1->size() + v2->size())
        {
            idx = i - v1->size();
            EXPECT_EQ(v2->getFieldAs<double>(Dimension::Id::X, idx),
                point.getFieldAs<double>(Dimension::Id::X));
            EXPECT_EQ(v2->getFieldAs<double>(Dimension::Id::Y, idx),
                point.getFieldAs<double>(Dimension::Id::Y));
            EXPECT_EQ(v2->getFieldAs<double>(Dimension::Id::Z, idx),
                point.getFieldAs<double>(Dimension::Id::Z));
        }
        i++;
        return true;
    };
    StreamCallbackFilter f;
    f.setInput(repro1);
    f.setCallback(cb);

    FixedPointTable table3(20);
    f.prepare(table3);
    f.execute(table3);
}
