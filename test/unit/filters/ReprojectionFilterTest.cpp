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
#include <LasReader.hpp>
#include <ReprojectionFilter.hpp>
#include <StreamCallbackFilter.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

#if defined(PDAL_HAVE_LIBGEOTIFF)
void getPoint(const PointView& data, double& x, double& y, double& z)
{
    x = data.getFieldAs<double>(Dimension::Id::X, 0);
    y = data.getFieldAs<double>(Dimension::Id::Y, 0);
    z = data.getFieldAs<double>(Dimension::Id::Z, 0);
}
#endif

} // unnamed namespace


#if defined(PDAL_HAVE_LIBGEOTIFF)
// Test reprojecting UTM 15 to DD with a filter
TEST(ReprojectionFilterTest, ReprojectionFilterTest_test_1)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    PointTable table;

    const double postX = -93.351563;
    const double postY = 41.577148;
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

        EXPECT_FLOAT_EQ(x, postX);
        EXPECT_FLOAT_EQ(y, postY);
        EXPECT_FLOAT_EQ(z, postZ);
    }
}
#endif

#if defined(PDAL_HAVE_LIBGEOTIFF)
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
        const double z = 16.000000;

        if (i == 0)
        {
            EXPECT_FLOAT_EQ(point.getFieldAs<float>(Dimension::Id::X), x);
            EXPECT_FLOAT_EQ(point.getFieldAs<float>(Dimension::Id::Y), y);
            EXPECT_FLOAT_EQ(point.getFieldAs<float>(Dimension::Id::Z), z);
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
#endif

