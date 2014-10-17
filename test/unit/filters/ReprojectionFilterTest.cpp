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

#include "UnitTest.hpp"

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Reprojection.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"
#include "../StageTester.hpp"

BOOST_AUTO_TEST_SUITE(ReprojectionFilterTest)

#ifdef PDAL_SRS_ENABLED

static void getPoint(const pdal::PointBuffer& data,
    double& x, double& y, double& z)
{
    using namespace pdal;

    x = data.getFieldAs<double>(Dimension::Id::X, 0);
    y = data.getFieldAs<double>(Dimension::Id::Y, 0);
    z = data.getFieldAs<double>(Dimension::Id::Z, 0);
}

// Test reprojecting UTM 15 to DD with a filter
BOOST_AUTO_TEST_CASE(ReprojectionFilterTest_test_1)
{
    using namespace pdal;

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    PointContext ctx;

    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;

    {
        const SpatialReference out_ref(epsg4326_wkt);

        Options ops1;
        ops1.add("filename", Support::datapath("las/utm15.las"));
        drivers::las::Reader reader;
        reader.setOptions(ops1);

        Options options;
        Option debug("debug", true, "");
        Option verbose("verbose", 9, "");
        Option out_srs("out_srs",out_ref.getWKT(),
            "Output SRS to reproject to");
        options.add(out_srs);

        filters::Reprojection reprojectionFilter;
        reprojectionFilter.setOptions(options);
        reprojectionFilter.setInput(&reader);

        reprojectionFilter.prepare(ctx);
        PointBufferSet pbSet = reprojectionFilter.execute(ctx);
        BOOST_CHECK_EQUAL(pbSet.size(), 1);
        PointBufferPtr buffer = *pbSet.begin();

        double x, y, z;
        getPoint(*buffer, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.1);
        BOOST_CHECK_CLOSE(y, postY, 0.1);
        BOOST_CHECK_CLOSE(z, postZ, 0.1);
    }
}

/**
 This test would pass but for the strange scaling of the dimension, which
 exceeds an integer.

// Test reprojecting UTM 15 to DD with a filter randomly
BOOST_AUTO_TEST_CASE(InPlaceReprojectionFilterTest_test_2)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    const double postX = 194161.33;
    const double postY = 258783.820;
    const double postZ = 131.570;

    {
        using namespace pdal;

        PointContext ctx;

        const SpatialReference out_ref(epsg4326_wkt);

        Options options;

        Option debug("debug", true, "");
        Option verbose("verbose", 9, "");
        Option out_srs("out_srs","EPSG:2993", "Output SRS to reproject to");

        Option filename("filename",
            Support::datapath("las/autzen-dd.las"), "filename");
        options.add(out_srs);
        options.add(filename);

        drivers::las::Reader reader(options);
        filters::Reprojection reprojectionFilter(options);
        reprojectionFilter.setInput(&reader);
        reprojectionFilter.prepare(ctx);

        PointBuffer buffer(ctx);
        StageSequentialIterator* iter = reader.createSequentialIterator();

        point_count_t numRead = iter->read(buffer, 1);
        BOOST_CHECK(numRead == 1);

        FilterTester::ready(&reprojectionFilter, ctx);
        FilterTester::filter(&reprojectionFilter, buffer);

        double x, y, z;
        getPoint(buffer, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.1);
        BOOST_CHECK_CLOSE(y, postY, 0.1);
        BOOST_CHECK_CLOSE(z, postZ, 0.1);
        delete iter;
    }
}
**/
#endif

BOOST_AUTO_TEST_SUITE_END()
