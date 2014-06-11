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

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"
#include "../StageTester.hpp"

BOOST_AUTO_TEST_SUITE(InPlaceReprojectionFilterTest)

#ifdef PDAL_SRS_ENABLED

static void getPoint(const pdal::PointBuffer& data,
    double & x, double& y, double& z)
{
    using namespace pdal;

    const Schema& schema = data.getSchema();

    Dimension const& dim_x = schema.getDimension("X");
    Dimension const& dim_y = schema.getDimension("Y");
    Dimension const& dim_z = schema.getDimension("Z");
    
    x = data.getFieldAs<double>(dim_x, 0);
    y = data.getFieldAs<double>(dim_y, 0);
    z = data.getFieldAs<double>(dim_z, 0);
}


// Test reprojecting UTM 15 to DD with a filter
BOOST_AUTO_TEST_CASE(InPlaceReprojectionFilterTest_test_1)
{
    using namespace pdal;

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    PointContext ctx;

    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;

    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));

        pdal::Options options;
        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");
        pdal::Option out_srs("out_srs",out_ref.getWKT(),
            "Output SRS to reproject to");
        pdal::Option x_dim("x_dim", std::string("X"),
            "Dimension name to use for 'X' data");
        pdal::Option y_dim("y_dim", std::string("Y"),
            "Dimension name to use for 'Y' data");
        pdal::Option z_dim("z_dim", std::string("Z"),
            "Dimension name to use for 'Z' data");
        pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X "
            "data in the case when 'X' dimension data are to be scaled.  "
            "Defaults to '1.0'.  If not set, the Dimensions's scale will "
            "be used");
        pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y "
            "data in the case when 'Y' dimension data are to be scaled.  "
            "Defaults to '1.0'.  If not set, the Dimensions's scale will "
            "be used");
        options.add(out_srs);
        options.add(x_dim);
        options.add(y_dim);
        options.add(z_dim);
        options.add(x_scale);
        options.add(y_scale);

        filters::InPlaceReprojection reprojectionFilter(options);
        reprojectionFilter.setInput(&reader);

        reprojectionFilter.prepare(ctx);

        PointBuffer buffer(ctx);

        StageSequentialIterator* iter = reader.createSequentialIterator();

std::cerr << "#1\n";
        boost::uint32_t numRead = iter->read(buffer, 1);
        BOOST_CHECK(numRead == 1);

std::cerr << "#2\n";
        FilterTester::ready(&reprojectionFilter, ctx);
std::cerr << "#3\n";
        FilterTester::filter(&reprojectionFilter, buffer);
std::cerr << "#4\n";

//ABELL
/**
        Bounds<double> newBounds_ref(postX, postY, postZ, postX, postY, postZ);
        Bounds<double> newBounds = buffer.getSpatialBounds();
        Support::compareBounds(newBounds_ref, newBounds);
**/

        double x, y, z;
        getPoint(buffer, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.0001);
        BOOST_CHECK_CLOSE(y, postY, 0.0001);
        BOOST_CHECK_CLOSE(z, postZ, 0.0001);
        delete iter;
    }
}


// Test reprojecting UTM 15 to DD with a filter randomly
BOOST_AUTO_TEST_CASE(InPlaceReprojectionFilterTest_test_2)
{
    using namespace pdal;

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    PointContext ctx;

    const double postX = 194161.33;
    const double postY = 258783.820;
    const double postZ = 131.570;

    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        Options options;
        
        Option debug("debug", true, "");
        Option verbose("verbose", 9, "");
        Option out_srs("out_srs","EPSG:2993", "Output SRS to reproject to");
        Option x_dim("x_dim", std::string("X"), "Dimension name to use for "
            "'X' data");
        Option y_dim("y_dim", std::string("Y"), "Dimension name to use for "
            "'Y' data");
        Option z_dim("z_dim", std::string("Z"), "Dimension name to use for "
            "'Z' data");
        Option x_scale("scale_x", 0.01f, "Scale for output X data in the "
            "case when 'X' dimension data are to be scaled.  Defaults "
            "to '1.0'.  If not set, the Dimensions's scale will be used");
        Option y_scale("scale_y", 0.01f, "Scale for output Y data in the "
            "case when 'Y' dimension data are to be scaled.  Defaults "
            "to '1.0'.  If not set, the Dimensions's scale will be used");
        Option offset_x("offset_x", -123.0f, "Scale for output X data in "
            "the case when 'X' dimension data are to be scaled.  Defaults "
            "to '1.0'.  If not set, the Dimensions's scale will be used");
        Option offset_y("offset_y", 43.0f, "Scale for output Y data in "
            "the case when 'Y' dimension data are to be scaled.  Defaults "
            "to '1.0'.  If not set, the Dimensions's scale will be used");
        Option filename("filename", Support::datapath("autzen-dd.las"),
            "filename");
        options.add(out_srs);
        options.add(x_dim);
        options.add(y_dim);
        options.add(z_dim);
        options.add(x_scale);
        options.add(y_scale);
        options.add(filename);

        drivers::las::Reader reader(options);
        filters::InPlaceReprojection reprojectionFilter(options);
        reprojectionFilter.setInput(&reader);
        reprojectionFilter.prepare(ctx);

        pdal::PointBuffer buffer(ctx);
        StageSequentialIterator* iter = reader.createSequentialIterator();
        
        point_count_t numRead = iter->read(buffer, 1);
        BOOST_CHECK(numRead == 1);

        FilterTester::ready(&reprojectionFilter, ctx);
        FilterTester::filter(&reprojectionFilter, buffer);

        double x, y, z;
        getPoint(buffer, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.0001);
        BOOST_CHECK_CLOSE(y, postY, 0.0001);
        BOOST_CHECK_CLOSE(z, postZ, 0.0001);
        delete iter;

    }
}
#endif

BOOST_AUTO_TEST_SUITE_END()
