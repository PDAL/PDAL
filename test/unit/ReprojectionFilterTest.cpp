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
#include <pdal/filters/Reprojection.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ReprojectionTest)


#ifdef PDAL_SRS_ENABLED

static void getPoint(const pdal::PointBuffer& data, double& x, double& y, double& z, double scaleX, double scaleY, double scaleZ)
{
    using namespace pdal;

    const ::pdal::Schema& schema = data.getSchema();

    const int indexX = schema.getDimensionIndex(DimensionId::X_i32);
    const int indexY = schema.getDimensionIndex(DimensionId::Y_i32);
    const int indexZ = schema.getDimensionIndex(DimensionId::Z_i32);

    const boost::int32_t xraw = data.getField<boost::int32_t>(0, indexX);
    const boost::int32_t yraw = data.getField<boost::int32_t>(0, indexY);
    const boost::int32_t zraw = data.getField<boost::int32_t>(0, indexZ);

    x = (double)xraw * scaleX;
    y = (double)yraw * scaleY;
    z = (double)zraw * scaleZ;

    return;
}


// Test reprojecting UTM 15 to DD with a filter
BOOST_AUTO_TEST_CASE(ReprojectionTest_test_1)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    const char* utm15_wkt = "PROJCS[\"NAD83 / UTM zone 15N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.2572221010002,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"26915\"]]";

    const double preX = 470692.447538;
    const double preY = 4602888.904642;
    const double preZ = 16.000000;
    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;

    // we compute three answers:
    //   (1) w/out reprojection
    //   (2) with scaling and reproj
    //   (3) with custom scaling and reproj

    //
    // (1)
    //
    {
        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));
        reader.initialize();

        const pdal::SpatialReference in_ref_test(utm15_wkt);
        const pdal::SpatialReference out_ref_test(epsg4326_wkt);
        
        const pdal::SpatialReference in_ref(reader.getSpatialReference());
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        BOOST_CHECK_EQUAL(in_ref, in_ref_test);
        BOOST_CHECK(out_ref == out_ref_test);

        const pdal::Schema& schema = reader.getSchema();
        pdal::PointBuffer data(schema, 1);

        pdal::StageSequentialIterator* iter = reader.createSequentialIterator();
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);
        delete iter;
    
        // note this file has only 1 points, so yes, the extent's mins and maxes are the same
        const pdal::Bounds<double> oldBounds_ref(preX, preY, preZ, preX, preY, preZ);
        const pdal::Bounds<double>& oldBounds = reader.getBounds();
        Support::compareBounds(oldBounds_ref, oldBounds);

        double x=0, y=0, z=0;
        getPoint(data, x, y, z, 0.01, 0.01, 0.01);

        BOOST_CHECK_CLOSE(x, preX, 1);
        BOOST_CHECK_CLOSE(y, preY, 1);
        BOOST_CHECK_CLOSE(z, preZ, 1);
    }

    //
    // (2)
    //
    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));

        pdal::filters::Scaling scalingFilter(reader);
        pdal::filters::Reprojection reprojectionFilter(scalingFilter, out_ref);
        pdal::filters::Descaling descalingFilter(reprojectionFilter);
        
        descalingFilter.initialize();

        const pdal::Schema& schema = descalingFilter.getSchema();
        pdal::PointBuffer data(schema, 1);

        pdal::StageSequentialIterator* iter = descalingFilter.createSequentialIterator();
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);
        delete iter;

        const pdal::Bounds<double> newBounds_ref(postX, postY, postZ, postX, postY, postZ);
        const pdal::Bounds<double>& newBounds = descalingFilter.getBounds();
        Support::compareBounds(newBounds_ref, newBounds);

        double x=0, y=0, z=0;
        getPoint(data, x, y, z, 0.01, 0.01, 0.01);

        BOOST_CHECK_CLOSE(x, postX, 1);
        BOOST_CHECK_CLOSE(y, postY, 1);
        BOOST_CHECK_CLOSE(z, postZ, 1);
    }

    //
    // (2), but using the Option-based ctor
    //
    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));

        pdal::Option opt2("out_srs", out_ref.getWKT());
        pdal::Options opts(opt2);

        pdal::filters::Scaling scalingFilter(reader);
        pdal::filters::Reprojection reprojectionFilter(scalingFilter, opts);
        pdal::filters::Descaling descalingFilter(reprojectionFilter);
        
        descalingFilter.initialize();

        const pdal::Schema& schema = descalingFilter.getSchema();
        pdal::PointBuffer data(schema, 1);

        pdal::StageSequentialIterator* iter = descalingFilter.createSequentialIterator();
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);
        delete iter;

        const pdal::Bounds<double> newBounds_ref(postX, postY, postZ, postX, postY, postZ);
        const pdal::Bounds<double>& newBounds = descalingFilter.getBounds();
        Support::compareBounds(newBounds_ref, newBounds);

        double x=0, y=0, z=0;
        getPoint(data, x, y, z, 0.01, 0.01, 0.01);

        BOOST_CHECK_CLOSE(x, postX, 1);
        BOOST_CHECK_CLOSE(y, postY, 1);
        BOOST_CHECK_CLOSE(z, postZ, 1);
    }

    //
    // (3)
    //
    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));
            
        // convert to doubles, use internal scale factor
        pdal::filters::Scaling scalingFilter(reader);

        pdal::filters::Reprojection reprojectionFilter(scalingFilter, out_ref);
    
        // convert to ints, using custom scale factor
        pdal::filters::Descaling descalingFilter(reprojectionFilter, 0.000001, 0.0, 0.000001, 0.0, 0.01, 0.0);
        
        descalingFilter.initialize();

        const pdal::Schema& schema = descalingFilter.getSchema();
        pdal::PointBuffer data2(schema, 1);

        pdal::StageSequentialIterator* iter = descalingFilter.createSequentialIterator();
        boost::uint32_t numRead = iter->read(data2);
        BOOST_CHECK(numRead == 1);
        delete iter;

        double x=0, y=0, z=0;
        getPoint(data2, x, y, z, 0.000001, 0.000001, 0.01);

        BOOST_CHECK_CLOSE(x, postX, 1);
        BOOST_CHECK_CLOSE(y, postY, 1);
        BOOST_CHECK_CLOSE(z, postZ, 1);
    }

    return;
}

#endif

BOOST_AUTO_TEST_CASE(test_impedence_mismatch)
{
    pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));
              
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
        
    pdal::SpatialReference out_ref;
    out_ref.setWKT(epsg4326_wkt);
        
    bool ok = false;
    try
    {
        pdal::filters::Reprojection filter(reader, out_ref);
        filter.initialize();
        ok = false;
    }
    catch (pdal::impedance_invalid&)
    {
        ok = true;
    }
    BOOST_CHECK(ok == true);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
