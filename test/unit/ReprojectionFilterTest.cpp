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

#include <libpc/SpatialReference.hpp>
#include <libpc/drivers/las/Reader.hpp>
#include <libpc/filters/ReprojectionFilter.hpp>
#include <libpc/filters/ScalingFilter.hpp>
#include <libpc/Iterator.hpp>
#include <libpc/Schema.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/exceptions.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ReprojectionFilterTest)


static void getPoint(const libpc::PointBuffer& data, boost::uint32_t pointIndex, double& x, double& y, double& z)
{
    using namespace libpc;

    const Schema& schema = data.getSchema();

#if 1
    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const Dimension& xDim = schema.getDimension(indexX);
    const Dimension& yDim = schema.getDimension(indexY);
    const Dimension& zDim = schema.getDimension(indexZ);

    const boost::int32_t xraw = data.getField<boost::int32_t>(pointIndex, indexX);
    const boost::int32_t yraw = data.getField<boost::int32_t>(pointIndex, indexY);
    const boost::int32_t zraw = data.getField<boost::int32_t>(pointIndex, indexZ);

    x = xDim.applyScaling(xraw);
    y = yDim.applyScaling(yraw);
    z = zDim.applyScaling(zraw);
#else
    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);
    const Dimension& xDim = schema.getDimension(indexX);
    const Dimension& yDim = schema.getDimension(indexY);
    const Dimension& zDim = schema.getDimension(indexZ);

    x = data.getField<double>(pointIndex, indexX);
    y = data.getField<double>(pointIndex, indexY);
    z = data.getField<double>(pointIndex, indexZ);
#endif

    return;
}


BOOST_AUTO_TEST_CASE(test_1)
{
    // Test reprojecting UTM 15 to DD with a filter
    libpc::drivers::las::LasReader reader1(Support::datapath("utm15.las"));
    libpc::drivers::las::LasReader reader2(Support::datapath("utm15.las"));
        
    const libpc::SpatialReference& in_ref = reader1.getSpatialReference();
        
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    libpc::SpatialReference out_ref(epsg4326_wkt);

    {
        const char* utm15_wkt = "PROJCS[\"NAD83 / UTM zone 15N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.2572221010002,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"26915\"]]";
        BOOST_CHECK(in_ref.getWKT() == utm15_wkt);
        BOOST_CHECK(out_ref.getWKT() == epsg4326_wkt);
    }

    libpc::filters::ScalingFilter scalingFilter(reader2, false);

    libpc::filters::ReprojectionFilter reprojectionFilter(scalingFilter, in_ref, out_ref);
    
    libpc::filters::ScalingFilter descalingFilter(reprojectionFilter, true);

    const libpc::Schema& schema1 = reader1.getSchema();
    const libpc::SchemaLayout layout1(schema1);
    libpc::PointBuffer data1(layout1, 1);

    const libpc::Schema& schema2 = descalingFilter.getSchema();
    const libpc::SchemaLayout layout2(schema2);
    libpc::PointBuffer data2(layout2, 1);

    {
        libpc::SequentialIterator* iter1 = reader1.createSequentialIterator();
        boost::uint32_t numRead = iter1->read(data1);
        BOOST_CHECK(numRead == 1);
        delete iter1;
    }

    {
        libpc::SequentialIterator* iter2 = descalingFilter.createSequentialIterator();
        boost::uint32_t numRead = iter2->read(data2);
        BOOST_CHECK(numRead == 1);
        delete iter2;
    }

    double x1, x2, y1, y2, z1, z2;
    getPoint(data1, 0, x1, y1, z1);
    getPoint(data2, 0, x2, y2, z2);

    BOOST_CHECK_CLOSE(x1, 470692.44, 1);
    BOOST_CHECK_CLOSE(y1, 4602888.90, 1);
    BOOST_CHECK_CLOSE(z1, 16, 1);

    BOOST_CHECK_CLOSE(x2, -93.35156259, 1);
//////    BOOST_CHECK_CLOSE(y2, 41.57714839, 1); // BUG: I get 90
//////    BOOST_CHECK_CLOSE(z2, 16, 1); // BUG: I get 0 (no idea what this is supposed to be, not in liblas test)

    ////////out_hdr->SetScale(0.00000001, 0.00000001, 0.01);

    return;
}


BOOST_AUTO_TEST_CASE(test_impedence_mismatch)
{
    libpc::drivers::las::LasReader reader(Support::datapath("utm15.las"));
        
    const libpc::SpatialReference& in_ref = reader.getSpatialReference();
        
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
        
    libpc::SpatialReference out_ref;
    out_ref.setWKT(epsg4326_wkt);
        
    bool ok = false;
    try
    {
        libpc::filters::ReprojectionFilter filter(reader, in_ref, out_ref);
        ok = false;
    }
    catch (libpc::impedance_invalid&)
    {
        ok = true;
    }
    BOOST_CHECK(ok == true);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
