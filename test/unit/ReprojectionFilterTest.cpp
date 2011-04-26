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
#include <libpc/Iterator.hpp>
#include <libpc/Schema.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/PointBuffer.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ReprojectionFilterTest)


static void getPoint(const libpc::PointBuffer& data, boost::uint32_t pointIndex, double& x, double& y, double& z)
{
    using namespace libpc;

    const Schema& schema = data.getSchema();

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

    return;
}


BOOST_AUTO_TEST_CASE(test_1)
{
    // Test reprojecting UTM 15 to DD with a filter
    libpc::drivers::las::LasReader reader1(Support::datapath("utm15.las"));
    libpc::drivers::las::LasReader reader2(Support::datapath("utm15.las"));
        
    const libpc::SpatialReference& in_ref = reader1.getSpatialReference();
        
    const char* utm15_wkt = "PROJCS[\"NAD83 / UTM zone 15N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.2572221010002,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"26915\"]]";
    BOOST_CHECK(in_ref.getWKT() == utm15_wkt);

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
        
    libpc::SpatialReference out_ref;
    out_ref.setWKT(epsg4326_wkt);
    BOOST_CHECK(out_ref.getWKT() == epsg4326_wkt);
        
    libpc::filters::ReprojectionFilter filter2(reader2, in_ref, out_ref);

    ////liblas::HeaderPtr out_hdr = liblas::HeaderPtr(new liblas::Header(header));
    ////out_hdr->SetScale(0.00000001, 0.00000001, 0.01);
    ////out_hdr->SetOffset(0,0,0);
    ////liblas::TransformPtr srs_transform = liblas::TransformPtr(new liblas::ReprojectionTransform(in_ref, out_ref, out_hdr));

    ////std::vector<liblas::TransformPtr> transforms;
    ////transforms.push_back(srs_transform);
    ////reader.ReadPointAt(0);

    ////liblas::Point unprojected_point = reader.GetPoint();
    //    
    ////ensure_distance("unprojected_point.GetX()", 
    ////                    unprojected_point.GetX(), 
    ////                    double(470692.44), 
    ////                    0.01);

    ////ensure_distance("unprojected_point.GetY()", 
    ////    unprojected_point.GetY(), 
    ////    double(4602888.90), 
    ////    0.01);
    //                    
    ////reader.SetTransforms(transforms);

    ////// This should throw an out of range exception because the given scale/offset 
    ////// combination is not sufficient to store the data.
    ////try
    ////{
    ////    reader.ReadPointAt(0);
    ////    ensure("std::domain_error was not thrown", false);
    ////}
    ////catch (std::domain_error const& e)
    ////{
    ////    ensure(e.what(), true);
    ////}


    ////out_hdr->SetScale(0.0000001, 0.0000001, 0.01);
    ////out_hdr->SetOffset(0,0,0);
    ////srs_transform = liblas::TransformPtr(new liblas::ReprojectionTransform(in_ref, out_ref, out_hdr));

    ////transforms.clear();
    ////transforms.push_back(srs_transform);
    ////reader.SetTransforms(transforms);

    ////reader.Reset();
    ////reader.ReadPointAt(0);


    ////liblas::Point const& projected_point = reader.GetPoint();

    ////ensure_distance("projected_point.GetX()", 
    ////    projected_point.GetX(), 
    ////    double(-93.35156259), 
    ////    0.0000001);
    ////ensure_distance("projected_point.GetY()", 
    ////    projected_point.GetY(), 
    ////    double(41.57714839), 
    ////    0.000001);


    const libpc::Schema& schema = reader1.getSchema();
    libpc::SchemaLayout layout(schema);
    libpc::PointBuffer data1(layout, 1);
    libpc::PointBuffer data2(layout, 1);

    {
        libpc::SequentialIterator* iter1 = reader1.createSequentialIterator();
        boost::uint32_t numRead = iter1->read(data1);
        BOOST_CHECK(numRead == 1);
    }

    {
        libpc::SequentialIterator* iter2 = filter2.createSequentialIterator();
        boost::uint32_t numRead = iter2->read(data2);
        BOOST_CHECK(numRead == 1);
    }

    double x1, x2, y1, y2, z1, z2;
    getPoint(data1, 0, x1, y1, z1);
    getPoint(data2, 0, x2, y2, z2);

    BOOST_CHECK(x1 != x2);
    BOOST_CHECK(y1 != y2);
    BOOST_CHECK(z1 == z2);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
