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

BOOST_AUTO_TEST_SUITE(InPlaceReprojectionFilterTest)


#ifdef PDAL_SRS_ENABLED

static void getPoint(const pdal::PointBuffer& data, double& x, double& y, double& z)
{
    using namespace pdal;

    const Schema& schema = data.getSchema();

    Dimension const& dim_x = schema.getDimension("X");
    Dimension const& dim_y = schema.getDimension("Y");
    Dimension const& dim_z = schema.getDimension("Z");
    
    const boost::int32_t xraw = data.getField<boost::int32_t>(dim_x, 0);
    const boost::int32_t yraw = data.getField<boost::int32_t>(dim_y, 0);
    const boost::int32_t zraw = data.getField<boost::int32_t>(dim_z, 0);
    
    // 
    // std::cout << "xraw: " << xraw << " yraw: " << yraw << " zraw: " << zraw << std::endl;
    // 
    // std::cout << data << std::endl;
    x = dim_x.applyScaling<boost::int32_t>(xraw);
    y = dim_y.applyScaling<boost::int32_t>(yraw);
    z = dim_z.applyScaling<boost::int32_t>(zraw);

    return;
}


// Test reprojecting UTM 15 to DD with a filter
BOOST_AUTO_TEST_CASE(InPlaceReprojectionFilterTest_test_1)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    // const char* utm15_wkt = "PROJCS[\"NAD83 / UTM zone 15N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.2572221010002,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"26915\"]]";

    // const double preX = 470692.447538;
    // const double preY = 4602888.904642;
    // const double preZ = 16.000000;
    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;

    // we compute three answers:
    //   (1) w/out reprojection
    //   (2) with scaling and reproj
    //   (3) with custom scaling and reproj

    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);

        pdal::drivers::las::Reader reader(Support::datapath("utm15.las"));

        pdal::Options options;
        
        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");
        // options.add(debug);
        // options.add(verbose);
        pdal::Option out_srs("out_srs",out_ref.getWKT(), "Output SRS to reproject to");
        pdal::Option x_dim("x_dim", std::string("X"), "Dimension name to use for 'X' data");
        pdal::Option y_dim("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
        pdal::Option z_dim("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
        pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
        pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
        // pdal::Option z_scale("scale_z", 1.0, "Scale for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
        // pdal::Option x_offset("offset_x", 0.0, "Offset for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
        // pdal::Option y_offset("offset_y", 0.0, "Offset for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
        // pdal::Option z_offset("offset_z", 0.0, "Offset for output Z data in the case when 'Z' dimension data are to be scaled.  Defaults to '0.0'.  If not set, the Dimensions's scale will be used");
        // options.add(in_srs);
        options.add(out_srs);
        options.add(x_dim);
        options.add(y_dim);
        options.add(z_dim);
        options.add(x_scale);
        options.add(y_scale);
        // options.add(z_scale);
        // options.add(x_offset);
        // options.add(y_offset);
        // options.add(z_offset);

        pdal::filters::InPlaceReprojection reprojectionFilter(reader, options);

        reprojectionFilter.initialize();

        const pdal::Schema& schema = reprojectionFilter.getSchema();
        pdal::PointBuffer data(schema, 1);
        

        pdal::StageSequentialIterator* iter = reprojectionFilter.createSequentialIterator(data);

        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);


        const pdal::Bounds<double> newBounds_ref(postX, postY, postZ, postX, postY, postZ);
        const pdal::Bounds<double>& newBounds = data.getSpatialBounds();
        Support::compareBounds(newBounds_ref, newBounds);

        double x=0, y=0, z=0;
        getPoint(data, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.0001);
        BOOST_CHECK_CLOSE(y, postY, 0.0001);
        BOOST_CHECK_CLOSE(z, postZ, 0.0001);
        delete iter;

    }


    return;
}



// Test reprojecting UTM 15 to DD with a filter randomly
BOOST_AUTO_TEST_CASE(InPlaceReprojectionFilterTest_test_2)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

    const double postX = -117.251688323;
    const double postY = 49.34165044;
    const double postZ = 446.390;


    {
        const pdal::SpatialReference out_ref(epsg4326_wkt);


        pdal::Options options;
        
        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");
        // options.add(debug);
        // options.add(verbose);
        pdal::Option in_srs("spatialreference","EPSG:2993", "Output SRS to reproject to");

        pdal::Option out_srs("out_srs",out_ref.getWKT(), "Output SRS to reproject to");
        pdal::Option x_dim("x_dim", std::string("X"), "Dimension name to use for 'X' data");
        pdal::Option y_dim("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
        pdal::Option z_dim("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
        pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
        pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
        
        pdal::Option filename("filename", Support::datapath("1.2-with-color.las"), "filename");
        options.add(out_srs);
        options.add(in_srs);
        options.add(x_dim);
        options.add(y_dim);
        options.add(z_dim);
        options.add(x_scale);
        options.add(y_scale);
        options.add(filename);
        pdal::drivers::las::Reader reader(options);

        pdal::filters::InPlaceReprojection reprojectionFilter(reader, options);

        reprojectionFilter.initialize();

        const pdal::Schema& schema = reprojectionFilter.getSchema();
        pdal::PointBuffer data(schema, 1);
        

        pdal::StageRandomIterator* iter = reprojectionFilter.createRandomIterator(data);
        
        if (iter==NULL) throw std::runtime_error("could not create random iterator from InPlaceReprojectionFilter!");
        iter->seek(1);
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);


        const pdal::Bounds<double> newBounds_ref(postX, postY, postZ, postX, postY, postZ);
        const pdal::Bounds<double>& newBounds = data.getSpatialBounds();
        Support::compareBounds(newBounds_ref, newBounds);

        double x=0, y=0, z=0;
        getPoint(data, x, y, z);

        BOOST_CHECK_CLOSE(x, postX, 0.0001);
        BOOST_CHECK_CLOSE(y, postY, 0.0001);
        BOOST_CHECK_CLOSE(z, postZ, 0.0001);
        delete iter;

    }


    return;
}
#endif

BOOST_AUTO_TEST_SUITE_END()
