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
#include <boost/cstdint.hpp>
#include <boost/scoped_ptr.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "Support.hpp"

#include <pdal/filters/Stats.hpp>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StatsFilterTest)

BOOST_AUTO_TEST_CASE(StatsFilterTest_test1)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Constant);

    pdal::filters::Stats filter(reader, Options::none());
    BOOST_CHECK_EQUAL(filter.getName(), "filters.stats");
    BOOST_CHECK_EQUAL(filter.getDescription(), "Statistics Filter");
    filter.initialize();

    const Schema& schema = filter.getSchema();
    PointBuffer data(schema, 1000);

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator(data));
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1000u);

    }

    pdal::filters::iterators::sequential::Stats* iterator = static_cast<pdal::filters::iterators::sequential::Stats*>(iter.get());

    const pdal::filters::stats::Summary& statsX = iterator->getStats(schema.getDimension("X"));
    const pdal::filters::stats::Summary& statsY = iterator->getStats(schema.getDimension("Y"));
    const pdal::filters::stats::Summary& statsZ = iterator->getStats(schema.getDimension("Z"));

    BOOST_CHECK_EQUAL(statsX.count(), 1000u);
    BOOST_CHECK_EQUAL(statsY.count(), 1000u);
    BOOST_CHECK_EQUAL(statsZ.count(), 1000u);

    BOOST_CHECK_CLOSE(statsX.minimum(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.minimum(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.minimum(), 3.0, 0.0001);

    BOOST_CHECK_CLOSE(statsX.maximum(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.maximum(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.maximum(), 3.0, 0.0001);

    BOOST_CHECK_CLOSE(statsX.average(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.average(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.average(), 3.0, 0.0001);

    return;
}




BOOST_AUTO_TEST_CASE(test_random_iterator)
{
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");

    pdal::filters::Stats filter(reader, Options::none());    
    filter.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer data(schema, 3);

    pdal::StageRandomIterator* iter = reader.createRandomIterator(data);

    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    // Can we seek it? Yes, we can!
    iter->seek(100);
    {
        BOOST_CHECK(iter->getIndex() == 100);
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p100_p101_p102(data);
    }

    // Can we seek to beginning? Yes, we can!
    iter->seek(0);
    {
        BOOST_CHECK(iter->getIndex() == 0);
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_multiple_dims_same_name)
{
    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    const pdal::SpatialReference out_ref(epsg4326_wkt);


    pdal::Options options;
    
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");
    // options.add(debug);
    // options.add(verbose);
    pdal::Option out_srs("out_srs",out_ref.getWKT(), "Output SRS to reproject to");
    pdal::Option spatialreference("spatialreference","EPSG:2993", "Output SRS to reproject to");
    pdal::Option x_dim("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    pdal::Option y_dim("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    pdal::Option z_dim("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");

    pdal::Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    pdal::Option ignore("ignore_old_dimensions", false, "");
    options.add(out_srs);
    options.add(x_dim);
    options.add(y_dim);
    options.add(z_dim);
    options.add(x_scale);
    options.add(y_scale);
    options.add(spatialreference);
    options.add(filename);
    options.add(ignore);

    pdal::drivers::las::Reader reader(options);
    pdal::filters::InPlaceReprojection reprojectionFilter(reader, options);
    pdal::filters::Stats filter(reprojectionFilter,options);    
    filter.initialize();


    const pdal::Schema& schema = filter.getSchema();
    pdal::PointBuffer data(schema, 1000u);
    

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator(data));
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1000u);

    }

    pdal::filters::iterators::sequential::Stats* iterator = static_cast<pdal::filters::iterators::sequential::Stats*>(iter.get());

    const pdal::filters::stats::Summary& statsX = iterator->getStats(schema.getDimension("X"));
    const pdal::filters::stats::Summary& statsY = iterator->getStats(schema.getDimension("Y"));
    const pdal::filters::stats::Summary& statsZ = iterator->getStats(schema.getDimension("Z"));

    BOOST_CHECK_EQUAL(statsX.count(), 1000u);
    BOOST_CHECK_EQUAL(statsY.count(), 1000u);
    BOOST_CHECK_EQUAL(statsZ.count(), 1000u);
    
    pdal::Metadata m = iterator->toMetadata();

    return;
}



BOOST_AUTO_TEST_CASE(test_specified_stats)
{

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    const pdal::SpatialReference out_ref(epsg4326_wkt);

    pdal::Options options;
    
    pdal::Option dimensions("dimensions", "X,drivers.las.reader.Y Z filters.inplacereprojection.X", "");

    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");
    // options.add(debug);
    // options.add(verbose);
    pdal::Option out_srs("out_srs",out_ref.getWKT(), "Output SRS to reproject to");
    pdal::Option spatialreference("spatialreference","EPSG:2993", "Output SRS to reproject to");
    pdal::Option x_dim("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    pdal::Option y_dim("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    pdal::Option z_dim("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");

    pdal::Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    pdal::Option ignore("ignore_old_dimensions", false, "");
    options.add(out_srs);
    options.add(x_dim);
    options.add(y_dim);
    options.add(z_dim);
    options.add(x_scale);
    options.add(y_scale);
    options.add(spatialreference);
    options.add(filename);
    options.add(ignore);
    options.add(dimensions);

    pdal::drivers::las::Reader reader(options);

    pdal::filters::InPlaceReprojection reprojectionFilter(reader, options);
    pdal::filters::Stats filter(reprojectionFilter,options);
    filter.initialize();


    const pdal::Schema& schema = filter.getSchema();
    pdal::PointBuffer data(schema, 1000u);
    

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator(data));
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1000u);

    }

    pdal::filters::iterators::sequential::Stats* iterator = static_cast<pdal::filters::iterators::sequential::Stats*>(iter.get());

    const pdal::filters::stats::Summary& statsX = iterator->getStats(schema.getDimension("filters.inplacereprojection.X"));
    const pdal::filters::stats::Summary& statsY = iterator->getStats(schema.getDimension("drivers.las.reader.Y"));
    const pdal::filters::stats::Summary& statsZ = iterator->getStats(schema.getDimension("Z"));

    BOOST_CHECK_EQUAL(statsX.count(), 1000u);
    BOOST_CHECK_EQUAL(statsY.count(), 1000u);
    BOOST_CHECK_EQUAL(statsZ.count(), 1000u);
    
    BOOST_CHECK_CLOSE(statsX.minimum(), -117.2686466233, 0.0001);
    BOOST_CHECK_CLOSE(statsY.minimum(), 848899.700, 0.0001);    


    return;
}

BOOST_AUTO_TEST_CASE(test_pointbuffer_stats)
{

    const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    const pdal::SpatialReference out_ref(epsg4326_wkt);

    pdal::Options options;
    
    pdal::Option dimensions("dimensions", "X,drivers.las.reader.Y Z filters.inplacereprojection.X, Classification", "");
    pdal::Option exact_dimensions("exact_dimensions", "Classification", "");

    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");
    // options.add(debug);
    // options.add(verbose);
    pdal::Option out_srs("out_srs",out_ref.getWKT(), "Output SRS to reproject to");
    pdal::Option spatialreference("spatialreference","EPSG:2993", "Output SRS to reproject to");
    pdal::Option x_dim("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    pdal::Option y_dim("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    pdal::Option z_dim("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data in the case when 'X' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");
    pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data in the case when 'Y' dimension data are to be scaled.  Defaults to '1.0'.  If not set, the Dimensions's scale will be used");

    pdal::Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    pdal::Option ignore("ignore_old_dimensions", false, "");
    options.add(out_srs);
    options.add(x_dim);
    options.add(y_dim);
    options.add(z_dim);
    options.add(x_scale);
    options.add(y_scale);
    options.add(spatialreference);
    options.add(filename);
    options.add(ignore);
    options.add(dimensions);
    options.add(exact_dimensions);

    pdal::drivers::las::Reader reader(options);

    pdal::filters::InPlaceReprojection reprojectionFilter(reader, options);
    pdal::filters::Stats filter(reprojectionFilter,options);
    filter.initialize();


    const pdal::Schema& schema = filter.getSchema();
    pdal::PointBuffer data(schema, 1000u);
    

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator(data));
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1000u);

    }
    
    pdal::Metadata m = data.getMetadata();
    
    BOOST_CHECK_EQUAL(m.toPTree().get<int>("metadata.filters_stats.metadata.Classification.metadata.counts.metadata.count-1.metadata.value.value"), 1);


    return;
}

BOOST_AUTO_TEST_SUITE_END()
