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

//ABELL
/**
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>
**/
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>
#include "Support.hpp"
#include "../StageTester.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(CropFilterTest)

//ABELL
// Need faux reader
/**
BOOST_AUTO_TEST_CASE(test_crop)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);

    // crop the window to 1/3rd the size in each dimension
    Bounds<double> dstBounds(3.33333, 33.33333, 333.33333, 6.66666, 66.66666, 666.66666);

    pdal::drivers::faux::Reader reader(srcBounds, 1000, pdal::drivers::faux::Reader::Ramp);

    pdal::filters::Crop filter(dstBounds);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Crop Filter");
    pdal::drivers::faux::Writer writer(Options::none());
    writer.setInput(&filter);
    writer.prepare();

    boost::uint64_t numWritten = writer.write(1000);
    BOOST_CHECK_EQUAL(numWritten, 333u);

    // 1000 * 1/3 = 333, plus or minus a bit for rounding
    BOOST_CHECK(Utils::compare_approx<double>(static_cast<double>(numWritten), 333, 6));

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();
    const double avgX = writer.getAvgX();
    const double avgY = writer.getAvgY();
    const double avgZ = writer.getAvgZ();

    const double delX = 10.0 / 999.0 * 100.0;
    const double delY = 100.0 / 999.0 * 100.0;
    const double delZ = 1000.0 / 999.0 * 100.0;

    BOOST_CHECK_CLOSE(minX, 3.33333, delX);
    BOOST_CHECK_CLOSE(minY, 33.33333, delY);
    BOOST_CHECK_CLOSE(minZ, 333.33333, delZ);
    BOOST_CHECK_CLOSE(maxX, 6.66666, delX);
    BOOST_CHECK_CLOSE(maxY, 66.66666, delY);
    BOOST_CHECK_CLOSE(maxZ, 666.66666, delZ);
    BOOST_CHECK_CLOSE(avgX, 5.00000, delX);
    BOOST_CHECK_CLOSE(avgY, 50.00000, delY);
    BOOST_CHECK_CLOSE(avgZ, 500.00000, delZ);
}
**/


BOOST_AUTO_TEST_CASE(test_crop_polygon)
{
    using namespace pdal;

#ifdef PDAL_HAVE_GEOS
    drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));

    Options options;
    Option debug("debug", true, "");
    Option verbose("verbose", 9, "");
    // options.add(debug);
    // options.add(verbose);
    
    std::istream* wkt_stream =
        FileUtils::openFile(Support::datapath("autzen-selection.wkt"));

    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();

    std::string wkt(strbuf.str());

    Option polygon("polygon", wkt, "");
    options.add(polygon);

    filters::Crop crop(options);
    crop.setInput(&reader);

    PointContext ctx;

    PointBufferPtr buffer(new PointBuffer(ctx));
    crop.prepare(ctx);

    StageSequentialIterator *iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(*buffer);
    StageTester::ready(&crop, ctx);
    PointBufferSet pbSet = StageTester::run(&crop, buffer);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buffer = *pbSet.begin();
    BOOST_CHECK_EQUAL(buffer->size(), 47u);
    
    delete iter;
    FileUtils::closeFile(wkt_stream);
#endif
}

BOOST_AUTO_TEST_CASE(test_crop_polygon_reprojection)
{
    using namespace pdal;

#ifdef PDAL_HAVE_GEOS
    Options options;

    Option in_srs("spatialreference","EPSG:2993", "Input SRS");
    Option out_srs("out_srs","EPSG:4326", "Output SRS to reproject to");
    Option x_dim("x_dim", std::string("drivers.las.reader.X"),
        "Dimension name to use for 'X' data");
    pdal::Option y_dim("y_dim", std::string("drivers.las.reader.Y"),
        "Dimension name to use for 'Y' data");
    pdal::Option z_dim("z_dim", std::string("drivers.las.reader.Z"),
        "Dimension name to use for 'Z' data");
    pdal::Option x_scale("scale_x", 0.0000001f, "Scale for output X data "
        "in the case when 'X' dimension data are to be scaled.  Defaults "
        "to '1.0'.  If not set, the Dimensions's scale will be used");
    pdal::Option y_scale("scale_y", 0.0000001f, "Scale for output Y data "
        "in the case when 'Y' dimension data are to be scaled.  Defaults "
        "to '1.0'.  If not set, the Dimensions's scale will be used");
    pdal::Option filename("filename", Support::datapath("1.2-with-color.las"));
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 9, "");
    // options.add(debug);
    // options.add(verbose);
    options.add(in_srs);
    options.add(out_srs);
    options.add(x_dim);
    options.add(y_dim);
    options.add(z_dim);
    options.add(x_scale);
    options.add(y_scale);
    options.add(filename);
    
    std::istream* wkt_stream =
        FileUtils::openFile(Support::datapath("autzen-selection.wkt"));
    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();
    std::string wkt(strbuf.str());
    Option polygon("polygon", wkt, "");
    options.add(polygon);

    drivers::las::Reader reader(options);
    filters::InPlaceReprojection reprojection(options);
    reprojection.setInput(&reader);
    filters::Crop crop(options);
    crop.setInput(&reprojection);

    PointContext ctx;
    PointBufferPtr buffer(new PointBuffer(ctx));
    crop.prepare(ctx);

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(*buffer);

    FilterTester::ready(&reprojection, ctx);
    FilterTester::filter(&reprojection, *buffer);
    FilterTester::done(&reprojection, ctx);

    StageTester::ready(&crop, ctx);
    PointBufferSet pbSet = StageTester::run(&crop, buffer);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buffer = *pbSet.begin();
    BOOST_CHECK_EQUAL(buffer->size(), 47u);
    
    delete iter;
    
    FileUtils::closeFile(wkt_stream);
    
#endif
}

BOOST_AUTO_TEST_SUITE_END()
