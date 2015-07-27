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

#include <pdal/pdal_test_main.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <CropFilter.hpp>
#include <FauxReader.hpp>
#include <LasReader.hpp>
#include <ReprojectionFilter.hpp>
#include <StatsFilter.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(CropFilterTest, create)
{
    StageFactory f;
    std::unique_ptr<Stage> filter(f.createStage("filters.crop"));
    EXPECT_TRUE(filter.get());
}

TEST(CropFilterTest, test_crop)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    Options opts;
    opts.add("bounds", srcBounds);
    opts.add("num_points", 1000);
    opts.add("mode", "ramp");
    FauxReader reader;
    reader.setOptions(opts);

    // crop the window to 1/3rd the size in each dimension
    BOX2D dstBounds(3.33333, 33.33333, 6.66666, 66.66666);
    Options cropOpts;
    cropOpts.add("bounds", dstBounds);

    CropFilter filter;
    filter.setOptions(cropOpts);
    filter.setInput(reader);

    Options statOpts;

    StatsFilter stats;
    stats.setOptions(statOpts);
    stats.setInput(filter);

    PointTable table;
    stats.prepare(table);
    PointViewSet viewSet = stats.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr buf = *viewSet.begin();

    const stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);
    EXPECT_EQ(buf->size(), 333u);

    const double minX = statsX.minimum();
    const double minY = statsY.minimum();
    const double minZ = statsZ.minimum();
    const double maxX = statsX.maximum();
    const double maxY = statsY.maximum();
    const double maxZ = statsZ.maximum();
    const double avgX = statsX.average();
    const double avgY = statsY.average();
    const double avgZ = statsZ.average();

    const double delX = 10.0 / 999.0 * 100.0;
    const double delY = 100.0 / 999.0 * 100.0;
    const double delZ = 1000.0 / 999.0 * 100.0;

    EXPECT_NEAR(minX, 3.33333, delX);
    EXPECT_NEAR(minY, 33.33333, delY);
    EXPECT_NEAR(minZ, 333.33333, delZ);
    EXPECT_NEAR(maxX, 6.66666, delX);
    EXPECT_NEAR(maxY, 66.66666, delY);
    EXPECT_NEAR(maxZ, 666.66666, delZ);
    EXPECT_NEAR(avgX, 5.00000, delX);
    EXPECT_NEAR(avgY, 50.00000, delY);
    EXPECT_NEAR(avgZ, 500.00000, delZ);
}


TEST(CropFilterTest, test_crop_polygon)
{
#ifdef PDAL_HAVE_GEOS
    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(ops1);

    Options options;
    Option debug("debug", true, "");
    Option verbose("verbose", 9, "");

    std::istream* wkt_stream =
        FileUtils::openFile(Support::datapath("autzen/autzen-selection.wkt"));

    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();

    std::string wkt(strbuf.str());

    Option polygon("polygon", wkt, "");
    options.add(polygon);

    CropFilter crop;
    crop.setInput(reader);
    crop.setOptions(options);

    PointTable table;

    crop.prepare(table);
    PointViewSet viewSet = crop.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 47u);

    FileUtils::closeFile(wkt_stream);
#endif
}

TEST(CropFilterTest, test_crop_polygon_reprojection)
{
#ifdef PDAL_HAVE_GEOS
    Options options;

    Option in_srs("spatialreference",Support::datapath("autzen/autzen-srs.wkt"), "Input SRS");
    Option out_srs("out_srs","EPSG:4326", "Output SRS to reproject to");
    Option x_dim("x_dim", std::string("readers.las.X"),
        "Dimension name to use for 'X' data");
    Option y_dim("y_dim", std::string("readers.las.Y"),
        "Dimension name to use for 'Y' data");
    Option z_dim("z_dim", std::string("readers.las.Z"),
        "Dimension name to use for 'Z' data");
    Option x_scale("scale_x", 0.0000001f, "Scale for output X data "
        "in the case when 'X' dimension data are to be scaled.  Defaults "
        "to '1.0'.  If not set, the Dimensions's scale will be used");
    Option y_scale("scale_y", 0.0000001f, "Scale for output Y data "
        "in the case when 'Y' dimension data are to be scaled.  Defaults "
        "to '1.0'.  If not set, the Dimensions's scale will be used");
    Option filename("filename", Support::datapath("las/1.2-with-color.las"));
    Option debug("debug", true, "");
    Option verbose("verbose", 9, "");
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

    std::istream* wkt_stream = FileUtils::openFile(
        Support::datapath("autzen/autzen-selection-dd.wkt"));
    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();
    std::string wkt(strbuf.str());
    Option polygon("polygon", wkt, "");
    options.add(polygon);

    LasReader reader;
    reader.setOptions(options);

    ReprojectionFilter reprojection;
    reprojection.setOptions(options);
    reprojection.setInput(reader);

    CropFilter crop;
    crop.setOptions(options);
    crop.setInput(reprojection);

    PointTable table;
    PointViewPtr view(new PointView(table));
    crop.prepare(table);
    PointViewSet viewSet = crop.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 47u);

    FileUtils::closeFile(wkt_stream);
#endif
}

/**
TEST(CropFilterTest, multibounds)
{
    using namespace Dimension;

    PointTable table;
    table.layout->registerDim(Id::X);
    table.layout->registerDim(Id::Y);
    table.layout->registerDim(Id::Z);

    PointView view(table);
    view.setField(Id::X, 0, 1);
    view.setField(Id::Y, 0, 1);

    view.setField(Id::X, 1, 2);
    view.setField(Id::Y, 1, 6);

    view.setField(Id::X, 2, 4);
    view.setField(Id::Y, 2, 4);

    BOX3D p
}
**/
