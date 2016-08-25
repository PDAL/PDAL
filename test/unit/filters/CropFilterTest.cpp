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
#include <BufferReader.hpp>
#include <CropFilter.hpp>
#include <FauxReader.hpp>
#include <LasReader.hpp>
#include <ReprojectionFilter.hpp>
#include <StatsFilter.hpp>
#include <StreamCallbackFilter.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(CropFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.crop"));
    EXPECT_TRUE(filter);
}

TEST(CropFilterTest, test_crop)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    Options opts;
    opts.add("bounds", srcBounds);
    opts.add("count", 1000);
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
    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(ops1);

    Options options;
    Option debug("debug", true);
    Option verbose("verbose", 9);

    std::istream* wkt_stream =
        FileUtils::openFile(Support::datapath("autzen/autzen-selection.wkt"));

    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();

    std::string wkt(strbuf.str());

    Option polygon("polygon", wkt);
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
}

TEST(CropFilterTest, test_crop_polygon_reprojection)
{
    Options readOptions;

    readOptions.add("filename", Support::datapath("las/1.2-with-color.las"));
    readOptions.add("spatialreference", Support::datapath("autzen/autzen-srs.wkt"));

    LasReader reader;
    reader.setOptions(readOptions);

    Options reproOptions;
    reproOptions.add("out_srs", "EPSG:4326");

    ReprojectionFilter reprojection;
    reprojection.setOptions(reproOptions);
    reprojection.setInput(reader);

    std::istream* wkt_stream = FileUtils::openFile(
        Support::datapath("autzen/autzen-selection-dd.wkt"));
    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();
    std::string wkt(strbuf.str());

    Options cropOptions;
    cropOptions.add("polygon", wkt);

    CropFilter crop;
    crop.setOptions(cropOptions);
    crop.setInput(reprojection);

    PointTable table;
    PointViewPtr view(new PointView(table));
    crop.prepare(table);
    PointViewSet viewSet = crop.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 47u);

    FileUtils::closeFile(wkt_stream);
}

TEST(CropFilterTest, multibounds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDim(Id::X);
    table.layout()->registerDim(Id::Y);
    table.layout()->registerDim(Id::Z);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 2);
    view->setField(Id::Y, 0, 2);

    view->setField(Id::X, 1, 4);
    view->setField(Id::Y, 1, 2);

    view->setField(Id::X, 2, 6);
    view->setField(Id::Y, 2, 2);

    view->setField(Id::X, 3, 8);
    view->setField(Id::Y, 3, 2);

    view->setField(Id::X, 4, 10);
    view->setField(Id::Y, 4, 2);

    view->setField(Id::X, 5, 12);
    view->setField(Id::Y, 5, 2);

    BufferReader r;
    r.addView(view);

    CropFilter crop;
    Options o;
    o.add("bounds", "([1, 3], [1, 3])");
    o.add("bounds", "([5, 7], [1, 3])");
    o.add("polygon", "POLYGON ((9 1, 11 1, 11 3, 9 3, 9 1))");
    crop.setInput(r);
    crop.setOptions(o);

    crop.prepare(table);
    PointViewSet s = crop.execute(table);
    // Make sure we get three views, one with the point 2, 2, one with the
    // point 6, 2 and one with 10,2.
    EXPECT_EQ(s.size(), 3u);
    static int total_cnt = 0;
    for (auto v : s)
    {
        int cnt = 0;
        EXPECT_EQ(v->size(), 1u);
        double x = v->getFieldAs<double>(Dimension::Id::X, 0);
        double y = v->getFieldAs<double>(Dimension::Id::Y, 0);
        if (x == 2 && y == 2)
            cnt = 1;
        if (x == 6 && y == 2)
            cnt = 2;
        if (x == 10 && y == 2)
            cnt = 4;
        EXPECT_TRUE(cnt > 0);
        total_cnt += cnt;
    }
    EXPECT_EQ(total_cnt, 7);
}

TEST(CropFilterTest, stream)
{
    using namespace Dimension;

    FixedPointTable table(2);
    table.layout()->registerDim(Id::X);
    table.layout()->registerDim(Id::Y);
    table.layout()->registerDim(Id::Z);

    class StreamReader : public Reader
    {
    public:
        std::string getName() const
            { return "readers.stream"; }
        bool processOne(PointRef& point)
        {
            static int i = 0;

            if (i == 0)
            {
                point.setField(Id::X, 2);
                point.setField(Id::Y, 2);
            }
            else if (i == 1)
            {
                point.setField(Id::X, 6);
                point.setField(Id::Y, 2);
            }
            else if (i == 2)
            {
                point.setField(Id::X, 8);
                point.setField(Id::Y, 2);
            }
            else if (i == 3)
            {
                point.setField(Id::X, 10);
                point.setField(Id::Y, 2);
            }
            else if (i == 4)
            {
                point.setField(Id::X, 12);
                point.setField(Id::Y, 2);
            }
            else
                return false;
            i++;
            return true;
        }
    };

    StreamReader r;

    CropFilter crop;
    Options o;
    o.add("bounds", "([1, 3], [1, 3])");
    o.add("bounds", "([5, 7], [1, 3])");
    o.add("polygon", "POLYGON ((9 1, 11 1, 11 3, 9 3, 9 1))");
    crop.setInput(r);
    crop.setOptions(o);

    auto cb = [](PointRef& point)
    {
        static int i = 0;
        if (i == 0)
        {
            EXPECT_EQ(point.getFieldAs<int>(Id::X), 2);
            EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
        }
        if (i == 1)
        {
            EXPECT_EQ(point.getFieldAs<int>(Id::X), 6);
            EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
        }
        if (i == 2)
        {
            EXPECT_EQ(point.getFieldAs<int>(Id::X), 10);
            EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
        }
        else
            return false;
        i++;
        return true;
    };

    StreamCallbackFilter f;
    f.setCallback(cb);
    f.setInput(crop);

    f.prepare(table);
    f.execute(table);
}

