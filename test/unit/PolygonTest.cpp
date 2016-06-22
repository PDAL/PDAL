/******************************************************************************
* Copyright (c) 2016, Howard Butler <howard@hobu.co>
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
#include <pdal/Options.hpp>

#include <pdal/Polygon.hpp>
#include "Support.hpp"



namespace pdal
{

std::string getWKT()
{
    std::istream* wkt_stream =
        FileUtils::openFile(Support::datapath("autzen/autzen-selection.wkt"));

    std::stringstream strbuf;
    strbuf << wkt_stream->rdbuf();

    std::string wkt(strbuf.str());

    FileUtils::closeFile(wkt_stream);

    return wkt;
}

std::string getJSON()
{
    std::istream* json_stream =
        FileUtils::openFile(Support::datapath("autzen/autzen-selection.json"));

    std::stringstream strbuf;
    strbuf << json_stream->rdbuf();

    std::string json(strbuf.str());
    FileUtils::closeFile(json_stream);
    return json;
}

TEST(PolygonTest, test_wkt_in)
{
    std::string wkt(getWKT());

    pdal::Polygon p(wkt);
}

TEST(PolygonTest, test_json_in)
{
    std::string json(getJSON());

    pdal::Polygon p(json);
}

TEST(PolygonTest, test_wkt_out)
{

    pdal::Polygon p(getWKT());
    std::string expected("POLYGON ((636889.41295124 851528.51229326, 636899.14233424 851475.00068676, 636899.14233424 851475.00068676, 636928.33048324 851494.45945276, 636928.33048324 851494.45945276, 636928.33048324 851494.45945276, 636976.97739824 851513.91821876, 636976.97739824 851513.91821876, 637069.40653674 851475.00068676, 637132.64752625 851445.81253776, 637132.64752625 851445.81253776, 637336.96456925 851411.75969726, 637336.96456925 851411.75969726, 637473.17593125 851158.79573925, 637589.92852726 850711.24412124, 637244.53543075 850511.79176973, 636758.06628074 850667.46189774, 636539.15516323 851056.63721775, 636889.41295124 851528.51229326))");

    std::string w = p.wkt();
    EXPECT_EQ(expected, w);

}


TEST(PolygonTest, test_json_out)
{
    pdal::Polygon p(getJSON());
    std::string j = p.json(5);
    std::string expected("{ \"type\": \"Polygon\", \"coordinates\": [ [ [ 636889.41295, 851528.51229 ], [ 636899.14233, 851475.00069 ], [ 636899.14233, 851475.00069 ], [ 636928.33048, 851494.45945 ], [ 636928.33048, 851494.45945 ], [ 636928.33048, 851494.45945 ], [ 636976.9774, 851513.91822 ], [ 636976.9774, 851513.91822 ], [ 637069.40654, 851475.00069 ], [ 637132.64753, 851445.81254 ], [ 637132.64753, 851445.81254 ], [ 637336.96457, 851411.7597 ], [ 637336.96457, 851411.7597 ], [ 637473.17593, 851158.79574 ], [ 637589.92853, 850711.24412 ], [ 637244.53543, 850511.79177 ], [ 636758.06628, 850667.4619 ], [ 636539.15516, 851056.63722 ], [ 636889.41295, 851528.51229 ] ] ] }");
    EXPECT_EQ(j, expected);
}

TEST(PolygonTest, smooth)
{
    pdal::Polygon p(getJSON());
    pdal::Polygon p2 = p.simplify(100000.0, 0.0);
    EXPECT_FLOAT_EQ(p.area(), 703744.56f);
    EXPECT_FLOAT_EQ(p2.area(), 492958.44f);
}

TEST(PolygonTest, covers)
{
    using namespace pdal::Dimension;

    pdal::Polygon p(getWKT());

    PointTable table;
    PointLayoutPtr layout(table.layout());
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);


    // Check if the very first point in the polygon is
    // covered by the polyg. Should be true
    //
    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 636889.412951239268295 );
    view->setField(Id::Y, 0, 851528.512293258565478);

    pdal::PointRef ref(*view, 0);
    bool covered = p.covers(ref);
    EXPECT_EQ(covered, true);
}

TEST(PolygonTest, valid)
{
    pdal::Polygon p(getWKT());
    EXPECT_EQ(p.valid(), true);
}

TEST(PolygonTest, bounds)
{
    pdal::Polygon p(getWKT());
    BOX3D b = p.bounds();
    EXPECT_FLOAT_EQ(b.minx, 636539.12f);
    EXPECT_FLOAT_EQ(b.miny, 850511.81f);
    EXPECT_FLOAT_EQ(b.minz, 420.50977f);
    EXPECT_FLOAT_EQ(b.maxx, 637589.94f);
    EXPECT_FLOAT_EQ(b.maxy, 851528.5f);
    EXPECT_FLOAT_EQ(b.maxz, 438.70996f);
}

TEST(PolygonTest, bounds2d)
{
    BOX2D box(636539.12, 850511.81, 637589.94, 851528.5);
    pdal::Polygon p(box);

    BOX3D b = p.bounds();
    EXPECT_FLOAT_EQ(b.minx, 636539.12f);
    EXPECT_FLOAT_EQ(b.miny, 850511.81f);
    EXPECT_FLOAT_EQ(b.minz, 0.0f);
    EXPECT_FLOAT_EQ(b.maxx, 637589.94f);
    EXPECT_FLOAT_EQ(b.maxy, 851528.5f);
    EXPECT_FLOAT_EQ(b.maxz, 0.0f);
}

TEST(PolygonTest, bounds3d)
{
    BOX3D box(636539.12, 850511.81, 420.50977, 637589.94, 851528.5, 438.70996);
    pdal::Polygon p(box);

    BOX3D b = p.bounds();
    EXPECT_FLOAT_EQ(b.minx, 636539.12f);
    EXPECT_FLOAT_EQ(b.miny, 850511.81f);
    EXPECT_FLOAT_EQ(b.minz, 420.50977f);
    EXPECT_FLOAT_EQ(b.maxx, 637589.94f);
    EXPECT_FLOAT_EQ(b.maxy, 851528.5f);
    EXPECT_FLOAT_EQ(b.maxz, 438.70996f);
}


TEST(PolygonTest, streams)
{
    pdal::Polygon p(getWKT());

    std::stringstream ss(std::stringstream::in | std::stringstream::out);

    ss << p;

    pdal::Polygon p2;
    ss >> p2;

    EXPECT_TRUE(p == p2);

}

} // namespace pdal
