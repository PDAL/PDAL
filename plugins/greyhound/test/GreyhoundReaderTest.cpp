/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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

#include <sstream>

#include <pdal/pdal_test_main.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Algorithm.hpp>

#include <arbiter/arbiter.hpp>

#include "Support.hpp"
#include "../io/GreyhoundReader.hpp"
#include "../io/bounds.hpp"

using namespace pdal;

namespace
{

const bool fullTest(false);

const greyhound::Bounds fullBounds(634950,848860,-1830,639650,853560,2870);
const greyhound::Bounds stadiumBounds(637000,851550,-1830,637800,852300,2870);
const greyhound::Bounds patchBounds(637000,851550,-1830,637100,851650,2870);
const greyhound::Bounds originBounds(
        greyhound::Bounds(-92369, 123812, -11170, -22218, 230745, 2226)
        .unscale(.01, greyhound::Point(637300, 851210, 520)));

std::string toString(const greyhound::Bounds& b)
{
    std::ostringstream ss;
    ss << "([" <<
        b.min().x << "," << b.max().x << "],[" <<
        b.min().y << "," << b.max().y << "],[" <<
        b.min().z << "," << b.max().z << "])";
    return ss.str();
}

const std::string server("http://data.greyhound.io");
const std::string resource("dev/autzen-chipped");

Options greyhoundOptions(
        const greyhound::Bounds* b = nullptr,
        const std::size_t depthBegin = 0,
        const std::size_t depthEnd = 0)
{
    Options options;
    options.add("url", server);
    options.add("resource", resource);
    if (b) options.add("bounds", toString(*b));
    if (depthBegin) options.add("depth_begin", depthBegin);
    if (depthEnd) options.add("depth_end", depthEnd);
    return options;
}

pdal::greyhound::Bounds toBounds(const BOX3D& b)
{
    return greyhound::Bounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
}

}

class GreyhoundReaderTest : public testing::Test
{
public:
    GreyhoundReaderTest()
        : m_doTests(false)
    {
        static std::string path(server + "/resource/" + resource + "/info");
        static arbiter::Arbiter a;
        static bool good(a.hasDriver(path) && a.tryGetSize(path));
        m_doTests = good;
    }

protected:
    bool doTests() const { return m_doTests; }
    bool m_doTests;
};

TEST_F(GreyhoundReaderTest, tilePath)
{
    if (!doTests()) return;

    pdal::GreyhoundReader reader;

    auto options(greyhoundOptions(nullptr, 14, 15));
    options.add("tile_path", "c-13.laz");
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 2676u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(originBounds.contains(p));
    }
}

TEST_F(GreyhoundReaderTest, readPatch)
{
    if (!doTests()) return;

    pdal::GreyhoundReader reader;
    reader.setOptions(greyhoundOptions(&stadiumBounds, 14, 15));

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 1880u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(stadiumBounds.contains(p));
    }
}

TEST_F(GreyhoundReaderTest, filter)
{
    if (!doTests()) return;

    {
        pdal::GreyhoundReader reader;
        auto options(greyhoundOptions(&stadiumBounds, 0, 12));
        options.add("filter", "{ \"Z\": { \"$gte\": 500 } }");

        reader.setOptions(options);

        pdal::PointTable table;
        reader.prepare(table);
        PointViewSet viewSet = reader.execute(table);
        PointViewPtr view = *viewSet.begin();
        ASSERT_LT(view->size(), 188260u);

        greyhound::Point p;
        for (std::size_t i(0); i < view->size(); ++i)
        {
            p.x = view->getFieldAs<double>(Dimension::Id::X, i);
            p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
            p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

            ASSERT_TRUE(stadiumBounds.contains(p));
            ASSERT_GE(p.z, 500);
        }
    }

    {
        pdal::GreyhoundReader reader;
        auto options(greyhoundOptions(&stadiumBounds, 0, 12));

        Json::Value filter;
        filter["Z"]["$gte"] = 500;
        options.add("filter", filter);

        reader.setOptions(options);

        pdal::PointTable table;
        reader.prepare(table);
        PointViewSet viewSet = reader.execute(table);
        PointViewPtr view = *viewSet.begin();
        ASSERT_LT(view->size(), 188260u);

        greyhound::Point p;
        for (std::size_t i(0); i < view->size(); ++i)
        {
            p.x = view->getFieldAs<double>(Dimension::Id::X, i);
            p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
            p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

            ASSERT_TRUE(stadiumBounds.contains(p));
            ASSERT_GE(p.z, 500);
        }
    }
}

TEST_F(GreyhoundReaderTest, quickFull)
{
    if (!doTests()) return;

    pdal::GreyhoundReader reader;
    reader.setOptions(greyhoundOptions());

    pdal::QuickInfo qi = reader.preview();
    EXPECT_EQ(qi.m_pointCount, 10653336u);
    EXPECT_EQ(toBounds(qi.m_bounds), fullBounds);
}

TEST_F(GreyhoundReaderTest, quickSplit)
{
    if (!doTests()) return;

    pdal::GreyhoundReader reader;
    reader.setOptions(greyhoundOptions(&patchBounds));

    // The quick-info point count is an upper bound, so it should be at no less
    // than the actual query result.
    pdal::QuickInfo qi = reader.preview();
    EXPECT_GE(qi.m_pointCount, 5141u);
}

TEST_F(GreyhoundReaderTest, full)
{
    if (!doTests() || !fullTest) return;

    pdal::GreyhoundReader reader;
    auto options(greyhoundOptions());
    options.add("threads", 10);
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 10653336u);
}

