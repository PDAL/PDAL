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

#include <nlohmann/json.hpp>

#include "Support.hpp"
#include "../io/GreyhoundReader.hpp"
#include "../io/bounds.hpp"

using namespace pdal;

namespace
{

const std::string server("http://data.greyhound.io");
const std::string resource("dev/ellipsoid-multi-nyc");
const greyhound::Point center(-8242596.04, 4966606.26);
const greyhound::Point radius(151, 101, 51);
const greyhound::Bounds full(center - radius, center + radius);

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

TEST_F(GreyhoundReaderTest, tileQuery)
{
    if (!doTests()) return;

    pdal::GreyhoundReader reader;

    Options opts;
    opts.add("tile_path", "neu.laz");
    reader.setOptions(opts);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 12593u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(p >= center) << p;
    }
}

TEST_F(GreyhoundReaderTest, boundsQuery)
{
    if (!doTests()) return;

    const greyhound::Bounds bounds(center - 100, center + 100);

    Options opts;
    opts.add("bounds", bounds.toJson());

    pdal::GreyhoundReader reader;
    reader.setOptions(opts);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 66647u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(bounds.contains(p));
    }
}

TEST_F(GreyhoundReaderTest, boundsAndDepthQuery)
{
    if (!doTests()) return;

    const greyhound::Bounds bounds(center - 100, center + 100);

    pdal::GreyhoundReader reader;
    Options opts;
    opts.add("bounds", bounds.toJson());
    opts.add("depth_begin", 7);
    opts.add("depth_end", 8);
    reader.setOptions(opts);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 13854u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(bounds.contains(p));
    }
}

TEST_F(GreyhoundReaderTest, filter)
{
    if (!doTests()) return;

    const greyhound::Bounds bounds(center - 100, center + 100);

    auto run([&bounds](const Options& opts)
    {
        pdal::GreyhoundReader reader;
        reader.setOptions(opts);

        pdal::PointTable table;
        reader.prepare(table);
        PointViewSet viewSet = reader.execute(table);
        PointViewPtr view = *viewSet.begin();
        ASSERT_EQ(view->size(), 33423u);

        greyhound::Point p;
        for (std::size_t i(0); i < view->size(); ++i)
        {
            p.x = view->getFieldAs<double>(Dimension::Id::X, i);
            p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
            p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

            ASSERT_TRUE(bounds.contains(p));
            ASSERT_GE(p.x, center.x);
        }
    });

    // Filter should work both as a JSON object and a stringified JSON object.
    NL::json filter {{ "X", {{ "$gte", center.x }} }};
    {
        Options opts;
        opts.add("bounds", bounds.toJson());
        opts.add("filter", filter);
        run(opts);
    }

    {
        Options opts;
        opts.add("bounds", bounds.toJson());
        opts.add("filter", filter.dump());
        run(opts);
    }
}

TEST_F(GreyhoundReaderTest, singleOption)
{
    if (!doTests()) return;

    const greyhound::Bounds bounds(center - 100, center + 100);
    NL::json filter {{ "X", {{ "$gte", center.x }} }};

    pdal::GreyhoundReader reader;

    pdal::Options options;
    options.add("url", server + "/resource/" + resource + "/read" +
            "?bounds=" + bounds.toJson().dump() +
            "&filter=" + filter.dump() +
            "&depth=7");
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    ASSERT_EQ(view->size(), 7008u);

    greyhound::Point p;
    for (std::size_t i(0); i < view->size(); ++i)
    {
        p.x = view->getFieldAs<double>(Dimension::Id::X, i);
        p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
        p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

        ASSERT_TRUE(bounds.contains(p));
        ASSERT_GE(p.x, center.x);
    }
}



