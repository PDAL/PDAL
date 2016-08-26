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

#include <pdal/pdal_test_main.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Algorithm.hpp>

#include "Support.hpp"
#include "../io/GreyhoundReader.hpp"

using namespace pdal;

namespace { // anonymous

std::string ROOT_URL("http://data.greyhound.io");

Options getGreyhoundOptions()
{
    Options options;

    options.add(Option("url", ROOT_URL));

    // Grab just the stadium area
    options.add(Option("bounds", "([638404.60,638895.46],[852818.85,853379.15], [-1,1000])"));
    options.add(Option("resource", "autzen-h"));
    options.add(Option("depth_begin", 1));
    options.add(Option("depth_end", 20));
    options.add(Option("timeout", 50000));
    options.add(Option("debug", true));
    options.add(Option("verbose", 8));

    return options;
}

}

class GreyhoundReaderTest : public testing::Test
{
public:
    GreyhoundReaderTest() : m_bSkipTests(false) {};
protected:
    virtual void SetUp()
    {
        arbiter::Arbiter a;
        try
        {
            a.get(ROOT_URL+"/resource/nyc-h/info");
            m_bSkipTests = false;
        } catch (arbiter::ArbiterError&)
        {
            m_bSkipTests = true;
        }
    }


    virtual void TearDown()
    {
    }

    bool shouldSkipTests() const { return m_bSkipTests; }
private:

    bool m_bSkipTests;
};


TEST_F(GreyhoundReaderTest, read)
{
    if (shouldSkipTests())
    {
        return;
    }

    pdal::GreyhoundReader reader;

    reader.setOptions(getGreyhoundOptions());
    pdal::PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 13874u);
//
    int position (10);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(pdal::Dimension::Id::X, position), 637472.70000000007);

}

TEST_F(GreyhoundReaderTest, quick)
{
    if (shouldSkipTests())
    {
        return;
    }

    pdal::GreyhoundReader reader;

    reader.setOptions(getGreyhoundOptions());
    pdal::QuickInfo qi = reader.preview();
    EXPECT_EQ(qi.m_pointCount, 5183374u);

    BOX3D bounds = qi.m_bounds;
    EXPECT_DOUBLE_EQ(bounds.minx, 635577.79000000004);
    EXPECT_DOUBLE_EQ(bounds.miny, 848882.15000000002);
    EXPECT_DOUBLE_EQ(bounds.minz, 406.13999999999999);
    EXPECT_DOUBLE_EQ(bounds.maxx, 639003.72999999998);
    EXPECT_DOUBLE_EQ(bounds.maxy, 853537.66000000003);
    EXPECT_DOUBLE_EQ(bounds.maxz, 615.25999999999999);


}

