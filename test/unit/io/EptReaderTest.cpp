/******************************************************************************
 * Copyright (c) 2018, Connor Manning (connor@hobu.co)
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

#include <algorithm>

#include <pdal/pdal_test_main.hpp>

#include <io/EptReader.hpp>
#include <io/LasReader.hpp>
#include <filters/CropFilter.hpp>
#include "Support.hpp"

using namespace pdal;

namespace
{
    const BOX3D expBoundsConforming(515368, 4918340, 2322,
            515402, 4918382, 2339);
    const std::string expSrsWkt("GEOCCS[\"unnamed\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"unknown\",1]]");
    const point_count_t expNumPoints(518862);
    const std::vector<std::string> expDimNames = {
         "X", "Y", "Z", "Intensity", "ReturnNumber", "NumberOfReturns",
         "ScanDirectionFlag", "EdgeOfFlightLine", "Classification",
         "ScanAngleRank", "UserData", "PointSourceId", "GpsTime", "OriginId"
    };
}

TEST(EptReaderTest, inspect)
{
    Options options;
    options.add("filename", "ept://" + Support::datapath("ept/ept-star"));

    EptReader reader;
    reader.setOptions(options);

    const QuickInfo qi(reader.preview());

    EXPECT_TRUE(qi.valid());
    EXPECT_EQ(qi.m_bounds, expBoundsConforming);
    EXPECT_EQ(qi.m_srs.getWKT(), expSrsWkt);
    EXPECT_EQ(qi.m_pointCount, expNumPoints);
    EXPECT_TRUE(std::equal(qi.m_dimNames.cbegin(), qi.m_dimNames.cend(),
                expDimNames.cbegin()));
}

TEST(EptReaderTest, fullRead)
{
    Options options;
    options.add("filename", "ept://" + Support::datapath("ept/ept-star"));

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(expBoundsConforming.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, expNumPoints);
}

TEST(EptReaderTest, resolutionLimit)
{
    Options options;
    options.add("filename", "ept://" + Support::datapath("ept/ept-star"));

    // Our test data cube is 44 units in length, with a span of 128.  Therefore
    // our resolution cell width values for the first few depths are:
    //      Depth 0: 0.34375
    //      Depth 1: 0.171875
    //      Depth 2: 0.0859375
    //
    // Any resolution option between 0.171875 and 0.0859375 will select all of
    // depths 0, 1, and 2, so we'll test a corresponding query.
    options.add("resolution", 0.1);

    // This expected value corresponds to the sum of the point counts of all
    // files in our dataset whose depth is less than 3.  This value is summed
    // from the hierarchy for depths 0 through 2 (our test dataset has depths
    // through 3, which are omitted here).
    const point_count_t expectedCount = 303955;

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(expBoundsConforming.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, expectedCount);
}

TEST(EptReaderTest, boundedRead2d)
{
    const std::string boundsString("([515380, 515400], [4918350, 4918370])");
    BOX2D bounds(515380, 4918350, 515400, 4918370);

    // First we'll query the EptReader for these bounds.
    EptReader reader;
    {
        Options options;
        options.add("filename", "ept://" + Support::datapath("ept/ept-star"));
        options.add("bounds", boundsString);
        reader.setOptions(options);
    }
    PointTable eptTable;
    reader.prepare(eptTable);
    const auto set(reader.execute(eptTable));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(bounds.contains(x, y)) << bounds << ": " <<
                x << ", " << y << ", " << z << std::endl;
            ASSERT_TRUE(o < 4);
        }
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", Support::datapath("ept/lone-star.laz"));
        source.setOptions(options);
    }
    CropFilter crop;
    {
        Options options;
        options.add("bounds", bounds);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    for (const PointViewPtr& view : crop.execute(sourceTable))
    {
        sourceNp += view->size();
    }

    EXPECT_EQ(np, sourceNp);
    EXPECT_EQ(np, 354211u);
}

TEST(EptReaderTest, boundedRead3d)
{
    const std::string boundsString(
            "([515380, 515400], [4918350, 4918370], [2320, 2325])");
    BOX3D bounds(515380, 4918350, 2320, 515400, 4918370, 2325);

    // First we'll query the EptReader for these bounds.
    EptReader reader;
    {
        Options options;
        options.add("filename", "ept://" + Support::datapath("ept/ept-star"));
        options.add("bounds", boundsString);
        reader.setOptions(options);
    }
    PointTable eptTable;
    reader.prepare(eptTable);
    const auto set(reader.execute(eptTable));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(bounds.contains(x, y, z)) << bounds << ": " <<
                x << ", " << y << ", " << z << std::endl;
            ASSERT_TRUE(o < 4);
        }
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", Support::datapath("ept/lone-star.laz"));
        source.setOptions(options);
    }
    CropFilter crop;
    {
        Options options;
        options.add("bounds", bounds);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    // The crop filter only works in 2D, so we'll have to manually count.
    for (const PointViewPtr& view : crop.execute(sourceTable))
    {
        for (uint64_t i(0); i < view->size(); ++i)
        {
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            if (bounds.contains(x, y, z))
                ++sourceNp;
        }
    }

    EXPECT_EQ(np, sourceNp);
    EXPECT_EQ(np, 45930u);
}

TEST(EptReaderTest, originRead)
{
    uint64_t np(0);
    for (uint64_t origin(0); origin < 4; ++origin)
    {
        EptReader reader;
        Options options;
        options.add("filename", "ept://" + Support::datapath("ept/ept-star"));
        options.add("origin", origin);
        reader.setOptions(options);
        PointTable table;
        reader.prepare(table);
        const auto set(reader.execute(table));

        uint64_t o;
        for (const PointViewPtr& view : set)
        {
            np += view->size();
            for (point_count_t i(0); i < view->size(); ++i)
            {
                o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
                ASSERT_EQ(o, origin);
            }
        }
    }

    EXPECT_EQ(np, expNumPoints);
}

TEST(EptReaderTest, badOriginQuery)
{
    EptReader reader;
    Options options;
    options.add("filename", "ept://" + Support::datapath("ept/ept-star"));
    options.add("origin", 4);
    reader.setOptions(options);
    PointTable table;
    EXPECT_THROW(reader.prepare(table), pdal_error);
}

