/******************************************************************************
* Copyright (c) 2022, Daniel Brookes (dbrookes@micromine.com)
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
*     * Neither the name of Hobu, Inc. nor the
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
#include <pdal/StageFactory.hpp>
#include <io/PtxReader.hpp>
#include "Support.hpp"

namespace pdal
{

TEST(PtxReader, Basic)
{
    PtxReader reader;

    Options options;
    options.add("filename", Support::datapath("ptx/1.2-with-color.ptx"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    const PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), static_cast<size_t>(1));
    const PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), static_cast<point_count_t>(15 * 71));

    const PointLayout *layout = view->layout();
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Intensity));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Red));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Green));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Blue));

    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, 0), 637012.24);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, 0), 849028.31);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, 0), 431.66);
    // Intensity = 0.034912 * 4096 ~= 143.0
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Intensity, 0), 143.0f);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Red, 0), 68);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Green, 0), 77);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Blue, 0), 88);

    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, 489), 635770.47);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, 489), 851464.67);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, 489), 422.28);
    // Intensity = 0.019775 * 4096 ~= 81.0
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Intensity, 489), 81.0f);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Red, 489), 105);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Green, 489), 85);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Blue, 489), 114);
}

TEST(PtxReader, DiscardMissingPointsWithComplexTransform)
{
    PtxReader reader;

    Options options;
    options.add("filename", Support::datapath("ptx/complex-transform.ptx"));
    options.add("discard_missing_points", true);
    reader.setOptions(options);

    // Since we are discarding missing points, the first point will be the fifth
    // point, the first not missing point in the input file. There are only four
    // not missing points in the input file so thats how many we expect to read.

    PointTable table;
    reader.prepare(table);
    const PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), static_cast<size_t>(1));
    const PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), static_cast<point_count_t>(4));

    const PointLayout *layout = view->layout();
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Red));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Green));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Blue));

    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::X, 0), -3.034408f);
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Y, 0), -3.173781f);
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Z, 0), -1.823750f);
    // Intensity = 0.494911 * 4096 ~= 2027.155456
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Intensity, 0), 2027);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Red, 0), 33);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Green, 0), 38);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Blue, 0), 24);
}

TEST(PtxReader, MultipleClouds)
{
    StageFactory factory;
    Stage *reader = factory.createStage("readers.ptx");
    EXPECT_TRUE(!!reader);

    Options options;
    options.add("filename", Support::datapath("ptx/multiple-and-transform.ptx"));
    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    const PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), static_cast<size_t>(1));
    const PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), static_cast<point_count_t>(2 * 2 + 4 * 1));

    const PointLayout *layout = view->layout();
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Red));
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Green));
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Blue));

    // The input file has two clouds with the same four points, but the first
    // clout has a transform. We compare them to ensure they're equal.

    for (point_count_t i = 0; i < static_cast<point_count_t>(4); ++i)
    {
        const point_count_t i2 = i + static_cast<point_count_t>(4);

        EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, i),
            view->getFieldAs<double>(Dimension::Id::X, i2));
        EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, i),
            view->getFieldAs<double>(Dimension::Id::Y, i2));
        EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, i),
            view->getFieldAs<double>(Dimension::Id::Z, i2));
    }
}

TEST(PtxReader, NoColor)
{
    PtxReader reader;

    Options options;
    options.add("filename", Support::datapath("ptx/no-color.ptx"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    const PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), static_cast<size_t>(1));
    const PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), static_cast<point_count_t>(15));

    const PointLayout *layout = view->layout();
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Red));
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Green));
    EXPECT_FALSE(layout->hasDim(Dimension::Id::Blue));

    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, 14), 635795.24);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, 14), 849310.43);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, 14), 426.61);
    // Intensity = 0.035645 * 4096 ~= 146.0
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Intensity, 14), 146.0f);
}

}
