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
#include <pdal/StageFactory.hpp>
#include <io/PtsReader.hpp>
#include "Support.hpp"

namespace pdal
{

TEST(PtsReader, Constructor)
{
    PtsReader reader1;

    StageFactory f;
    Stage* reader2(f.createStage("readers.pts"));
    EXPECT_TRUE(reader2);
}



TEST(PtsReader, ReadPtsExtraDims)
{
    PtsReader reader;
    Options options;
    options.add("filename", Support::datapath("pts/test.pts"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 19u);

    PointLayout *layout = view->layout();
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::X, 0), 3.9809721f);
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Y, 0), -2.006119f);
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Z, 0), -0.010086f);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Red, 0), 97);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Green, 0), 59);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::Blue, 0), 38);
}



TEST(PtsReader, ReadPtsThreeDims)
{
    PtsReader reader;
    Options options;
    options.add("filename", Support::datapath("pts/bunny_8.pts"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 8u);

    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, 0), -0.037829);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, 0), 0.12794);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, 0), 0.004474);
}

TEST(PtsReader, ReadPtsFourDims)
{
    PtsReader reader;
    Options options;
    options.add("filename", Support::datapath("pts/site_56_8.pts"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 8u);

    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::X, 0), 6691.797611);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, 0), 17.347517);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Z, 0), 1203.033447);
    EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Intensity, 0), -255 + 2048);
}

}
