/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)*
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
*
****************************************************************************/

#include <pdal/pdal_test_main.hpp>

#include <pdal/StageFactory.hpp>
#include "../io/ArrowReader.hpp"
#include "Support.hpp"

namespace pdal
{


namespace
{

std::string getTestfilePath()
{
    return Support::datapath("arrow/1.2-with-color.feather");
}

class ArrowReaderTest : public ::testing::Test
{
public:
    ArrowReaderTest()
        : ::testing::Test()
        , m_reader()
    {
        Options options;
        options.add("filename", getTestfilePath());
        m_reader.setOptions(options);
    }

    ArrowReader m_reader;
};
}

TEST_F(ArrowReaderTest, Constructor)
{
    ArrowReader reader1;

    StageFactory f;
    Stage* reader2(f.createStage("readers.arrow"));
}

TEST_F(ArrowReaderTest, ReadingPoints)
{
    PointTable table;
    m_reader.prepare(table);
    PointViewSet viewSet = m_reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);

    //number of points
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1065);

    //some tests on the first point
    int point_id(0);
    EXPECT_NEAR(637012.240, view->getFieldAs<double>(Dimension::Id::X, point_id),1e-4);
    EXPECT_NEAR(849028.310, view->getFieldAs<double>(Dimension::Id::Y, point_id),1e-4);
    EXPECT_NEAR(431.660, view->getFieldAs<double>(Dimension::Id::Z, point_id),1e-4);
    EXPECT_EQ(143, view->getFieldAs<uint16_t>(Dimension::Id::Intensity, point_id));
    EXPECT_EQ(7326, view->getFieldAs<uint16_t>(Dimension::Id::PointSourceId, point_id));
    EXPECT_EQ(68, view->getFieldAs<uint16_t>(Dimension::Id::Red, point_id));
    EXPECT_EQ(77, view->getFieldAs<uint16_t>(Dimension::Id::Green, point_id));
    EXPECT_EQ(88, view->getFieldAs<uint16_t>(Dimension::Id::Blue, point_id));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, point_id));
    EXPECT_EQ(1, (int)view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, point_id));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::Classification, 0));
    EXPECT_EQ(-9, view->getFieldAs<int16_t>(Dimension::Id::ScanAngleRank, 0));


    point_id = 8;
    EXPECT_NEAR(636198.79, view->getFieldAs<double>(Dimension::Id::X, point_id),1e-4);
    EXPECT_NEAR(849238.09, view->getFieldAs<double>(Dimension::Id::Y, point_id),1e-4);
    EXPECT_NEAR(428.05, view->getFieldAs<double>(Dimension::Id::Z, point_id),1e-4);
    EXPECT_EQ(142, view->getFieldAs<uint16_t>(Dimension::Id::Intensity, point_id));
    EXPECT_EQ(7326, view->getFieldAs<uint16_t>(Dimension::Id::PointSourceId, point_id));
    EXPECT_EQ(106, view->getFieldAs<uint16_t>(Dimension::Id::Red, point_id));
    EXPECT_EQ(95, view->getFieldAs<uint16_t>(Dimension::Id::Green, point_id));
    EXPECT_EQ(124, view->getFieldAs<uint16_t>(Dimension::Id::Blue, point_id));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, point_id));
    EXPECT_EQ(1, (int)view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, point_id));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::Classification, 0));
    EXPECT_EQ(-9, view->getFieldAs<int16_t>(Dimension::Id::ScanAngleRank, 0));
}
}

