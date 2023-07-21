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
    EXPECT_NEAR(635618.98, view->getFieldAs<double>(Dimension::Id::X, 0),1e-4);
    EXPECT_NEAR(848898.71, view->getFieldAs<double>(Dimension::Id::Y, 0),1e-4);
    EXPECT_NEAR(405.59, view->getFieldAs<double>(Dimension::Id::Z, 0),1e-4);
    EXPECT_DOUBLE_EQ(0, view->getFieldAs<double>(Dimension::Id::OffsetTime, 0));
    EXPECT_EQ(55040, view->getFieldAs<uint16_t>(Dimension::Id::Intensity, 0));
    EXPECT_EQ(0, view->getFieldAs<uint16_t>(Dimension::Id::PointSourceId, 0));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 0));
    EXPECT_EQ(0, view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 0));
    EXPECT_EQ(20, view->getFieldAs<uint8_t>(Dimension::Id::Classification, 0));
}
}

