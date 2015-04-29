/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include "OptechReader.hpp"

#include <pdal/StageFactory.hpp>
#include "Support.hpp"


namespace pdal
{


namespace
{


std::string getTestfilePath()
{
    return Support::datapath("optech/sample.csd");
}


class OptechReaderTest : public ::testing::Test
{
public:
    OptechReaderTest()
        : ::testing::Test()
        , m_reader()
    {
        Options options;
        options.add("filename", getTestfilePath());
        m_reader.setOptions(options);
    }

    OptechReader m_reader;
};
}


TEST(OptechReader, Constructor)
{
    OptechReader reader1;

    StageFactory f;
    std::unique_ptr<Stage> reader2(f.createStage("readers.optech"));
    EXPECT_TRUE(reader2.get());
}


TEST_F(OptechReaderTest, Header)
{
    PointTable table;
    m_reader.prepare(table);
    CsdHeader header = m_reader.getHeader();

    EXPECT_STREQ("CSD", header.signature);
    EXPECT_STREQ("Optech Incorporated", header.vendorId);
    EXPECT_STREQ("DASHMap", header.softwareVersion);
    EXPECT_FLOAT_EQ(5.2010002f, header.formatVersion);
    EXPECT_EQ(2048, header.headerSize);
    EXPECT_EQ(1660u, header.gpsWeek);
    EXPECT_DOUBLE_EQ(575644.74484563898, header.minTime);
    EXPECT_DOUBLE_EQ(575644.75883187703, header.maxTime);
    EXPECT_EQ(1000u, header.numRecords);
    EXPECT_EQ(1u, header.numStrips);
    EXPECT_EQ(0u, header.stripPointers[0]);
    EXPECT_DOUBLE_EQ(0.028000000000000001, header.misalignmentAngles[0]);
    EXPECT_DOUBLE_EQ(0.014, header.misalignmentAngles[1]);
    EXPECT_DOUBLE_EQ(0.002, header.misalignmentAngles[2]);
    EXPECT_DOUBLE_EQ(0.002250602070446688, header.imuOffsets[0]);
    EXPECT_DOUBLE_EQ(-0.0021128955924643355, header.imuOffsets[1]);
    EXPECT_DOUBLE_EQ(0.0054852207731677788, header.imuOffsets[2]);
    EXPECT_DOUBLE_EQ(13, header.temperature);
    EXPECT_DOUBLE_EQ(1026.75, header.pressure);
}


TEST_F(OptechReaderTest, ReadingPoints)
{
    PointTable table;
    m_reader.prepare(table);
    PointViewSet viewSet = m_reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1000u);

    EXPECT_DOUBLE_EQ(-82.554028877408555,
                     view->getFieldAs<double>(Dimension::Id::X, 0));
    EXPECT_DOUBLE_EQ(36.534611447321907,
                     view->getFieldAs<double>(Dimension::Id::Y, 0));
    EXPECT_DOUBLE_EQ(344.80889224602356,
                     view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(5.756447448456390e5,
                     view->getFieldAs<double>(Dimension::Id::GpsTime, 0));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 0));
    EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 0));
    EXPECT_FLOAT_EQ(8.27356689453125e2,
        view->getFieldAs<float>(Dimension::Id::EchoRange, 0));
    EXPECT_EQ(384, view->getFieldAs<uint16_t>(Dimension::Id::Intensity, 0));
    EXPECT_FLOAT_EQ(-14.55516,
        view->getFieldAs<float>(Dimension::Id::ScanAngleRank, 0));
}


TEST_F(OptechReaderTest, Spatialreference)
{
    SpatialReference expected;
    expected.setFromUserInput("EPSG:4326");
    SpatialReference actual = m_reader.getSpatialReference();
    EXPECT_EQ(expected, actual);
}
}
