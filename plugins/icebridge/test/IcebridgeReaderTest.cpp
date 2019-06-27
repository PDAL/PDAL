/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

template <typename T>
void checkDimension(const PointView& data, std::size_t index,
    Dimension::Id dim, T expected)
{
    T actual = data.getFieldAs<T>(dim, index);
    EXPECT_FLOAT_EQ((float)expected, (float)actual);
}

void checkPoint(
        const PointView& data,
        std::size_t index,
        float time,
        float latitude,
        float longitude,
        float elevation,
        int xmtSig,
        int rcvSig,
        float azimuth,
        float pitch,
        float roll,
        float gpsPdop,
        float pulseWidth,
        float relTime)
{
    using namespace Dimension;
    checkDimension(data, index, Id::OffsetTime, time);
    checkDimension(data, index, Id::Y, latitude);
    checkDimension(data, index, Id::X, longitude);
    checkDimension(data, index, Id::Z, elevation);
    checkDimension(data, index, Id::StartPulse, xmtSig);
    checkDimension(data, index, Id::ReflectedPulse, rcvSig);
    checkDimension(data, index, Id::Azimuth, azimuth);
    checkDimension(data, index, Id::Pitch, pitch);
    checkDimension(data, index, Id::Roll, roll);
    checkDimension(data, index, Id::Pdop, gpsPdop);
    checkDimension(data, index, Id::PulseWidth, pulseWidth);
    checkDimension(data, index, Id::GpsTime, relTime);
}

std::string getFilePath()
{
    return Support::datapath("icebridge/twoPoints.h5");
}

TEST(IcebridgeReaderTest, testRead)
{
    StageFactory f;
    Stage* reader(f.createStage("readers.icebridge"));
    EXPECT_TRUE(reader);

    Option filename("filename", getFilePath());
    Options options(filename);
    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 2u);

    checkPoint(
            *view,
            0,
            1414375e2f,      // time
            82.60531f,       // latitude
            -58.59381f,      // longitude
            18.678f,         // elevation
            2408,            // xmtSig
            181,             // rcvSig
            49.91f,          // azimuth
            -4.376f,         // pitch
            0.608f,          // roll
            2.9f,            // gpsPdop
            20.0f,           // pulseWidth
            0.0f);           // relTime

    checkPoint(
            *view,
            1,
            1414375e2f,      // time
            82.60528f,       // latitude
            -58.59512f,      // longitude
            18.688f,         // elevation
            2642,            // xmtSig
            173,             // rcvSig
            52.006f,         // azimuth
            -4.376f,         // pitch
            0.609f,          // roll
            2.9f,            // gpsPdop
            17.0f,           // pulseWidth
            0.0f);           // relTime
}

TEST(IcebridgeReaderTest, testPipeline)
{
    PipelineManager manager;

    manager.readPipeline(Support::configuredpath("icebridge/pipeline.json"));

    point_count_t numPoints = manager.execute();
    EXPECT_EQ(numPoints, 2u);
    FileUtils::deleteFile(Support::datapath("icebridge/outfile.txt"));
}
