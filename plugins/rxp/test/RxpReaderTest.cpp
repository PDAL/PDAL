/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include "gtest/gtest.h"

#include <pdal/Options.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointBuffer.hpp>

#include "RxpReader.hpp"
#include "Config.hpp"

using namespace pdal;

Options defaultRxpReaderOptions()
{
    Options options;
    Option filename("filename",
        testDataPath() + "130501_232206_cut.rxp", "");
    options.add(filename);
    return options;
}

template <typename T>
void checkDimensionClose(const PointBuffer& data,
                         std::size_t index,
                         Dimension::Id::Enum dim,
                         T expected)
{
    T actual = data.getFieldAs<T>(dim, index);
    EXPECT_FLOAT_EQ(expected, actual);
}

template <typename T>
void checkDimensionEqual(const PointBuffer& data,
                         std::size_t index,
                         Dimension::Id::Enum dim,
                         T expected)
{
    T actual = data.getFieldAs<T>(dim, index);
    EXPECT_EQ(expected, actual);
}

void checkPoint(const PointBuffer& data, std::size_t index,
                float x, float y, float z,
                double time, double echoRange, float amplitude,
                float reflectance, float deviation,
                bool isPpsLocked, uint8_t returnNumber,
                uint8_t numberOfReturns
                )
{
    using namespace Dimension;
    checkDimensionClose(data, index, Id::X, x);
    checkDimensionClose(data, index, Id::Y, y);
    checkDimensionClose(data, index, Id::Z, z);
    checkDimensionClose(data,
                        index,
                        getTimeDimensionId(isPpsLocked),
                        time);
    checkDimensionClose(data, index, Id::EchoRange, echoRange);
    checkDimensionClose(data, index, Id::Amplitude, amplitude);
    checkDimensionClose(data, index, Id::Reflectance, reflectance);
    checkDimensionClose(data, index, Id::Deviation, deviation);
    checkDimensionEqual(data, index, Id::IsPpsLocked, isPpsLocked);
    checkDimensionEqual(data, index, Id::ReturnNumber, returnNumber);
    checkDimensionEqual(data, index, Id::NumberOfReturns, numberOfReturns);
}

TEST(RxpReaderTest, testConstructor)
{
    Options options = defaultRxpReaderOptions();
    RxpReader reader;
    reader.setOptions(options);
    EXPECT_TRUE(reader.getDescription() == "RXP Reader");
    EXPECT_EQ(reader.getName(), "readers.rxp");
}

TEST(RxpReaderTest, testRead)
{
    Options options = defaultRxpReaderOptions();
    RxpReader reader;
    reader.setOptions(options);

    PointContext ctx;
    reader.prepare(ctx);

    PointBufferSet pbSet = reader.execute(ctx);
    EXPECT_EQ(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 177208);

    checkPoint(*buf, 0, 2.2630672454833984, -0.038407701998949051, -1.3249952793121338, 342656.34233957872,
            2.6865001276019029, 19.8699989, 5.70246553, 4, true, 1, 1);
    checkPoint(*buf, 1, 2.2641847133636475, -0.038409631699323654, -1.3245694637298584, 342656.34235912003,
            2.687250127637526, 19.5299988, 5.36292124, 2, true, 1, 1);
    checkPoint(*buf, 2, 2.26853346824646, -0.038410264998674393, -1.3260456323623657, 342656.34237866144,
            2.6917501278512646, 19.3699989, 5.2056551, 5, true, 1, 1);
}

TEST(RxpReaderTest, testNoPpsSync)
{
    Options options = defaultRxpReaderOptions();
    Option syncToPps("sync_to_pps", "false", "");
    options.add(syncToPps);
    RxpReader reader;
    reader.setOptions(options);

    PointContext ctx;
    reader.prepare(ctx);

    PointBufferSet pbSet = reader.execute(ctx);
    PointBufferPtr buf = *pbSet.begin();

    checkPoint(*buf, 0, 0.0705248788, -0.0417557284, 0.0304775704, 31.917255942733149,
            0.14050000667339191, 0.689999998, -14.4898596, 3, false, 1, 1);
}

TEST(RxpReaderTest, testInclFix)
{
    Options options = defaultRxpReaderOptions();
    options.add("inclination_fix", "true", "");
    options.add("inclination_fix_window", "2", "");
    RxpReader reader;
    reader.setOptions(options);

    PointContext ctx;
    reader.prepare(ctx);

    PointBufferSet pbSet = reader.execute(ctx);
    EXPECT_EQ(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 136482);

    // TODO can we test any of the motion? Hard to say, that.
}

TEST(RxpReaderTest, testRotatePoint)
{
    Point p{1, 2, 3};

    Point r1 = rotatePoint(p, makeRotationMatrix(0, 0));
    EXPECT_FLOAT_EQ(r1.x, 1);
    EXPECT_FLOAT_EQ(r1.y, 2);
    EXPECT_FLOAT_EQ(r1.z, 3);

    Point r2 = rotatePoint(p, makeRotationMatrix(M_PI / 2, 0));
    EXPECT_FLOAT_EQ(r2.x, 1);
    EXPECT_FLOAT_EQ(r2.y, 3);
    EXPECT_FLOAT_EQ(r2.z, -2);
}

TEST(RxpReaderTest, testMovingAverage)
{
    InclinationVector incl;
    incl.push_back(Inclination{0, 1, 2});
    incl.push_back(Inclination{2, 3, 4});

    Inclination i1 = movingAverage(incl, 0, 1);    
    EXPECT_FLOAT_EQ(i1.time, 1);
    EXPECT_EQ(i1.roll, 2);
    EXPECT_EQ(i1.pitch, 3);
}

TEST(RxpReaderTest, testURILogic)
{
    Option fileOption("filename", "foobar", "");
    Options fileOptions(fileOption);
    EXPECT_EQ(extractRivlibURI(fileOptions), "file:foobar");

    Option rdtpOption("rdtp", "192.168.0.33", "");
    Options rdtpOptions(rdtpOption);
    EXPECT_EQ(extractRivlibURI(rdtpOptions), "rdtp://192.168.0.33");

    Options emptyOptions;
    EXPECT_THROW(extractRivlibURI(emptyOptions), option_not_found);

    Options bothOptions;
    bothOptions.add(fileOption);
    bothOptions.add(rdtpOption);
    EXPECT_THROW(extractRivlibURI(bothOptions), option_not_found);
}
