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
* OF USE, view, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/pdal_test_main.hpp>

#include <pdal/Options.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>

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
void checkDimensionClose(const PointViewPtr view,
                         std::size_t index,
                         Dimension::Id::Enum dim,
                         T expected)
{
    T actual = view->getFieldAs<T>(dim, index);
    EXPECT_FLOAT_EQ(expected, actual);
}

template <typename T>
void checkDimensionEqual(const PointViewPtr view,
                         std::size_t index,
                         Dimension::Id::Enum dim,
                         T expected)
{
    T actual = view->getFieldAs<T>(dim, index);
    EXPECT_EQ(expected, actual);
}

void checkPoint(const PointViewPtr view, std::size_t index,
                float x, float y, float z,
                double time, double echoRange, float amplitude,
                float reflectance, float deviation,
                bool isPpsLocked, uint8_t returnNumber,
                uint8_t numberOfReturns
                )
{
    using namespace Dimension;
    checkDimensionClose(view, index, Id::X, x);
    checkDimensionClose(view, index, Id::Y, y);
    checkDimensionClose(view, index, Id::Z, z);
    checkDimensionClose(view,
                        index,
                        getTimeDimensionId(isPpsLocked),
                        time);
    checkDimensionClose(view, index, Id::EchoRange, echoRange);
    checkDimensionClose(view, index, Id::Amplitude, amplitude);
    checkDimensionClose(view, index, Id::Reflectance, reflectance);
    checkDimensionClose(view, index, Id::Deviation, deviation);
    checkDimensionEqual(view, index, Id::IsPpsLocked, isPpsLocked);
    checkDimensionEqual(view, index, Id::ReturnNumber, returnNumber);
    checkDimensionEqual(view, index, Id::NumberOfReturns, numberOfReturns);
}

TEST(RxpReaderTest, testConstructor)
{
    Options options = defaultRxpReaderOptions();
    RxpReader reader;
    reader.setOptions(options);
    EXPECT_EQ(reader.getName(), "readers.rxp");
}

TEST(RxpReaderTest, testRead)
{
    Options options = defaultRxpReaderOptions();
    RxpReader reader;
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 177208);

    checkPoint(view, 0, 2.2630672454833984, -0.038407701998949051, -1.3249952793121338, 342656.34233957872,
            2.6865001276019029, 19.8699989, 5.70246553, 4, true, 1, 1);
    checkPoint(view, 1, 2.2641847133636475, -0.038409631699323654, -1.3245694637298584, 342656.34235912003,
            2.687250127637526, 19.5299988, 5.36292124, 2, true, 1, 1);
    checkPoint(view, 2, 2.26853346824646, -0.038410264998674393, -1.3260456323623657, 342656.34237866144,
            2.6917501278512646, 19.3699989, 5.2056551, 5, true, 1, 1);
}

TEST(RxpReaderTest, testNoPpsSync)
{
    Options options = defaultRxpReaderOptions();
    Option syncToPps("sync_to_pps", "false", "");
    options.add(syncToPps);
    RxpReader reader;
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    checkPoint(view, 0, 0.0705248788, -0.0417557284, 0.0304775704, 31.917255942733149,
            0.14050000667339191, 0.689999998, -14.4898596, 3, false, 1, 1);
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
    EXPECT_THROW(extractRivlibURI(emptyOptions), Option::not_found);

    Options bothOptions;
    bothOptions.add(fileOption);
    bothOptions.add(rdtpOption);
    EXPECT_THROW(extractRivlibURI(bothOptions), pdal_error);
}
