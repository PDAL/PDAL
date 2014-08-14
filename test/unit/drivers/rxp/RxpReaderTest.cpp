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

#include <boost/test/unit_test.hpp>

#include <pdal/Options.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/rxp/RxpReader.hpp>
#include "../../Support.hpp"


pdal::Options defaultRxpReaderOptions()
{
    pdal::Options options;
    pdal::Option filename("filename",
            Support::datapath("rxp/130501_232206_cut.rxp"), "");
    options.add(filename);
    return options;
}


template <typename T>
void checkDimensionClose(const pdal::PointBuffer& data,
                         std::size_t index,
                         pdal::Dimension::Id::Enum dim,
                         T expected)
{
    T actual = data.getFieldAs<T>(dim, index);
    BOOST_CHECK_CLOSE(expected, actual, 0.000001);
}


template <typename T>
void checkDimensionEqual(const pdal::PointBuffer& data,
                         std::size_t index,
                         pdal::Dimension::Id::Enum dim,
                         T expected)
{
    T actual = data.getFieldAs<T>(dim, index);
    BOOST_CHECK_EQUAL(expected, actual);
}


void checkPoint(const pdal::PointBuffer& data, std::size_t index,
                float x, float y, float z,
                double time, double echoRange, float amplitude,
                float reflectance, float deviation,
                bool isPpsLocked, boost::uint8_t returnNumber,
                boost::uint8_t numberOfReturns
                )
{
    using namespace pdal::Dimension;
    checkDimensionClose(data, index, Id::X, x);
    checkDimensionClose(data, index, Id::Y, y);
    checkDimensionClose(data, index, Id::Z, z);
    checkDimensionClose(data,
                        index,
                        pdal::drivers::rxp::getTimeDimensionId(isPpsLocked),
                        time);
    checkDimensionClose(data, index, Id::EchoRange, echoRange);
    checkDimensionClose(data, index, Id::Amplitude, amplitude);
    checkDimensionClose(data, index, Id::Reflectance, reflectance);
    checkDimensionClose(data, index, Id::Deviation, deviation);
    checkDimensionEqual(data, index, Id::IsPpsLocked, isPpsLocked);
    checkDimensionEqual(data, index, Id::ReturnNumber, returnNumber);
    checkDimensionEqual(data, index, Id::NumberOfReturns, numberOfReturns);
}


BOOST_AUTO_TEST_SUITE(RxpReaderTest)


BOOST_AUTO_TEST_CASE(testConstructor)
{
    pdal::Options options = defaultRxpReaderOptions();
    pdal::drivers::rxp::RxpReader reader(options);
    BOOST_CHECK(reader.getDescription() == "RXP Reader");
    BOOST_CHECK_EQUAL(reader.getName(), "drivers.rxp.reader");
}


BOOST_AUTO_TEST_CASE(testRead)
{
    pdal::Options options = defaultRxpReaderOptions();
    pdal::drivers::rxp::RxpReader reader(options);

    pdal::PointContext ctx;
    reader.prepare(ctx);

    pdal::PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    pdal::PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 177208);

    checkPoint(*buf, 0, 2.2630672454833984, -0.038407701998949051, -1.3249952793121338, 342656.34233957872,
            2.6865001276019029, 19.8699989, 5.70246553, 4, true, 1, 1);
    checkPoint(*buf, 1, 2.2641847133636475, -0.038409631699323654, -1.3245694637298584, 342656.34235912003,
            2.687250127637526, 19.5299988, 5.36292124, 2, true, 1, 1);
    checkPoint(*buf, 2, 2.26853346824646, -0.038410264998674393, -1.3260456323623657, 342656.34237866144,
            2.6917501278512646, 19.3699989, 5.2056551, 5, true, 1, 1);
}


BOOST_AUTO_TEST_CASE(testNoPpsSync)
{
    pdal::Options options = defaultRxpReaderOptions();
    pdal::Option syncToPps("sync_to_pps", "false", "");
    options.add(syncToPps);
    pdal::drivers::rxp::RxpReader reader(options);

    pdal::PointContext ctx;
    reader.prepare(ctx);

    pdal::PointBufferSet pbSet = reader.execute(ctx);
    pdal::PointBufferPtr buf = *pbSet.begin();

    checkPoint(*buf, 0, 0.0705248788, -0.0417557284, 0.0304775704, 31.917255942733149,
            0.14050000667339191, 0.689999998, -14.4898596, 3, false, 1, 1);
}


BOOST_AUTO_TEST_CASE(testInclFix)
{
    pdal::Options options = defaultRxpReaderOptions();
    options.add("inclination_fix", "true", "");
    options.add("inclination_fix_window", "2", "");
    pdal::drivers::rxp::RxpReader reader(options);

    pdal::PointContext ctx;
    reader.prepare(ctx);

    pdal::PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    pdal::PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 136482);

    // TODO can we test any of the motion? Hard to say, that.
}



BOOST_AUTO_TEST_CASE(testRotatePoint)
{
    using namespace pdal::drivers::rxp;

    Point p{1, 2, 3};

    Point r1 = rotatePoint(p, makeRotationMatrix(0, 0));
    BOOST_CHECK_CLOSE(r1.x, 1, 0.001);
    BOOST_CHECK_CLOSE(r1.y, 2, 0.001);
    BOOST_CHECK_CLOSE(r1.z, 3, 0.001);

    Point r2 = rotatePoint(p, makeRotationMatrix(M_PI / 2, 0));
    BOOST_CHECK_CLOSE(r2.x, 1, 0.001);
    BOOST_CHECK_CLOSE(r2.y, 3, 0.001);
    BOOST_CHECK_CLOSE(r2.z, -2, 0.001);
}



BOOST_AUTO_TEST_CASE(testURILogic)
{
    using namespace pdal::drivers::rxp;

    pdal::Option fileOption("filename", "foobar", "");
    pdal::Options fileOptions(fileOption);
    BOOST_CHECK_EQUAL(extractRivlibURI(fileOptions), "file:foobar");

    pdal::Option rdtpOption("rdtp", "192.168.0.33", "");
    pdal::Options rdtpOptions(rdtpOption);
    BOOST_CHECK_EQUAL(extractRivlibURI(rdtpOptions), "rdtp://192.168.0.33");

    pdal::Options emptyOptions;
    BOOST_CHECK_THROW(extractRivlibURI(emptyOptions), pdal::option_not_found);

    pdal::Options bothOptions;
    bothOptions.add(fileOption);
    bothOptions.add(rdtpOption);
    BOOST_CHECK_THROW(extractRivlibURI(bothOptions), pdal::option_not_found);
}


BOOST_AUTO_TEST_SUITE_END()
