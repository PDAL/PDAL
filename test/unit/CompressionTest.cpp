/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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

#include <iterator>
#include <sstream>
#include <iostream>
#include <string>
#include <random>

#include "Support.hpp"
#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <LasReader.hpp>
#include <pdal/Compression.hpp>

using namespace pdal;

std::vector<char> getBytes(PointViewPtr view)
{
    std::vector<char> bytes(view->pointSize() * view->size());
    DimTypeList dimTypes = view->dimTypes();

    char *p = bytes.data();
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        view->getPackedPoint(dimTypes, idx, p);
        p += view->pointSize();
    }
    return bytes;
}


TEST(Compression, Simple)
{
    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const pdal::Option opt_filename("filename", file);
    pdal::Options opts;
    opts.add(opt_filename);

    LasReader reader;
    reader.setOptions(opts);

    PointTable table;
    PointLayoutPtr layout(table.layout());

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(layout->pointSize(), 52U);

    std::vector<unsigned char> rawBuf;
    LazPerfBuf b(rawBuf);

    DimTypeList dimTypes = layout->dimTypes();
    LazPerfCompressor<LazPerfBuf> compressor(b, dimTypes);

    std::vector<char> tmpbuf(compressor.pointSize());
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        view->getPackedPoint(dimTypes, idx, tmpbuf.data());
        compressor.compress(tmpbuf.data(), compressor.pointSize());
    }
    compressor.done();

    EXPECT_EQ(view->size() * compressor.pointSize(), (size_t)55380);
    EXPECT_EQ(rawBuf.size(), (size_t)30945);

    LazPerfBuf b2(rawBuf);

    LazPerfDecompressor<LazPerfBuf> decompressor(b2, dimTypes);

    size_t outbufSize = decompressor.pointSize() * view->size();
    std::vector<char> outbuf(outbufSize);
    decompressor.decompress(outbuf.data(), outbufSize);

    PointViewPtr otherView(new PointView(table));

    char *pos = outbuf.data();
    for (PointId nextId = 0; nextId < 11; nextId++)
    {
        otherView->setPackedPoint(dimTypes, nextId, pos);
        pos += decompressor.pointSize();
    }
    EXPECT_EQ(otherView->size(), 11U);
    EXPECT_EQ(getBytes(otherView).size(), (size_t)(52 * 11));

    uint16_t r = otherView->getFieldAs<uint16_t>(Dimension::Id::Red, 10);
    EXPECT_EQ(r, 64U);
    int32_t x = otherView->getFieldAs<int32_t>(Dimension::Id::X, 10);
    EXPECT_EQ(x, 636038);
    double xd = otherView->getFieldAs<double>(Dimension::Id::X, 10);
    EXPECT_FLOAT_EQ(xd, 636037.53);
    int32_t y = otherView->getFieldAs<int32_t>(Dimension::Id::Y, 10);
    EXPECT_EQ(y, 849338);
}


TEST(Compression, types)
{
    using namespace Dimension;
    Type::Enum types[] = {
        Type::Unsigned8, Type::Unsigned16, Type::Unsigned32, Type::Unsigned64,
        Type::Signed8, Type::Signed16, Type::Signed32, Type::Signed64,
        Type::Float, Type::Double
    };
    // Size is 42.

    std::default_random_engine generator;
    std::uniform_int_distribution<int> dist(std::numeric_limits<int>::min());
    char pts[3][42];

    // Fill three "points" with some random data.
    char *c = &pts[0][0];
    for (size_t i = 0; i < 3 * 42; ++i)
    {
        int v = dist(generator);
        memcpy(c++, &v, sizeof(char));
    }

    DimTypeList dimTypes;
    for (auto ti = std::begin(types); ti != std::end(types); ++ti)
        dimTypes.push_back(DimType(Dimension::Id::Unknown, *ti));

    std::vector<unsigned char> rawBuf;
    LazPerfBuf b(rawBuf);
    LazPerfCompressor<LazPerfBuf> compressor(b, dimTypes);
    for (size_t i = 0; i < 50; i++)
    {
        compressor.compress(pts[0], 42);
        compressor.compress(pts[1], 42);
        compressor.compress(pts[2], 42);
    }
    compressor.done();

    LazPerfBuf b2(rawBuf);

    LazPerfDecompressor<LazPerfBuf> decompressor(b2, dimTypes);
    char oPts[3][42];
    for (size_t i = 0; i < 50; ++i)
    {
        decompressor.decompress(oPts[0], 42);
        decompressor.decompress(oPts[1], 42);
        decompressor.decompress(oPts[2], 42);
        EXPECT_EQ(memcmp(pts[0], oPts[0], 42), 0);
        EXPECT_EQ(memcmp(pts[1], oPts[1], 42), 0);
        EXPECT_EQ(memcmp(pts[2], oPts[2], 42), 0);
        memset(oPts[0], 0, 42);
        memset(oPts[1], 0, 42);
        memset(oPts[2], 0, 42);
    }
}

//
// BOOST_AUTO_TEST_CASE(test_compress_copied_view)
// {
//     using namespace pdal;
//
//
//     const std::string file(Support::datapath("las/1.2-with-color.las"));
//     const pdal::Option opt_filename("filename", file);
//     pdal::Options opts;
//     opts.add(opt_filename);
//     pdal::drivers::las::Reader reader;
//     reader.setOptions(opts);
//     PointTable ftable;
//     reader.prepare(ftable);
//     PointViewSet viewSet = reader.execute(ftable);
//     PointViewPtr view = *viewSet.begin();
//
//     PointTable table;
//     table->registerDim(Dimension::Id::X);
//     table->registerDim(Dimension::Id::Y);
//     table->registerDim(Dimension::Id::Z);
//     PointView new_view(table);
// //
//     for (PointId i = 0; i < view->size(); ++i)
//     {
//         new_view->setField(Dimension::Id::X, i, view->getFieldAs<double>(Dimension::Id::X, i));
//         new_view->setField(Dimension::Id::Y, i, view->getFieldAs<double>(Dimension::Id::Y, i));
//         new_view->setField(Dimension::Id::Z, i, view->getFieldAs<double>(Dimension::Id::Z, i));
//     }
//     SQLiteTestStream s;
//     compression::Compress<SQLiteTestStream>(table, new_view, s, compression::CompressionType::Lazperf, 0, 0);
//     SQLiteTestStream s2;
//     s2.buf = s.buf;
//     PointViewPtr b = compression::Decompress<SQLiteTestStream>(table, s2, 11, compression::CompressionType::Lazperf);
//
//     int32_t y = b->getFieldAs<int32_t>(Dimension::Id::Y, 10);
//     BOOST_CHECK_EQUAL(y, 849338);
//     int32_t x = b->getFieldAs<int32_t>(Dimension::Id::X, 10);
//     BOOST_CHECK_EQUAL(x, 636038);
//     double xd = b->getFieldAs<double>(Dimension::Id::X, 10);
//     BOOST_CHECK_CLOSE(xd, 636037.53, 0.001);
//
// }
//
//
//
// BOOST_AUTO_TEST_CASE(test_compress_simple_view)
// {
//     using namespace pdal;
//
//     PointTable table;
//     table->registerDim(Dimension::Id::X);
//     table->registerDim(Dimension::Id::Y);
//     table->registerDim(Dimension::Id::Z);
//     table->registerDim(Dimension::Id::GpsTime);
//     table->registerDim(Dimension::Id::Intensity);
//     table->registerDim(Dimension::Id::PointSourceId);
//     table->registerDim(Dimension::Id::ScanAngleRank);
//     table->registerDim(Dimension::Id::Red);
//     table->registerDim(Dimension::Id::Green);
//     table->registerDim(Dimension::Id::Blue);
//     table->registerDim(Dimension::Id::ReturnNumber);
//     table->registerDim(Dimension::Id::NumberOfReturns);
//     table->registerDim(Dimension::Id::ScanDirectionFlag);
//     table->registerDim(Dimension::Id::EdgeOfFlightLine);
//     table->registerDim(Dimension::Id::Classification);
//     table->registerDim(Dimension::Id::UserData);
//     PointView view(table);
//     for (PointId i = 0; i < 100; ++i)
//     {
//         view->setField(Dimension::Id::X, i, i);
//         view->setField(Dimension::Id::Y, i, i+100);
//         view->setField(Dimension::Id::Z, i, i+1000);
//         view->setField(Dimension::Id::GpsTime, i, i+10000);
//         view->setField(Dimension::Id::Intensity, i, 600);
//         view->setField(Dimension::Id::PointSourceId, i, 60);
//         view->setField(Dimension::Id::ScanAngleRank, i, 1003.23);
//         view->setField(Dimension::Id::Red, i, 26);
//         view->setField(Dimension::Id::Green, i, 42);
//         view->setField(Dimension::Id::Blue, i, 255);
//         view->setField(Dimension::Id::ReturnNumber, i, 2);
//         view->setField(Dimension::Id::NumberOfReturns, i, 2);
//         view->setField(Dimension::Id::ScanDirectionFlag, i, 1);
//         view->setField(Dimension::Id::EdgeOfFlightLine, i, 1);
//         view->setField(Dimension::Id::Classification, i, 2);
//         view->setField(Dimension::Id::UserData, i, 25);
//     }
//
//     SQLiteTestStream s;
//     compression::Compress<SQLiteTestStream>(table, view, s, compression::CompressionType::Lazperf, 0, 0);
//     SQLiteTestStream s2;
//     s2.buf = s.buf;
//     PointViewPtr b = compression::Decompress<SQLiteTestStream>(table, s2, 11, compression::CompressionType::Lazperf);
// //     std::cout << *b << std::endl;
//     uint16_t r = b->getFieldAs<uint16_t>(Dimension::Id::Red, 10);
//     BOOST_CHECK_EQUAL(r, 26u);
// }

