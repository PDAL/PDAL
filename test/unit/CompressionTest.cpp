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

#include "gtest/gtest.h"

#include <sstream>
#include <iostream>
#include <string>

#include "Support.hpp"
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <LasReader.hpp>
#include <pdal/Compression.hpp>

struct SQLiteTestStream {
    SQLiteTestStream() : buf(), idx(0) {}

    void putBytes(const unsigned char* b, size_t len) {
        while(len --) {
            buf.push_back(*b++);
        }
    }

    void putByte(const unsigned char b) {
        buf.push_back(b);
    }

    unsigned char getByte() {
        return buf[idx++];
    }

    void getBytes(unsigned char *b, int len) {
        for (int i = 0 ; i < len ; i ++) {
            b[i] = getByte();
        }
    }

    std::vector<unsigned char> buf;
    size_t idx;
};

std::vector<char> getBytes(PointBuffer buffer, PointContextRef ctx)
{
    std::vector<char> bytes;
    bytes.resize(ctx.pointSize() * buffer.size());

    Charbuf buf(bytes);
    std::ostream strm(&buf);
    buffer.getBytes(strm, 0, buffer.size());

    return bytes;
}

TEST(CompressionTest, test_compress_file)
{
    using namespace pdal;

    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const Option opt_filename("filename", file);
    Options opts;
    opts.add(opt_filename);

    LasReader reader;
    reader.setOptions(opts);

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet buffers = reader.execute(ctx);
    PointBufferPtr buffer = *buffers.begin();

    EXPECT_EQ(ctx.pointSize(), 52);
    SQLiteTestStream s;
    compression::Compress<SQLiteTestStream>(ctx, *buffer, s, compression::CompressionType::Lazperf, 0, 0);

    EXPECT_EQ(getBytes(*buffer, ctx).size(), 55380);
    EXPECT_EQ(s.buf.size(), 30945);

    SQLiteTestStream s2;
    s2.buf = s.buf;
    PointBufferPtr b = compression::Decompress<SQLiteTestStream>(ctx, s2, 11, compression::CompressionType::Lazperf);

    EXPECT_EQ(b->size(), 11);
    EXPECT_EQ(getBytes(*b, ctx).size(), 52 * 11);

    uint16_t r = b->getFieldAs<uint16_t>(Dimension::Id::Red, 10);
    EXPECT_EQ(r, 64u);
    int32_t x = b->getFieldAs<int32_t>(Dimension::Id::X, 10);
    EXPECT_EQ(x, 636038);
    double xd = b->getFieldAs<double>(Dimension::Id::X, 10);
    EXPECT_FLOAT_EQ(xd, 636037.53);
    int32_t y = b->getFieldAs<int32_t>(Dimension::Id::Y, 10);
    EXPECT_EQ(y, 849338);
}

//
// TEST(CompressionTest, test_compress_copied_buffer)
// {
//     using namespace pdal;
//
//
//     const std::string file(Support::datapath("las/1.2-with-color.las"));
//     const Option opt_filename("filename", file);
//     Options opts;
//     opts.add(opt_filename);
//     drivers::las::Reader reader;
//     reader.setOptions(opts);
//     PointContext fctx;
//     reader.prepare(fctx);
//     PointBufferSet buffers = reader.execute(fctx);
//     PointBufferPtr buffer = *buffers.begin();
//
//     PointContext ctx;
//     ctx.registerDim(Dimension::Id::X);
//     ctx.registerDim(Dimension::Id::Y);
//     ctx.registerDim(Dimension::Id::Z);
//     PointBuffer new_buffer(ctx);
// //
//     for (PointId i = 0; i < buffer->size(); ++i)
//     {
//         new_buffer.setField(Dimension::Id::X, i, buffer->getFieldAs<double>(Dimension::Id::X, i));
//         new_buffer.setField(Dimension::Id::Y, i, buffer->getFieldAs<double>(Dimension::Id::Y, i));
//         new_buffer.setField(Dimension::Id::Z, i, buffer->getFieldAs<double>(Dimension::Id::Z, i));
//     }
//     SQLiteTestStream s;
//     compression::Compress<SQLiteTestStream>(ctx, new_buffer, s, compression::CompressionType::Lazperf, 0, 0);
//     SQLiteTestStream s2;
//     s2.buf = s.buf;
//     PointBufferPtr b = compression::Decompress<SQLiteTestStream>(ctx, s2, 11, compression::CompressionType::Lazperf);
//
//     int32_t y = b->getFieldAs<int32_t>(Dimension::Id::Y, 10);
//     EXPECT_EQ(y, 849338);
//     int32_t x = b->getFieldAs<int32_t>(Dimension::Id::X, 10);
//     EXPECT_EQ(x, 636038);
//     double xd = b->getFieldAs<double>(Dimension::Id::X, 10);
//     EXPECT_FLOAT_EQ(xd, 636037.53);
//
// }
//
//
//
// TEST(CompressionTest, test_compress_simple_buffer)
// {
//     using namespace pdal;
//
//     PointContext ctx;
//     ctx.registerDim(Dimension::Id::X);
//     ctx.registerDim(Dimension::Id::Y);
//     ctx.registerDim(Dimension::Id::Z);
//     ctx.registerDim(Dimension::Id::GpsTime);
//     ctx.registerDim(Dimension::Id::Intensity);
//     ctx.registerDim(Dimension::Id::PointSourceId);
//     ctx.registerDim(Dimension::Id::ScanAngleRank);
//     ctx.registerDim(Dimension::Id::Red);
//     ctx.registerDim(Dimension::Id::Green);
//     ctx.registerDim(Dimension::Id::Blue);
//     ctx.registerDim(Dimension::Id::ReturnNumber);
//     ctx.registerDim(Dimension::Id::NumberOfReturns);
//     ctx.registerDim(Dimension::Id::ScanDirectionFlag);
//     ctx.registerDim(Dimension::Id::EdgeOfFlightLine);
//     ctx.registerDim(Dimension::Id::Classification);
//     ctx.registerDim(Dimension::Id::UserData);
//     PointBuffer buffer(ctx);
//     for (PointId i = 0; i < 100; ++i)
//     {
//         buffer.setField(Dimension::Id::X, i, i);
//         buffer.setField(Dimension::Id::Y, i, i+100);
//         buffer.setField(Dimension::Id::Z, i, i+1000);
//         buffer.setField(Dimension::Id::GpsTime, i, i+10000);
//         buffer.setField(Dimension::Id::Intensity, i, 600);
//         buffer.setField(Dimension::Id::PointSourceId, i, 60);
//         buffer.setField(Dimension::Id::ScanAngleRank, i, 1003.23);
//         buffer.setField(Dimension::Id::Red, i, 26);
//         buffer.setField(Dimension::Id::Green, i, 42);
//         buffer.setField(Dimension::Id::Blue, i, 255);
//         buffer.setField(Dimension::Id::ReturnNumber, i, 2);
//         buffer.setField(Dimension::Id::NumberOfReturns, i, 2);
//         buffer.setField(Dimension::Id::ScanDirectionFlag, i, 1);
//         buffer.setField(Dimension::Id::EdgeOfFlightLine, i, 1);
//         buffer.setField(Dimension::Id::Classification, i, 2);
//         buffer.setField(Dimension::Id::UserData, i, 25);
//     }
//
//     SQLiteTestStream s;
//     compression::Compress<SQLiteTestStream>(ctx, buffer, s, compression::CompressionType::Lazperf, 0, 0);
//     SQLiteTestStream s2;
//     s2.buf = s.buf;
//     PointBufferPtr b = compression::Decompress<SQLiteTestStream>(ctx, s2, 11, compression::CompressionType::Lazperf);
// //     std::cout << *b << std::endl;
//     uint16_t r = b->getFieldAs<uint16_t>(Dimension::Id::Red, 10);
//     EXPECT_EQ(r, 26u);
// }
