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

#include <iostream>
#include <string>
#include <random>

#include "Support.hpp"
#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <io/LasReader.hpp>

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


TEST(Compression, simple)
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

    DimTypeList dimTypes = layout->dimTypes();
    auto cb = [&rawBuf](char *buf, size_t bufsize)
    {
        unsigned char *ubuf = reinterpret_cast<unsigned char *>(buf);
        rawBuf.insert(rawBuf.end(), ubuf, ubuf + bufsize);
    };

    LazPerfCompressor compressor(cb, dimTypes);

    std::vector<char> tmpbuf(layout->pointSize());
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        view->getPackedPoint(dimTypes, idx, tmpbuf.data());
        compressor.compress(tmpbuf.data(), layout->pointSize());
    }
    compressor.done();

    EXPECT_EQ(view->size() * layout->pointSize(), (size_t)55380);
    EXPECT_EQ(rawBuf.size(), (size_t)30945);

    PointViewPtr otherView(new PointView(table));
    PointId nextId(0);
    auto cb2 = [&otherView, &dimTypes, &nextId](char *buf, size_t bufsize)
    {

        otherView->setPackedPoint(dimTypes, nextId, buf);
        nextId++;
    };

    LazPerfDecompressor(cb2, dimTypes, view->size()).
        decompress(reinterpret_cast<const char *>(rawBuf.data()),
            rawBuf.size());

    EXPECT_EQ(otherView->size(), 1065U);
    EXPECT_EQ(getBytes(otherView).size(), (size_t)(52 * 1065));

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
    Type types[] = {
        Type::Unsigned8, Type::Unsigned16, Type::Unsigned32, Type::Unsigned64,
        Type::Signed8, Type::Signed16, Type::Signed32, Type::Signed64,
        Type::Float, Type::Double
    };
    // Size is 42.

    std::default_random_engine generator;
    std::uniform_int_distribution<int> dist((std::numeric_limits<int>::min)());
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
    auto cb = [&rawBuf](char *buf, size_t bufsize)
    {
        unsigned char *ubuf = reinterpret_cast<unsigned char *>(buf);
        rawBuf.insert(rawBuf.begin(), ubuf, ubuf + bufsize);
    };

    LazPerfCompressor compressor(cb, dimTypes);
    for (size_t i = 0; i < 50; i++)
    {
        compressor.compress(pts[0], 42);
        compressor.compress(pts[1], 42);
        compressor.compress(pts[2], 42);
    }
    compressor.done();

    char oPts[3][42];
    PointId id = 0;
    auto cb2 = [&pts, &oPts, &id](char *buf, size_t bufsize)
    {
        memcpy(oPts[id++], buf, bufsize);
        if (id == 3)
        {
            EXPECT_EQ(memcmp(pts[0], oPts[0], 42), 0);
            EXPECT_EQ(memcmp(pts[0], oPts[0], 42), 0);
            EXPECT_EQ(memcmp(pts[2], oPts[2], 42), 0);
            memset(oPts[0], 0, 42);
            memset(oPts[1], 0, 42);
            memset(oPts[2], 0, 42);
            id = 0;
        }
    };
    LazPerfDecompressor(cb2, dimTypes, 50 * 3).
           decompress(reinterpret_cast<const char *>(rawBuf.data()),
                       rawBuf.size());
}

