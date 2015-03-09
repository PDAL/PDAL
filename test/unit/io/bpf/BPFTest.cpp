/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Utils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <BpfReader.hpp>
#include <BpfWriter.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

template<typename LeftIter, typename RightIter>
::testing::AssertionResult CheckEqualCollections(
    LeftIter left_begin, LeftIter left_end, RightIter right_begin)
{
    bool equal(true);
    std::string message;
    size_t index(0);
    while (left_begin != left_end)
    {
        if (*left_begin++ != *right_begin++)
        {
            equal = false;
            message += "\n\tMismatch at index " + std::to_string(index);
        }
        ++index;
    }
    if (message.size())
        message += "\n\t";
    return equal ? ::testing::AssertionSuccess() :
        ::testing::AssertionFailure() << message;
}

void test_file_type(const std::string& filename)
{
    PointContext context;

    Options ops;

    ops.add("filename", filename);
    ops.add("count", 506);
    std::shared_ptr<BpfReader> reader(new BpfReader);
    reader->setOptions(ops);

    reader->prepare(context);
    PointBufferSet pbSet = reader->execute(context);

    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 506u);

    struct PtData
    {
        float x;
        float y;
        float z;
    };

    PtData pts2[3] = { {494057.312, 4877433.5, 130.630005},
                       {494133.812, 4877440, 130.440002},
                       {494021.094, 4877440, 130.460007} };

    for (int i = 0; i < 3; ++i)
    {
        float x = buf->getFieldAs<float>(Dimension::Id::X, i);
        float y = buf->getFieldAs<float>(Dimension::Id::Y, i);
        float z = buf->getFieldAs<float>(Dimension::Id::Z, i);

        EXPECT_FLOAT_EQ(x, pts2[i].x);
        EXPECT_FLOAT_EQ(y, pts2[i].y);
        EXPECT_FLOAT_EQ(z, pts2[i].z);
    }

    PtData pts[3] = { {494915.25, 4878096.5, 128.220001},
                      {494917.062, 4878124.5, 128.539993},
                      {494920.781, 4877914.5, 127.43} };

    for (int i = 503; i < 3; ++i)
    {
        float x = buf->getFieldAs<float>(Dimension::Id::X, i);
        float y = buf->getFieldAs<float>(Dimension::Id::Y, i);
        float z = buf->getFieldAs<float>(Dimension::Id::Z, i);

        EXPECT_FLOAT_EQ(x, pts[i].x);
        EXPECT_FLOAT_EQ(y, pts[i].y);
        EXPECT_FLOAT_EQ(z, pts[i].z);
    }
}

void test_roundtrip(Options& writerOps)
{
    std::string infile(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
    std::string outfile(Support::temppath("tmp.bpf"));


    PointContext context;

    Options readerOps;

    readerOps.add("filename", infile);
    BpfReader reader;
    reader.setOptions(readerOps);

    writerOps.add("filename", outfile);
    BpfWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    FileUtils::deleteFile(outfile);
    writer.prepare(context);
    writer.execute(context);

    test_file_type(outfile);
}


} //namespace

TEST(BPFTest, test_point_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
}

TEST(BPFTest, test_dim_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3.bpf"));
}

TEST(BPFTest, test_byte_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-segregated.bpf"));
}

TEST(BPFTest, test_point_major_zlib)
{
    test_file_type(
        Support::datapath("bpf/"
            "autzen-utm-chipped-25-v3-deflate-interleaved.bpf"));
}

TEST(BPFTest, test_dim_major_zlib)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-deflate.bpf"));
}

TEST(BPFTest, test_byte_major_zlib)
{
    test_file_type(
        Support::datapath("bpf/"
            "autzen-utm-chipped-25-v3-deflate-segregated.bpf"));
}

TEST(BPFTest, roundtrip_byte)
{
    Options ops;

    ops.add("format", "BYTE");
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_dimension)
{
    Options ops;

    ops.add("format", "DIMENSION");
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_point)
{
    Options ops;

    ops.add("format", "POINT");
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_byte_compression)
{
    Options ops;

    ops.add("format", "BYTE");
    ops.add("compression", true);
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_dimension_compression)
{
    Options ops;

    ops.add("format", "DIMENSION");
    ops.add("compression", true);
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_point_compression)
{
    Options ops;

    ops.add("format", "POINT");
    ops.add("compression", true);
    test_roundtrip(ops);
}

TEST(BPFTest, roundtrip_scaling)
{
    Options ops;

    ops.add("format", "POINT");
    ops.add("offset_x", 494000.0);
    ops.add("offset_y", 487000.0);
    ops.add("offset_z", 130.0);
    ops.add("scale_x", .001);
    ops.add("scale_y", .01);
    ops.add("scale_z", 10.0);
    test_roundtrip(ops);
}

TEST(BPFTest, inspect)
{
    Options ops;
    ops.add("filename", Support::datapath("bpf/autzen-dd.bpf"));

    BpfReader reader;
    reader.setOptions(ops);

    QuickInfo qi = reader.preview();

    std::string testWkt = "PROJCS[\"WGS 84 / SCAR IMW ST05-08\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"standard_parallel_1\",-76.66666666666667],PARAMETER[\"standard_parallel_2\",-79.33333333333333],PARAMETER[\"latitude_of_origin\",-90],PARAMETER[\"central_meridian\",-144],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH],AUTHORITY[\"EPSG\",\"3261\"]]";
    EXPECT_EQ(qi.m_srs.getWKT(), testWkt);

    EXPECT_EQ(qi.m_pointCount, 1065u);

    BOX3D bounds(
        -13676090.610841721296, 4894836.9556098170578, 123.93000030517578125,
        -13674705.011110275984, 4896224.6888861842453, 178.7299957275390625);
    EXPECT_EQ(qi.m_bounds, bounds);

    const char *dims[] = 
    {
        "Blue",
        "Classification",
        "GPSTime",
        "Green",
        "Intensity",
        "Number of Returns",
        "Red",
        "Return Information",
        "Return Number",
        "X",
        "Y",
        "Z"
    };

    std::sort(qi.m_dimNames.begin(), qi.m_dimNames.end());
    EXPECT_TRUE(CheckEqualCollections(qi.m_dimNames.begin(),
        qi.m_dimNames.end(), std::begin(dims)));
}

TEST(BPFTest, mueller)
{
    BpfMuellerMatrix xform;

    double x = 12.345;
    double y = 45.345;
    double z = 999.341;

    double xp = x;
    double yp = y;
    double zp = z;
    xform.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(x, xp);
    EXPECT_DOUBLE_EQ(y, yp);
    EXPECT_DOUBLE_EQ(z, zp);

    // Test translation.
    xp = 1.0;
    yp = 1.0;
    zp = 1.0;

    BpfMuellerMatrix translate;

    translate.m_vals[3] = 2.0;
    translate.m_vals[7] = 2.0;
    translate.m_vals[11] = 1.0;
    translate.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(xp, 3.0);
    EXPECT_DOUBLE_EQ(yp, 3.0);
    EXPECT_DOUBLE_EQ(zp, 2.0);

    BpfMuellerMatrix scale;
    xp = 1.0;
    yp = 1.0;
    zp = 1.0;
    scale.m_vals[0] = 2.0;
    scale.m_vals[5] = 7.0;
    scale.m_vals[10] = -3.0;
    scale.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(xp, 2.0);
    EXPECT_DOUBLE_EQ(yp, 7.0);
    EXPECT_DOUBLE_EQ(zp, -3.0);
}

