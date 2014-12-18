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

#include <pdal/PointBuffer.hpp>
#include <LasReader.hpp>
#include "Support.hpp"

using namespace pdal;

namespace {
template<typename LeftIter, typename RightIter>
::testing::AssertionResult CheckEqualCollections(LeftIter left_begin, LeftIter left_end, RightIter right_begin)
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
  return equal ? ::testing::AssertionSuccess() : ::testing::AssertionFailure() << message;
}
}

TEST(LasReaderTest, test_base_options)
{
    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const Option opt_filename("filename", file);
    const Option opt_verbose_string("verbose", "99");
    const Option opt_verbose_uint8("verbose", 99);
    const Option opt_debug_string("debug", "true");
    const Option opt_debug_bool("debug", true);

    {
        Options opts;
        opts.add(opt_filename);

        LasReader reader;
        reader.setOptions(opts);
        EXPECT_TRUE(reader.getVerboseLevel() == 0);
        EXPECT_TRUE(reader.isDebug() == false);
    }

    {
        Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_string);
        opts.add(opt_debug_string);
        LasReader reader;
        reader.setOptions(opts);
        EXPECT_TRUE(reader.getVerboseLevel() == 99);
        EXPECT_TRUE(reader.isDebug() == true);
    }

    {
        Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_uint8);
        opts.add(opt_debug_bool);
        LasReader reader;
        reader.setOptions(opts);
        EXPECT_TRUE(reader.getVerboseLevel() == 99);
        EXPECT_TRUE(reader.isDebug() == true);
    }
}


TEST(LasReaderTest, header)
{
    PointContext ctx;
    Options ops;
    ops.add("filename", Support::datapath("las/simple.las"));
    LasReader reader;
    reader.setOptions(ops);

    reader.prepare(ctx);
    // This tests the copy ctor, too.
    LasHeader h = reader.header();

    EXPECT_EQ(h.fileSignature(), "LASF");
    EXPECT_EQ(h.fileSourceId(), 0);
    EXPECT_TRUE(h.projectId().is_nil());
    EXPECT_EQ(h.versionMajor(), 1);
    EXPECT_EQ(h.versionMinor(), 2);
    EXPECT_EQ(h.creationDOY(), 0);
    EXPECT_EQ(h.creationYear(), 0);
    EXPECT_EQ(h.vlrOffset(), 227);
    EXPECT_EQ(h.pointFormat(), 3);
    EXPECT_EQ(h.pointCount(), 1065u);
    EXPECT_FLOAT_EQ(h.scaleX(), .01);
    EXPECT_FLOAT_EQ(h.scaleY(), .01);
    EXPECT_FLOAT_EQ(h.scaleZ(), .01);
    EXPECT_FLOAT_EQ(h.offsetX(), 0);
    EXPECT_FLOAT_EQ(h.offsetY(), 0);
    EXPECT_FLOAT_EQ(h.offsetZ(), 0);
    EXPECT_FLOAT_EQ(h.maxX(), 638982.55);
    EXPECT_FLOAT_EQ(h.maxY(), 853535.43);
    EXPECT_FLOAT_EQ(h.maxZ(), 586.38);
    EXPECT_FLOAT_EQ(h.minX(), 635619.85);
    EXPECT_FLOAT_EQ(h.minY(), 848899.70);
    EXPECT_FLOAT_EQ(h.minZ(), 406.59);
    EXPECT_EQ(h.compressed(), false);
    EXPECT_EQ(h.compressionInfo(), "");
    EXPECT_EQ(h.pointCountByReturn(0), 925u);
    EXPECT_EQ(h.pointCountByReturn(1), 114u);
    EXPECT_EQ(h.pointCountByReturn(2), 21u);
    EXPECT_EQ(h.pointCountByReturn(3), 5u);
    EXPECT_EQ(h.pointCountByReturn(4), 0u);
}


TEST(LasReaderTest, test_sequential)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    ops1.add("count", 103);
    LasReader reader;
    reader.setOptions(ops1);

    EXPECT_TRUE(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();
    Support::check_p0_p1_p2(*buf);
    PointBufferPtr buf2 = buf->makeNew();
    buf2->appendPoint(*buf, 100);
    buf2->appendPoint(*buf, 101);
    buf2->appendPoint(*buf, 102);
    Support::check_p100_p101_p102(*buf2);
}


static void test_a_format(const std::string& file, uint8_t majorVersion, uint8_t minorVersion, int pointFormat,
                          double xref, double yref, double zref, double tref, uint16_t rref,  uint16_t gref,  uint16_t bref)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath(file));
    ops1.add("count", 1);
    LasReader reader;
    reader.setOptions(ops1);
    reader.prepare(ctx);

    EXPECT_EQ(reader.header().pointFormat(), pointFormat);
    EXPECT_EQ(reader.header().versionMajor(), majorVersion);
    EXPECT_EQ(reader.header().versionMinor(), minorVersion);

    PointBufferSet pbSet = reader.execute(ctx);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 1u);

    Support::check_pN(*buf, 0, xref, yref, zref, tref, rref, gref, bref);
}

TEST(LasReaderTest, test_different_formats)
{
    test_a_format("las/permutations/1.0_0.las", 1, 0, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.0_1.las", 1, 0, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("las/permutations/1.1_0.las", 1, 1, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.1_1.las", 1, 1, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("las/permutations/1.2_0.las", 1, 2, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.2_1.las", 1, 2, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);
    test_a_format("las/permutations/1.2_2.las", 1, 2, 2, 470692.440000, 4602888.900000, 16.000000, 0, 255, 12, 234);
    test_a_format("las/permutations/1.2_3.las", 1, 2, 3, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 255, 12, 234);
}


TEST(LasReaderTest, inspect)
{
    Options ops;
    ops.add("filename", Support::datapath("las/epsg_4326.las"));

    LasReader reader;
    reader.setOptions(ops);

    QuickInfo qi = reader.preview();

    std::string testWkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"unretrievable - using WGS84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
#ifdef PDAL_HAVE_LIBGEOTIFF
    EXPECT_EQ(qi.m_srs.getWKT(), testWkt);
#endif // PDAL_HAVE_LIBGEOTIFF

    EXPECT_EQ(qi.m_pointCount, 5380u);

    BOX3D bounds(-94.683465399999989, 31.0367341, 39.081000199999998,
        -94.660631099999989, 31.047329099999999, 78.119000200000002);
    EXPECT_EQ(qi.m_bounds, bounds);

    const char *dims[] =
    {
        "Classification",
        "EdgeOfFlightLine",
        "Intensity",
        "NumberOfReturns",
        "PointSourceId",
        "ReturnNumber",
        "ScanAngleRank",
        "ScanDirectionFlag",
        "UserData",
        "X",
        "Y",
        "Z"
    };

    std::sort(qi.m_dimNames.begin(), qi.m_dimNames.end());
    EXPECT_TRUE(CheckEqualCollections(qi.m_dimNames.begin(), qi.m_dimNames.end(), std::begin(dims)));
}


//ABELL - Find another way to do this.
/**
TEST(LasReaderTest, test_vlr)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/lots_of_vlr.las"));
    LasReader reader;
    reader.setOptions(ops1);
    reader.prepare(ctx);
    reader.execute(ctx);

    EXPECT_EQ(reader.header().getVLRs().getAll().size(), 390);
}
**/


TEST(LasReaderTest, testInvalidFileSignature)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las.wkt"));
    LasReader reader;
    reader.setOptions(ops1);

    EXPECT_TRUE(reader.header().valid());
}
