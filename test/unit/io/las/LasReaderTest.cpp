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

#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <LasReader.hpp>
#include "Support.hpp"

using namespace pdal;

namespace {
template<typename LeftIter, typename RightIter>
::testing::AssertionResult CheckEqualCollections(LeftIter left_begin,
    LeftIter left_end, RightIter right_begin)
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

} // unnamed namespace

TEST(LasReaderTest, create)
{
    StageFactory f;

    std::unique_ptr<Stage> s(f.createStage("readers.las"));
    EXPECT_TRUE(s.get());
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
    PointTable table;
    Options ops;
    ops.add("filename", Support::datapath("las/simple.las"));

    LasReader reader;
    reader.setOptions(ops);

    reader.prepare(table);
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
    EXPECT_DOUBLE_EQ(h.scaleX(), .01);
    EXPECT_DOUBLE_EQ(h.scaleY(), .01);
    EXPECT_DOUBLE_EQ(h.scaleZ(), .01);
    EXPECT_DOUBLE_EQ(h.offsetX(), 0);
    EXPECT_DOUBLE_EQ(h.offsetY(), 0);
    EXPECT_DOUBLE_EQ(h.offsetZ(), 0);
    EXPECT_DOUBLE_EQ(h.maxX(), 638982.55);
    EXPECT_DOUBLE_EQ(h.maxY(), 853535.43);
    EXPECT_DOUBLE_EQ(h.maxZ(), 586.38);
    EXPECT_DOUBLE_EQ(h.minX(), 635619.85);
    EXPECT_DOUBLE_EQ(h.minY(), 848899.70);
    EXPECT_DOUBLE_EQ(h.minZ(), 406.59);
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
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    ops1.add("count", 103);
    LasReader reader;
    reader.setOptions(ops1);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    Support::check_p0_p1_p2(*view);
    PointViewPtr view2 = view->makeNew();
    view2->appendPoint(*view, 100);
    view2->appendPoint(*view, 101);
    view2->appendPoint(*view, 102);
    Support::check_p100_p101_p102(*view2);
}


static void test_a_format(const std::string& file, uint8_t majorVersion,
    uint8_t minorVersion, int pointFormat,
    double xref, double yref, double zref, double tref,
    uint16_t rref,  uint16_t gref,  uint16_t bref)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath(file));
    ops1.add("count", 1);
    LasReader reader;
    reader.setOptions(ops1);
    reader.prepare(table);

    EXPECT_EQ(reader.header().pointFormat(), pointFormat);
    EXPECT_EQ(reader.header().versionMajor(), majorVersion);
    EXPECT_EQ(reader.header().versionMinor(), minorVersion);

    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1u);

    Support::check_pN(*view, 0, xref, yref, zref, tref, rref, gref, bref);
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

    std::string testWkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";

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
    EXPECT_TRUE(CheckEqualCollections(qi.m_dimNames.begin(),
        qi.m_dimNames.end(), std::begin(dims)));
}

//ABELL - Find another way to do this.
/**
TEST(LasReaderTest, test_vlr)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("las/lots_of_vlr.las"));
    LasReader reader;
    reader.setOptions(ops1);
    reader.prepare(table);
    reader.execute(table);

    EXPECT_EQ(reader.header().getVLRs().getAll().size(), 390);
}
**/


TEST(LasReaderTest, testInvalidFileSignature)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las.wkt"));
    LasReader reader;
    reader.setOptions(ops1);

    EXPECT_TRUE(reader.header().valid());
}

TEST(LasReaderTest, extraBytes)
{
    PointTable table;
    PointLayoutPtr layout(table.layout());

    Options readOps;
    readOps.add("filename", Support::datapath("las/extrabytes.las"));
    LasReader reader;
    reader.setOptions(readOps);

    reader.prepare(table);

    DimTypeList dimTypes = layout->dimTypes();
    EXPECT_EQ(dimTypes.size(), (size_t)24);

    Dimension::Id::Enum color0 = layout->findProprietaryDim("Colors0");
    EXPECT_EQ(layout->dimType(color0), Dimension::Type::Unsigned16);
    Dimension::Id::Enum color1 = layout->findProprietaryDim("Colors1");
    EXPECT_EQ(layout->dimType(color1), Dimension::Type::Unsigned16);
    Dimension::Id::Enum color2 = layout->findProprietaryDim("Colors2");
    EXPECT_EQ(layout->dimType(color2), Dimension::Type::Unsigned16);

    Dimension::Id::Enum flag0 = layout->findProprietaryDim("Flags0");
    EXPECT_EQ(layout->dimType(flag0), Dimension::Type::Signed8);
    Dimension::Id::Enum flag1 = layout->findProprietaryDim("Flags1");
    EXPECT_EQ(layout->dimType(flag1), Dimension::Type::Signed8);

    Dimension::Id::Enum intense2 = layout->findProprietaryDim("Intensity");
    EXPECT_EQ(layout->dimType(intense2), Dimension::Type::Unsigned32);

    Dimension::Id::Enum time2 = layout->findProprietaryDim("Time");
    EXPECT_EQ(layout->dimType(time2), Dimension::Type::Unsigned64);

    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), (size_t)1);
    PointViewPtr view = *viewSet.begin();

    Dimension::Id::Enum red = layout->findDim("Red");
    Dimension::Id::Enum green = layout->findDim("Green");
    Dimension::Id::Enum blue = layout->findDim("Blue");

    Dimension::Id::Enum returnNum = layout->findDim("ReturnNumber");
    Dimension::Id::Enum numReturns = layout->findDim("NumberOfReturns");

    Dimension::Id::Enum intensity = layout->findDim("Intensity");
    Dimension::Id::Enum time = layout->findDim("GpsTime");

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        EXPECT_EQ(view->getFieldAs<uint16_t>(red, idx),
            view->getFieldAs<uint16_t>(color0, idx));
        EXPECT_EQ(view->getFieldAs<uint16_t>(green, idx),
            view->getFieldAs<uint16_t>(color1, idx));
        EXPECT_EQ(view->getFieldAs<uint16_t>(blue, idx),
            view->getFieldAs<uint16_t>(color2, idx));

        EXPECT_EQ(view->getFieldAs<uint16_t>(flag0, idx),
            view->getFieldAs<uint16_t>(returnNum, idx));
        EXPECT_EQ(view->getFieldAs<uint16_t>(flag1, idx),
            view->getFieldAs<uint16_t>(numReturns, idx));

        EXPECT_EQ(view->getFieldAs<uint16_t>(intensity, idx),
            view->getFieldAs<uint16_t>(intense2, idx));

        // Time was written truncated rather than rounded.
        EXPECT_NEAR(view->getFieldAs<double>(time, idx),
            view->getFieldAs<double>(time2, idx), 1.0);

    }
}

TEST(LasReaderTest, callback)
{
    PointTable table;
    point_count_t count = 0;

    Options ops;
    ops.add("filename", Support::datapath("las/simple.las"));

    Reader::PointReadFunc cb = [&count](PointView& view, PointId id)
    {
        count++;
    };
    LasReader reader;
    reader.setOptions(ops);
    reader.setReadCb(cb);

    reader.prepare(table);
    reader.execute(table);
    EXPECT_EQ(count, (point_count_t)1065);
}


// The header of 1.2-with-color-clipped says that it has 1065 points,
// but it really only has 1064.
TEST(LasReaderTest, LasHeaderIncorrentPointcount)
{
    PointTable table;

    Options readOps;
    readOps.add("filename", Support::datapath("las/1.2-with-color-clipped.las"));
    LasReader reader;
    reader.setOptions(readOps);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1064u, view->size());
}
