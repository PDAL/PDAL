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

#include <pdal/pdal_features.hpp>
#include <pdal/Filter.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/FileUtils.hpp>
#include <io/LasHeader.hpp>
#include <io/LasReader.hpp>
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

    auto s = f.createStage("readers.las");
    EXPECT_TRUE(s);
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
    las::Header h = reader.header();

    EXPECT_EQ(h.magic, "LASF");
    EXPECT_EQ(h.fileSourceId, 0);
    EXPECT_TRUE(h.projectGuid.isNull());
    EXPECT_EQ(h.versionMajor, 1);
    EXPECT_EQ(h.versionMinor, 2);
    EXPECT_EQ(h.creationDoy, 0);
    EXPECT_EQ(h.creationYear, 0);
    EXPECT_EQ(h.vlrOffset, 227);
    EXPECT_EQ(h.pointFormatBits, 3);
    EXPECT_EQ(h.pointCount(), 1065u);
    EXPECT_DOUBLE_EQ(h.scale.x, .01);
    EXPECT_DOUBLE_EQ(h.scale.y, .01);
    EXPECT_DOUBLE_EQ(h.scale.z, .01);
    EXPECT_DOUBLE_EQ(h.offset.x, 0);
    EXPECT_DOUBLE_EQ(h.offset.y, 0);
    EXPECT_DOUBLE_EQ(h.offset.z, 0);
    EXPECT_DOUBLE_EQ(h.bounds.maxx, 638982.55);
    EXPECT_DOUBLE_EQ(h.bounds.maxy, 853535.43);
    EXPECT_DOUBLE_EQ(h.bounds.maxz, 586.38);
    EXPECT_DOUBLE_EQ(h.bounds.minx, 635619.85);
    EXPECT_DOUBLE_EQ(h.bounds.miny, 848899.70);
    EXPECT_DOUBLE_EQ(h.bounds.minz, 406.59);
    EXPECT_EQ(h.dataCompressed(), false);
    EXPECT_EQ(h.legacyPointsByReturn[0], 925u);
    EXPECT_EQ(h.legacyPointsByReturn[1], 114u);
    EXPECT_EQ(h.legacyPointsByReturn[2], 21u);
    EXPECT_EQ(h.legacyPointsByReturn[3], 5u);
    EXPECT_EQ(h.legacyPointsByReturn[4], 0u);
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

    const las::Header& h = reader.header();
    EXPECT_EQ(h.pointFormat(), pointFormat);
    EXPECT_EQ(h.versionMajor, majorVersion);
    EXPECT_EQ(h.versionMinor, minorVersion);

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

    // This string is common for WKT1 and WKT2.  When we move to WKT2
    // completely, this can be fixed.
    std::string testWkt {
         R"(GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433)"
    };

    EXPECT_TRUE(Utils::startsWith(qi.m_srs.getWKT(), testWkt));
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

TEST(LasReaderTest, test_vlr)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("las/lots_of_vlr.las"));
    LasReader reader;
    reader.setOptions(ops1);
    reader.prepare(table);
    reader.execute(table);

    MetadataNode root = reader.getMetadata();
    for (size_t i = 0; i < 390; ++i)
    {
        std::string name("vlr_");
        name += std::to_string(i);
        MetadataNode m = root.findChild(name);
        EXPECT_TRUE(!m.empty()) << "No node " << i;
        m = m.findChild("data");
        EXPECT_TRUE(!m.empty()) << "No value for node " << i;
    }
}


TEST(LasReaderTest, testInvalidFileSignature)
{
    PointTable table;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las.wkt"));
    LasReader reader;
    reader.setOptions(ops1);

    EXPECT_EQ(reader.header().magic, "LASF");
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
    EXPECT_EQ(dimTypes.size(), (size_t)22);

    Dimension::Id color0 = layout->findProprietaryDim("Colors0");
    EXPECT_EQ(layout->dimType(color0), Dimension::Type::Unsigned16);
    Dimension::Id color1 = layout->findProprietaryDim("Colors1");
    EXPECT_EQ(layout->dimType(color1), Dimension::Type::Unsigned16);
    Dimension::Id color2 = layout->findProprietaryDim("Colors2");
    EXPECT_EQ(layout->dimType(color2), Dimension::Type::Unsigned16);

    Dimension::Id flag0 = layout->findProprietaryDim("Flags0");
    EXPECT_EQ(layout->dimType(flag0), Dimension::Type::Signed8);
    Dimension::Id flag1 = layout->findProprietaryDim("Flags1");
    EXPECT_EQ(layout->dimType(flag1), Dimension::Type::Signed8);

    Dimension::Id time2 = layout->findDim("Time");
    EXPECT_EQ(layout->dimType(time2), Dimension::Type::Unsigned64);

    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), (size_t)1);
    PointViewPtr view = *viewSet.begin();

    Dimension::Id red = layout->findDim("Red");
    Dimension::Id green = layout->findDim("Green");
    Dimension::Id blue = layout->findDim("Blue");

    Dimension::Id returnNum = layout->findDim("ReturnNumber");
    Dimension::Id numReturns = layout->findDim("NumberOfReturns");

    Dimension::Id intensity = layout->findDim("Intensity");
    Dimension::Id time = layout->findDim("GpsTime");

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        ASSERT_EQ(view->getFieldAs<uint16_t>(red, idx),
            view->getFieldAs<uint16_t>(color0, idx));
        ASSERT_EQ(view->getFieldAs<uint16_t>(green, idx),
            view->getFieldAs<uint16_t>(color1, idx));
        ASSERT_EQ(view->getFieldAs<uint16_t>(blue, idx),
            view->getFieldAs<uint16_t>(color2, idx));

        ASSERT_EQ(view->getFieldAs<uint16_t>(flag0, idx),
            view->getFieldAs<uint16_t>(returnNum, idx));
        ASSERT_EQ(view->getFieldAs<uint16_t>(flag1, idx),
            view->getFieldAs<uint16_t>(numReturns, idx));

        // Time was written truncated rather than rounded.
        ASSERT_NEAR(view->getFieldAs<double>(time, idx),
            view->getFieldAs<double>(time2, idx), 1.0);
    }
}

TEST(LasReaderTest, noextra)
{
    Options ro;
    ro.add("filename", Support::datapath("las/autzen_trim.las"));
    ro.add("extra_dims", "Foo=uint32_t");

    LasReader r;
    r.setOptions(ro);

    PointTable t;
    EXPECT_THROW(r.prepare(t), pdal_error);
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

// LAZ files are normally written in chunks of 50,000, so a file of size
// 110,000 ensures we read some whole chunks and a partial.
TEST(LasReaderTest, lazperf)
{
    Options ops1;
    ops1.add("filename", Support::datapath("laz/autzen_trim.laz"));
    ops1.add("compression", "lazperf");

    LasReader lazReader;
    lazReader.setOptions(ops1);

    PointTable t1;
    lazReader.prepare(t1);
    PointViewSet pbSet = lazReader.execute(t1);
    EXPECT_EQ(pbSet.size(), 1UL);
    PointViewPtr view1 = *pbSet.begin();
    EXPECT_EQ(view1->size(), (point_count_t)110000);

    Options ops2;
    ops2.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader lasReader;
    lasReader.setOptions(ops2);

    PointTable t2;
    lasReader.prepare(t2);
    pbSet = lasReader.execute(t2);
    EXPECT_EQ(pbSet.size(), 1UL);
    PointViewPtr view2 = *pbSet.begin();
    EXPECT_EQ(view2->size(), (point_count_t)110000);

    DimTypeList dims = view1->dimTypes();
    size_t pointSize = view1->pointSize();
    EXPECT_EQ(view1->pointSize(), view2->pointSize());
    // Validate some point data.
    std::unique_ptr<char> buf1(new char[pointSize]);
    std::unique_ptr<char> buf2(new char[pointSize]);
    for (PointId i = 0; i < 110000; i += 100)
    {
       view1->getPackedPoint(dims, i, buf1.get());
       view2->getPackedPoint(dims, i, buf2.get());
       EXPECT_EQ(memcmp(buf1.get(), buf2.get(), pointSize), 0);
    }
}

void streamTest(const std::string src)
{
    Options ops1;
    ops1.add("filename", src);

    LasReader lasReader;
    lasReader.setOptions(ops1);

    PointTable t;
    lasReader.prepare(t);
    PointViewSet s = lasReader.execute(t);
    PointViewPtr p = *s.begin();

    class Checker : public Filter, public Streamable
    {
    public:
        std::string getName() const
            { return "checker"; }
        Checker(PointViewPtr v) : m_cnt(0), m_view(v),
            m_bulkBuf(v->pointSize()), m_buf(v->pointSize()),
            m_dims(v->dimTypes())
            {}
    private:
        point_count_t m_cnt;
        PointViewPtr m_view;
        std::vector<char> m_bulkBuf;
        std::vector<char> m_buf;
        DimTypeList m_dims;

        bool processOne(PointRef& point)
        {
            PointRef bulkPoint = m_view->point(m_cnt);

            bulkPoint.getPackedData(m_dims, m_bulkBuf.data());
            point.getPackedData(m_dims, m_buf.data());
            EXPECT_EQ(memcmp(m_buf.data(), m_bulkBuf.data(),
                m_view->pointSize()), 0);
            m_cnt++;
            return true;
        }

        void done(PointTableRef)
        {
            EXPECT_EQ(m_cnt, 110000u);
        }
    };

    Options ops2;
    ops2.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader lazReader;
    lazReader.setOptions(ops2);

    Checker c(p);
    c.setInput(lazReader);

    FixedPointTable fixed(100);
    c.prepare(fixed);
    c.execute(fixed);
}

TEST(LasReaderTest, stream)
{
    // Compression option is ignored for non-compressed file.
    streamTest(Support::datapath("las/autzen_trim.las"));
    streamTest(Support::datapath("laz/autzen_trim.laz"));
}


// The header of 1.2-with-color-clipped says that it has 1065 points,
// but it really only has 1064.
TEST(LasReaderTest, LasHeaderIncorrectPointcount)
{
    PointTable table;

    Options readOps;
    readOps.add("filename", Support::datapath("las/1.2-with-color-clipped.las"));
    LasReader reader;
    reader.setOptions(readOps);

    EXPECT_THROW(reader.prepare(table), pdal_error);
}

TEST(LasReaderTest, EmptyGeotiffVlr)
{
    PointTable table;

    Options readOps;
    readOps.add("filename", Support::datapath("las/1.2-empty-geotiff-vlrs.las"));
    LasReader reader;
    reader.setOptions(readOps);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(43u, view->size());

}

TEST(LasReaderTest, IgnoreVLRs)
{
    PointTable table;

    Options readOps;
    readOps.add("filename", Support::datapath("las/lots_of_vlr.las"));
    readOps.add("ignore_vlr", "Merrick");
    LasReader reader;
    reader.setOptions(readOps);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);

    // First two VLRs are SRS info, the other 388 would be
    // Merrick ones that we want to ignore/remove
    MetadataNode root = reader.getMetadata();
    for (size_t i = 2; i < 390; ++i)
    {
        std::string name("vlr_");
        name += std::to_string(i);
        MetadataNode m = root.findChild(name);
        EXPECT_FALSE(!m.empty()) << "No node " << i;
        m = m.findChild("data");
        EXPECT_FALSE(!m.empty()) << "No value for node " << i;
    }
}

TEST(LasReaderTest, SyntheticPoints)
{
    using namespace Dimension;

    PointTable table;

    Options readOps;
    readOps.add("filename", Support::datapath("las/synthetic_test.las"));
    LasReader reader;
    reader.setOptions(readOps);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr outView = *viewSet.begin();

    EXPECT_EQ(ClassLabel::CreatedNeverClassified | ClassLabel::Synthetic,
        outView->getFieldAs<uint8_t>(Id::Classification, 0));
}

TEST(LasReaderTest, Start)
{
    // Create a LAZ file that increments XYZ
    std::string source = Support::temppath("increment.laz");
    StageFactory f;
    {
        Stage *faux = f.createStage("readers.faux");
        Options opts;
        opts.add("mode", "ramp");
        opts.add("bounds", "([0, 69999],[100,70099],[500,70499])");
        opts.add("count", 70000);
        faux->setOptions(opts);

        Stage *las = f.createStage("writers.las");
        Options opts2;
        opts2.add("filename", source);
        las->setOptions(opts2);
        las->setInput(*faux);

        PointTable t;
        las->prepare(t);
        las->execute(t);
    }

    auto test1 = [source, &f](int start)
    {
        Stage *las = f.createStage("readers.las");
        Options opts;
        opts.add("filename", source);
        opts.add("start", start);
        las->setOptions(opts);

        PointTable t;
        las->prepare(t);
        PointViewSet s = las->execute(t);
        PointViewPtr v = *s.begin();
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::X, 0), start);
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::Y, 0), start + 100);
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::Z, 0), start + 500);
    };
    auto test2 = [source, &f]()
    {
        Stage *las = f.createStage("readers.las");
        Options opts;
        opts.add("filename", source);
        opts.add("start", 70000);
        las->setOptions(opts);

        PointTable t;
        las->prepare(t);
        PointViewSet s = las->execute(t);
        PointViewPtr v = *s.begin();
        EXPECT_EQ(v->size(), (point_count_t)0);
    };
    auto test3 = [&f](int start, float xval, float yval, float zval)
    {
        Stage *las = f.createStage("readers.las");
        Options opts;
        opts.add("filename", Support::datapath("laz/ellipsoid.copc.laz"));
        opts.add("start", start);
        las->setOptions(opts);

        PointTable t;
        las->prepare(t);
        PointViewSet s = las->execute(t);
        PointViewPtr v = *s.begin();
        EXPECT_EQ(v->getFieldAs<float>(Dimension::Id::X, 0), xval);
        EXPECT_EQ(v->getFieldAs<float>(Dimension::Id::Y, 0), yval);
        EXPECT_EQ(v->getFieldAs<float>(Dimension::Id::Z, 0), zval);
    };

    std::vector<int> starts {0, 49999, 50000, 62520, 2525, 69999};
    for (auto i : starts)
        test1(i);
    test2();
    test3(66271, -8242595.58f, 4966706.0f, 0.28f);
    test3(66272, -8242746.0f, 4966605.44f, -0.28f);
    test3(96000, -8242474.88f, 4966662.72f, -8.1f);

    // Delete the created file.
    FileUtils::deleteFile(source);
}
