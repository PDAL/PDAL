/******************************************************************************
 * Copyright (c) 2018, Connor Manning (connor@hobu.co)
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

#include <algorithm>

#include <nlohmann/json.hpp>

#include <pdal/pdal_test_main.hpp>

#include <io/EptReader.hpp>
#include <io/LasReader.hpp>
#include <filters/CropFilter.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <pdal/PointViewIter.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

namespace pdal
{

namespace
{
    const BOX3D expBoundsConforming(515368, 4918340, 2322,
            515402, 4918382, 2339);
    const std::string expSrsWkt = R"(PROJCS["NAD83 / UTM zone 12N",GEOGCS["NAD83",DATUM["North_American_Datum_1983",SPHEROID["GRS 1980",6378137,298.257222101,AUTHORITY["EPSG","7019"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY["EPSG","6269"]],PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]],AUTHORITY["EPSG","4269"]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",-111],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",0],UNIT["meter",1,AUTHORITY["EPSG","9001"]],AXIS["Easting",EAST],AXIS["Northing",NORTH],AUTHORITY["EPSG","26912"]])";
    const point_count_t expNumPoints(518862);
    const std::vector<std::string> expDimNames = {
         "X", "Y", "Z", "Intensity", "ReturnNumber", "NumberOfReturns",
         "ScanDirectionFlag", "EdgeOfFlightLine", "Classification",
         "ScanAngleRank", "UserData", "PointSourceId", "GpsTime", "OriginId"
    };

    // Most of our tests will exercise this laszip-based EPT dataset based on
    // a 4-tile split of Lone Star Geyser.
    const std::string sourceFilePath(
            Support::datapath("ept/source/lone-star.laz"));
    const std::string eptLaszipPath(
            "ept://" + Support::datapath("ept/lone-star-laszip"));
    const std::string eptAutzenPath(
            "ept://" + Support::datapath("ept/1.2-with-color"));
    const std::string attributesPath(
            Support::datapath("autzen/attributes.json"));

    // Also test a basic read of binary/zstandard versions of a smaller dataset.
    const std::string ellipsoidEptBinaryPath(
            "ept://" + Support::datapath("ept/ellipsoid-binary"));
    const std::string ellipsoidEptZstandardPath(
            "ept://" + Support::datapath("ept/ellipsoid-zstandard"));

    const point_count_t ellipsoidNumPoints(100000);
    const BOX3D ellipsoidBoundsConforming(-8242746, 4966506, -50,
            -8242446, 4966706, 50);
}

TEST(EptReaderTest, inspect)
{
    Options options;
    options.add("filename", eptLaszipPath);

    EptReader reader;
    reader.setOptions(options);

    const QuickInfo qi(reader.preview());

    EXPECT_TRUE(qi.valid());
    EXPECT_EQ(qi.m_bounds, expBoundsConforming);
    EXPECT_EQ(qi.m_pointCount, expNumPoints);
    EXPECT_TRUE(std::equal(qi.m_dimNames.cbegin(), qi.m_dimNames.cend(),
                expDimNames.cbegin()));

    std::string wkt = qi.m_srs.getWKT();
    // Sometimes we get back "metre" when we're execting "meter".
    while (true)
    {
        auto pos = wkt.find("metre");
        if (pos == std::string::npos)
            break;
        wkt.replace(pos, 5, "meter");
    }
    EXPECT_EQ(wkt, expSrsWkt);
}

TEST(EptReaderTest, fullReadLaszip)
{
    Options options;
    options.add("filename", eptLaszipPath);

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(expBoundsConforming.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, expNumPoints);
}

TEST(EptReaderTest, fullReadBinary)
{
    Options options;
    options.add("filename", ellipsoidEptBinaryPath);

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(ellipsoidBoundsConforming.contains(x, y, z));
            ASSERT_EQ(o, 0u);
        }
    }

    EXPECT_EQ(np, ellipsoidNumPoints);
}

TEST(EptReaderTest, fullReadZstandard)
{
#ifdef PDAL_HAVE_ZSTD
    Options options;
    options.add("filename", ellipsoidEptZstandardPath);

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(ellipsoidBoundsConforming.contains(x, y, z));
            ASSERT_EQ(o, 0u);
        }
    }

    EXPECT_EQ(np, ellipsoidNumPoints);
#endif
}

TEST(EptReaderTest, resolutionLimit)
{
    Options options;
    options.add("filename", eptLaszipPath);

    // Our test data cube is 44 units in length, with a span of 128.  Therefore
    // our resolution cell width values for the first few depths are:
    //      Depth 0: 0.34375
    //      Depth 1: 0.171875
    //      Depth 2: 0.0859375
    //
    // Any resolution option between 0.171875 and 0.0859375 will select all of
    // depths 0, 1, and 2, so we'll test a corresponding query.
    options.add("resolution", 0.1);

    // This expected value corresponds to the sum of the point counts of all
    // files in our dataset whose depth is less than 3.  This value is summed
    // from the hierarchy for depths 0 through 2 (our test dataset has depths
    // through 3, which are omitted here).
    const point_count_t expectedCount = 479269;

    PointTable table;

    EptReader reader;
    reader.setOptions(options);
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;

            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(expBoundsConforming.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, expectedCount);
}

TEST(EptReaderTest, bounds2dXform)
{
    SrsBounds eptBounds(BOX2D(515380, 4918360, 515390, 4918370));
    SrsBounds boxBounds(
        BOX2D(-110.80680478060, 44.418368816508,
              -110.80667887010, 44.418458631945),
        SpatialReference("EPSG:4326"));

    PointViewPtr v1;
    PointViewPtr v2;
    {
        EptReader reader;
        Options options;
        options.add("filename", eptLaszipPath);
        options.add("bounds", eptBounds);
        reader.setOptions(options);
        PointTable eptTable;
        reader.prepare(eptTable);
        auto vset = reader.execute(eptTable);
        v1 = *vset.begin();
    }
    {
        EptReader reader;
        Options options;
        options.add("filename", eptLaszipPath);
        options.add("bounds", boxBounds);
        reader.setOptions(options);
        PointTable eptTable;
        reader.prepare(eptTable);
        auto vset = reader.execute(eptTable);
        v2 = *vset.begin();
    }

    EXPECT_EQ(v1->size(), v2->size());
}

TEST(EptReaderTest, boundedRead2d)
{
    BOX2D bounds(515380, 4918350, 515400, 4918370);

    // First we'll query the EptReader for these bounds.
    EptReader reader;
    {
        Options options;
        options.add("filename", eptLaszipPath);
        options.add("bounds", bounds);
        reader.setOptions(options);
    }
    PointTable eptTable;
    reader.prepare(eptTable);
    const auto set(reader.execute(eptTable));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(bounds.contains(x, y)) << bounds << ": " <<
                x << ", " << y << ", " << z << std::endl;
            ASSERT_TRUE(o < 4);
        }
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", sourceFilePath);
        source.setOptions(options);
    }
    CropFilter crop;
    {
        Options options;
        options.add("bounds", bounds);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    for (const PointViewPtr& view : crop.execute(sourceTable))
    {
        sourceNp += view->size();
    }

    EXPECT_EQ(np, sourceNp);
    EXPECT_EQ(np, 354211u);
}

TEST(EptReaderTest, boundedRead3d)
{
    BOX3D bounds(515380, 4918350, 2320, 515400, 4918370, 2325);

    // First we'll query the EptReader for these bounds.
    EptReader reader;
    {
        Options options;
        options.add("filename", eptLaszipPath);
        options.add("bounds", bounds);
        reader.setOptions(options);
    }
    PointTable eptTable;
    reader.prepare(eptTable);
    const auto set(reader.execute(eptTable));

    double x, y, z;
    uint64_t o;
    uint64_t np(0);
    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ++np;
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            ASSERT_TRUE(bounds.contains(x, y, z)) << bounds << ": " <<
                x << ", " << y << ", " << z << std::endl;
            ASSERT_TRUE(o < 4);
        }
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", sourceFilePath);
        source.setOptions(options);
    }
    CropFilter crop;
    {
        Options options;
        options.add("bounds", bounds);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    // The crop filter only works in 2D, so we'll have to manually count.
    for (const PointViewPtr& view : crop.execute(sourceTable))
    {
        for (uint64_t i(0); i < view->size(); ++i)
        {
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            z = view->getFieldAs<double>(Dimension::Id::Z, i);
            if (bounds.contains(x, y, z))
                ++sourceNp;
        }
    }

    EXPECT_EQ(np, sourceNp);
    EXPECT_EQ(np, 45930u);
}

TEST(EptReaderTest, originRead)
{
    uint64_t np(0);
    for (uint64_t origin(0); origin < 4; ++origin)
    {
        EptReader reader;
        Options options;
        options.add("filename", eptLaszipPath);
        options.add("origin", origin);
        reader.setOptions(options);
        PointTable table;
        reader.prepare(table);
        const auto set(reader.execute(table));

        uint64_t o;
        for (const PointViewPtr& view : set)
        {
            np += view->size();
            for (point_count_t i(0); i < view->size(); ++i)
            {
                o = view->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
                ASSERT_EQ(o, origin);
            }
        }
    }

    EXPECT_EQ(np, expNumPoints);
}

TEST(EptReaderTest, badOriginQuery)
{
    EptReader reader;
    Options options;
    options.add("filename", eptLaszipPath);
    options.add("origin", 4);
    reader.setOptions(options);
    PointTable table;
    EXPECT_THROW(reader.prepare(table), pdal_error);
}

TEST(EptReaderTest, getRemoteType)
{
    NL::json j = {{ "type", "signed" }, { "size", 4 }, { "scale", 1.0 }};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::Signed32);
    j = {{ "scale", "foo" }};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::None);
    j = {{ "type", "float"}, {"size", 2}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::None);
    j = {{ "type", "float"}, {"size", 4}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::Float);
    j = {{ "type", "unsigned"}, {"size", 4}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::Unsigned32);
    j = {{ "type", "signed"}, {"size", 2}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::Signed16);
    j = {{ "tope", "signed"}, {"size", 2}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::None);
    j = {{ "type", "signed"}, {"size", 3}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::None);
    j = {{ "type", "signed"}};
    EXPECT_EQ(EptReader::getRemoteTypeTest(j), Dimension::Type::None);
}

TEST(EptReaderTest, getCoercedType)
{
    // Scaled attributes are coerced to doubles regardless of schema type.
    NL::json j = {{ "type", "signed" }, { "size", 4 }, { "scale", 1.0 }};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::Double);
    j = {{ "scale", "foo" }};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::None);
    j = {{ "type", "float"}, {"size", 2}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::None);
    j = {{ "type", "float"}, {"size", 4}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::Float);
    j = {{ "type", "unsigned"}, {"size", 4}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::Unsigned32);
    j = {{ "type", "signed"}, {"size", 2}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::Signed16);
    j = {{ "tope", "signed"}, {"size", 2}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::None);
    j = {{ "type", "signed"}, {"size", 3}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::None);
    j = {{ "type", "signed"}};
    EXPECT_EQ(EptReader::getCoercedTypeTest(j), Dimension::Type::None);
}

void streamTest(const std::string src)
{
    Options ops;
    ops.add("filename", src);
    ops.add("resolution", 1);

    // Execute the reader in normal non-streaming mode.
    EptReader normalReader;
    normalReader.setOptions(ops);
    PointTable normalTable;
    const auto nodeIdDim = normalTable.layout()->registerOrAssignDim(
        "EptNodeId",
        Dimension::Type::Unsigned32);
    const auto pointIdDim = normalTable.layout()->registerOrAssignDim(
        "EptPointId",
        Dimension::Type::Unsigned32);
    normalReader.prepare(normalTable);
    const auto views = normalReader.execute(normalTable);
    PointView& normalView = **views.begin();

    // A table that satisfies the streaming interface and simply adds the data
    // to a normal PointView.  We'll compare the result with the PointView
    // resulting from standard execution.
    class TestPointTable : public StreamPointTable
    {
    public:
        TestPointTable(PointView& view)
            : StreamPointTable(*view.table().layout(), 1024)
            , m_view(view)
        { }

    protected:
        virtual void reset() override
        {
            m_offset += numPoints();
        }

        virtual char* getPoint(PointId index) override
        {
            return m_view.getOrAddPoint(m_offset + index);
        }

        PointView& m_view;
        PointId m_offset = 0;
    };

    // Execute the reder in streaming mode.
    EptReader streamReader;
    streamReader.setOptions(ops);
    std::vector<char> streamBuffer;
    PointTable streamTable;
    PointView streamView(streamTable);
    TestPointTable testTable(streamView);
    const auto streamNodeIdDim = streamTable.layout()->registerOrAssignDim(
        "EptNodeId",
        Dimension::Type::Unsigned32);
    const auto streamPointIdDim = streamTable.layout()->registerOrAssignDim(
        "EptPointId",
        Dimension::Type::Unsigned32);

    ASSERT_EQ(streamNodeIdDim, nodeIdDim);
    ASSERT_EQ(streamPointIdDim, pointIdDim);

    streamReader.prepare(testTable);
    streamReader.execute(testTable);

    // Make sure our non-streaming and streaming views are identical, note that
    // we'll need to sort them since the EPT reader loads data asynchronously
    // so we can't rely on their order being the same.
    ASSERT_EQ(streamView.size(), normalView.size());
    ASSERT_EQ(
        streamTable.layout()->pointSize(),
        normalTable.layout()->pointSize());

    const std::size_t numPoints(normalView.size());
    const std::size_t pointSize(normalTable.layout()->pointSize());

    const auto sort([nodeIdDim, pointIdDim]
        (const PointIdxRef& a, const PointIdxRef& b)
    {
        if (a.compare(nodeIdDim, b)) return true;
        return !b.compare(nodeIdDim, a) && a.compare(pointIdDim, b);
    });
    std::stable_sort(normalView.begin(), normalView.end(), sort);
    std::stable_sort(streamView.begin(), streamView.end(), sort);

    for (PointId i(0); i < normalView.size(); ++i)
    {
        for (const auto& id : normalTable.layout()->dims())
        {
            ASSERT_EQ(
                normalView.getFieldAs<double>(id, i),
                streamView.getFieldAs<double>(id, i));
        }
    }
}

TEST(EptReaderTest, binaryStream)
{
    streamTest(ellipsoidEptBinaryPath);
}

TEST(EptReaderTest, laszipStream)
{
#ifdef PDAL_HAVE_LASZIP
    streamTest(eptLaszipPath);
#endif
}

TEST(EptReaderTest, zstandardStream)
{
#ifdef PDAL_HAVE_ZSTANDARD
    streamTest(ellipsoidEptZstandardPath);
#endif
}

TEST(EptReaderTest, boundedCrop)
{
    std::string wkt = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection.wkt"));

    // First we'll query the EptReader for these bounds.
    EptReader reader;
    {
        Options options;
        options.add("filename", eptAutzenPath);
        std::string overrides(wkt + "/ EPSG:3644");
        Option polygon("polygon", overrides);
        options.add(polygon);
        reader.setOptions(options);
    }

    PointTable eptTable;
    reader.prepare(eptTable);

    uint64_t eptNp(0);
    for (const PointViewPtr& view : reader.execute(eptTable))
    {
        eptNp += view->size();
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", Support::datapath("las/1.2-with-color.las"));
        source.setOptions(options);
    }
    CropFilter crop;
    {
        std::string overrides(wkt + "/ EPSG:3644");
        Options options;
        Option polygon("polygon", overrides);
        options.add(polygon);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    for (const PointViewPtr& view : crop.execute(sourceTable))
    {
        sourceNp += view->size();
    }

    EXPECT_EQ(eptNp, sourceNp);
    EXPECT_EQ(eptNp, 47u);
    EXPECT_EQ(sourceNp, 47u);
}

TEST(EptReaderTest, boundedCropReprojection)
{
    std::string selection = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection.wkt"));
    std::string selection4326 = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection-dd.wkt"));
    std::string srs = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-srs.wkt"));

    EptReader reader;
    {
        Options options;
        options.add("filename", eptAutzenPath);
        options.add("override_srs", srs);
        options.add("polygon", selection4326 + "/EPSG:4326");
        reader.setOptions(options);
    }

    PointTable eptTable;

    reader.prepare(eptTable);

    uint64_t eptNp(0);
    for (const PointViewPtr& view : reader.execute(eptTable))
        eptNp += view->size();

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", Support::datapath("las/1.2-with-color.las"));
        options.add("override_srs", srs);
        source.setOptions(options);
    }

    ReprojectionFilter reproj;
    {
        Options options;
        options.add("out_srs", "EPSG:4326");
        reproj.setOptions(options);
        reproj.setInput(source);
    }

    CropFilter crop;
    {
        Options options;
        options.add("polygon", selection4326);
        options.add("a_srs", "EPSG:4326");
        crop.setOptions(options);
        crop.setInput(reproj);
    }

    PointTable sourceTable;
    crop.prepare(sourceTable);
    uint64_t sourceNp(0);
    for (const PointViewPtr& view : crop.execute(sourceTable))
        sourceNp += view->size();

    EXPECT_EQ(eptNp, sourceNp);
    EXPECT_EQ(eptNp, 47u);
    EXPECT_EQ(sourceNp, 47u);
}


TEST(EptReaderTest, ogrCrop)
{

    EptReader reader;
    {
        Options options;
        options.add("filename", eptAutzenPath);
        NL::json ogr;
        ogr["drivers"] = {"GeoJSON"};
        ogr["datasource"] = attributesPath;
        ogr["sql"] = "select \"_ogr_geometry_\" from attributes";

        options.add("ogr", ogr);

        reader.setOptions(options);
    }

    PointTable eptTable;
    reader.prepare(eptTable);

    uint64_t eptNp(0);
    for (const PointViewPtr& view : reader.execute(eptTable))
    {
        eptNp += view->size();
    }

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", Support::datapath("autzen/autzen-attribute-cropped.las"));
        source.setOptions(options);
    }
    PointTable sourceTable;
    source.prepare(sourceTable);
    uint64_t sourceNp(0);
    for (const PointViewPtr& view : source.execute(sourceTable))
    {
        sourceNp += view->size();
    }

    EXPECT_EQ(eptNp, sourceNp);
    EXPECT_EQ(eptNp, 86u);
    EXPECT_EQ(sourceNp, 86u);
}



} // namespace pdal
