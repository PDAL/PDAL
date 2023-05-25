/******************************************************************************
 * Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <io/CopcReader.hpp>
#include <io/LasReader.hpp>
#include <filters/CropFilter.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <filters/SortFilter.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "Support.hpp"

namespace pdal
{

namespace
{
    const std::string copcPath(Support::datapath("copc/lone-star.copc.laz"));
    const std::string copcAutzenPath(Support::datapath("copc/1.2-with-color.copc.laz"));
    const BOX3D pointBounds(515368.60225, 4918340.364, 2322.89625,
        515401.043, 4918381.12375, 2338.5755);
    const point_count_t numPoints(518862);
}

/**
namespace
{
    const BOX3D expBoundsConforming(515368, 4918340, 2322,
            515402, 4918382, 2339);
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
    const std::string eptAutzenPath(
            Support::datapath("ept/1.2-with-color/ept.json"));
    const std::string attributesPath(
            Support::datapath("autzen/attributes.json"));

    // Also test a basic read of binary/zstandard versions of a smaller dataset.
    const std::string ellipsoidEptBinaryPath(
            Support::datapath("ept/ellipsoid-binary/ept.json"));
    const std::string ellipsoidEptZstandardPath(
            Support::datapath("ept/ellipsoid-zstandard/ept.json"));

    const point_count_t ellipsoidNumPoints(100000);
    const BOX3D ellipsoidBoundsConforming(-8242746, 4966506, -50,
            -8242446, 4966706, 50);
}

TEST(EptReaderTest, protocol)
{
    Options opts;
    opts.add("filename", "ept://http://testfile");

    EptReader reader;
    reader.setOptions(opts);

    bool gotEx = false;
    try
    {
        reader.preview();
    }
    catch (const pdal_error& err)
    {
        EXPECT_TRUE(strstr(err.what(), "ept.json"));
        gotEx = true;
    }
    EXPECT_TRUE(gotEx);
}
**/

TEST(CopcReaderTest, inspect)
{
    const std::vector<std::string> dimNames = {
         "ClassFlags", "Classification", "EdgeOfFlightLine", "GpsTime", "Intensity",
         "NumberOfReturns", "PointSourceId", "ReturnNumber", "ScanAngleRank", "ScanChannel",
         "ScanDirectionFlag", "UserData", "X", "Y", "Z"
    };

    Options options;
    options.add("filename", copcPath);

    CopcReader reader;
    reader.setOptions(options);

    const QuickInfo qi(reader.preview());

    EXPECT_TRUE(qi.valid());
    EXPECT_EQ(qi.m_bounds, pointBounds);
    EXPECT_EQ(qi.m_pointCount, numPoints);
    StringList d(qi.m_dimNames.begin(), qi.m_dimNames.end());
    std::sort(d.begin(), d.end());
    EXPECT_TRUE(std::equal(d.cbegin(), d.cend(), dimNames.cbegin()));

    std::string srs = qi.m_srs.getWKT();
    // Sometimes we get back "metre" when we're execting "meter".
    while (true)
    {
        auto pos = srs.find("metre");
        if (pos == std::string::npos)
            break;
        srs.replace(pos, 5, "meter");
    }

    const std::string wkt = R"(GEOCCS["unnamed",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0],UNIT["meter",1],AXIS["Geocentric X",OTHER],AXIS["Geocentric Y",OTHER],AXIS["Geocentric Z",NORTH]])";
    EXPECT_EQ(wkt, srs);
}

TEST(CopcReaderTest, fullRead)
{
    Options options;
    options.add("filename", copcPath);

    PointTable table;

    CopcReader reader;
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
            ASSERT_TRUE(pointBounds.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, numPoints);
}

TEST(CopcReaderTest, resolutionLimit)
{
    Options options;
    options.add("filename", copcPath);

    // Resolution for the first
    //      Depth 0: 0.31846
    //      Depth 1: 0.15923
    //      Depth 2: 0.079615
    //
    // Any resolution option between 0.31846 and 0.15923 will select depths 0 and 1,
    // so we'll test a corresponding query.
    options.add("resolution", 0.2);

    // This expected value corresponds to the sum of the point counts of all
    // files in our dataset whose depth is less than 2.  This value is summed
    // from the hierarchy for depths 0 through 1.
    const point_count_t expectedCount = 163993;

    PointTable table;

    CopcReader reader;
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
            ASSERT_TRUE(pointBounds.contains(x, y, z));
            ASSERT_TRUE(o < 4);
        }
    }

    EXPECT_EQ(np, expectedCount);
}


TEST(CopcReaderTest, boundedRead2d)
{
    BOX2D bounds(515380, 4918350, 515400, 4918370);

    // First we'll query the EptReader for these bounds.
    CopcReader reader;
    {
        Options options;
        options.add("filename", copcPath);
        options.add("bounds", bounds);
        reader.setOptions(options);
    }
    PointTable copcTable;
    reader.prepare(copcTable);
    const auto set(reader.execute(copcTable));

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
        options.add("filename", copcPath);
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
    PointViewSet s = crop.execute(sourceTable);
    EXPECT_EQ(s.size(), 1u);
    PointViewPtr v = *s.begin();

    EXPECT_EQ(np, v->size());
    EXPECT_EQ(np, 354211u);
}

TEST(CopcReaderTest, boundedRead3d)
{
    BOX3D bounds(515380, 4918350, 2320, 515400, 4918370, 2325);

    // First we'll query the EptReader for these bounds.
    CopcReader reader;
    {
        Options options;
        options.add("filename", copcPath);
        options.add("bounds", bounds);
        reader.setOptions(options);
    }
    PointTable copcTable;
    reader.prepare(copcTable);
    const auto set(reader.execute(copcTable));

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
        options.add("filename", copcPath);
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

TEST(CopcReaderTest, stream)
{
    Options ops;
    ops.add("filename", copcPath);
    ops.add("resolution", 1);

    // Execute the reader in normal non-streaming mode.
    CopcReader normalReader;
    normalReader.setOptions(ops);
    PointTable normalTable;
    normalReader.prepare(normalTable);
    PointViewSet s = normalReader.execute(normalTable);
    PointView& normalView = *(*s.begin());

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
    CopcReader streamReader;
    streamReader.setOptions(ops);
    std::vector<char> streamBuffer;
    PointTable streamTable;
    PointView streamView(streamTable);
    TestPointTable testTable(streamView);

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

    const auto cmp = [](const PointRef& a, const PointRef& b)
    {
        return (a.compare(Dimension::Id::GpsTime, b));
    };
    normalView.stableSort(cmp);
    streamView.stableSort(cmp);

    for (PointId i(0); i < normalView.size(); ++i)
        for (const auto& dim : normalTable.layout()->dims())
        {
            double nval = normalView.getFieldAs<double>(dim, i);
            double sval = streamView.getFieldAs<double>(dim, i);
            ASSERT_EQ(nval, sval) <<
                "Point ID: " << i << " dim: " << normalView.layout()->dimName(dim) <<
                " don't match. Values normal/stream = " << nval << "/" << sval << ".";
        }
}

TEST(EptReaderTest, boundedCrop)
{
    std::string wkt = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection.wkt"));

    // First we'll query the EptReader for these bounds.
    CopcReader reader;
    {
        Options options;
        options.add("filename", copcAutzenPath);
        Option polygon("polygon", wkt + "/ EPSG:2991");
        options.add(polygon);
        reader.setOptions(options);
    }

    PointTable copcTable;
    reader.prepare(copcTable);
    PointViewSet s = reader.execute(copcTable);
    PointViewPtr v = *s.begin();

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
        Options options;
        Option polygon("polygon", wkt + "/ EPSG:2991");
        options.add(polygon);
        crop.setOptions(options);
        crop.setInput(source);
    }
    PointTable sourceTable;
    crop.prepare(sourceTable);
    PointViewSet s2 = crop.execute(sourceTable);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v->size(), v2->size());
    EXPECT_EQ(v->size(), 47u);
    EXPECT_EQ(v2->size(), 47u);
}

TEST(CopcReaderTest, polygonAndBoundsCrop)
{
    std::string wkt = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection.wkt"));

    // This box is approximately the bounding box of the WKT above, with the
    // eastmost 25% of the bounds omitted.  So this should shrink our query
    // results from the "boundedCrop" test above since we are further limiting
    // our spatial selection.
    std::string boxstring = "([636577.1, 637297.4225], [850571.42, 851489.35])";
    BOX2D box;
    Utils::fromString(boxstring, box);

    // First we'll query the CopcReader for these bounds.
    CopcReader reader;
    {
        Options options;
        options.add("filename", copcAutzenPath);
        options.add("polygon", wkt + "/ EPSG:2991");
        options.add("bounds", boxstring);
        reader.setOptions(options);
    }
    SortFilter sortCopc;
    {
        Options options;
        options.add("dimension", "GpsTime");
        sortCopc.setOptions(options);
        sortCopc.setInput(reader);
    }

    PointTable copcTable;
    sortCopc.prepare(copcTable);
    PointViewSet s = sortCopc.execute(copcTable);
    PointViewPtr v = *s.begin();

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", copcAutzenPath);
        source.setOptions(options);
    }
    CropFilter boundsCrop;
    {
        Options options;
        options.add("bounds", boxstring);
        boundsCrop.setOptions(options);
        boundsCrop.setInput(source);
    }
    CropFilter polygonCrop;
    {
        Options options;
        Option polygon("polygon", wkt + "/ EPSG:2991");
        options.add(polygon);
        polygonCrop.setOptions(options);
        polygonCrop.setInput(boundsCrop);
    }

    PointTable sourceTable;
    polygonCrop.prepare(sourceTable);
    PointViewSet s2 = polygonCrop.execute(sourceTable);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v->size(), v2->size());
    EXPECT_EQ(v->size(), 38u);
}


TEST(CopcReaderTest, boundedCropReprojection)
{
    std::string selection = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection.wkt"));
    std::string selection4326 = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-selection-dd.wkt"));
    std::string srs = FileUtils::readFileIntoString(
        Support::datapath("autzen/autzen-srs.wkt"));

    CopcReader reader;
    {
        Options options;
        options.add("filename", copcAutzenPath);
        options.add("override_srs", srs);
        options.add("polygon", selection4326 + "/EPSG:4326");
        reader.setOptions(options);
    }

    PointTable copcTable;
    reader.prepare(copcTable);
    PointViewSet s = reader.execute(copcTable);
    PointViewPtr v = *s.begin();

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("filename", copcAutzenPath);
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
    PointViewSet s2 = crop.execute(sourceTable);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v->size(), v2->size());
    // F'in proj keeps changing, making these numbers dance around.
    EXPECT_GE(v->size(), 40u);
    EXPECT_LE(v->size(), 50u);
}


TEST(CopcReaderTest, ogrCrop)
{
    const std::string srs = R"(PROJCS["NAD_1983_HARN_Lambert_Conformal_Conic",GEOGCS["GCS_North_American_1983_HARN",DATUM["NAD83_High_Accuracy_Reference_Network",SPHEROID["GRS 1980",6378137,298.2572221010002,AUTHORITY["EPSG","7019"]],AUTHORITY["EPSG","6152"]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]],PROJECTION["Lambert_Conformal_Conic_2SP"],PARAMETER["standard_parallel_1",43],PARAMETER["standard_parallel_2",45.5],PARAMETER["latitude_of_origin",41.75],PARAMETER["central_meridian",-120.5],PARAMETER["false_easting",1312335.958005249],PARAMETER["false_northing",0],UNIT["foot",0.3048,AUTHORITY["EPSG","9002"]]])";

    NL::json ogr;
    ogr["drivers"] = {"GeoJSON"};
    ogr["datasource"] = Support::datapath("autzen/attributes.json");
    ogr["sql"] = "select \"_ogr_geometry_\" from attributes";

    CopcReader reader;
    {
        Options options;
        options.add("override_srs", srs);
        options.add("filename", copcAutzenPath);
        options.add("ogr", ogr);

        reader.setOptions(options);
    }

    PointTable copcTable;
    reader.prepare(copcTable);
    PointViewSet s = reader.execute(copcTable);
    PointViewPtr v = *s.begin();

    // Now we'll check the result against a crop filter of the source file with
    // the same bounds.
    LasReader source;
    {
        Options options;
        options.add("override_srs", srs);
        options.add("filename", copcAutzenPath);
        source.setOptions(options);
    }

    std::vector<Polygon> polys = gdal::getPolygons(ogr);
    for (Polygon& p : polys)
        p.transform(srs);
    PointTable sourceTable;
    source.prepare(sourceTable);
    PointViewSet s2 = source.execute(sourceTable);
    PointViewPtr vTemp = *s2.begin();
    PointViewPtr v2 = vTemp->makeNew();
    for (PointId idx = 0; idx < vTemp->size(); ++idx)
    {
        double x = vTemp->getFieldAs<double>(Dimension::Id::X, idx);
        double y = vTemp->getFieldAs<double>(Dimension::Id::Y, idx);
        for (Polygon& poly : polys)
            if (poly.contains(x, y))
            {
                v2->appendPoint(*vTemp, idx);
                break;
            }
    }

    EXPECT_EQ(v->size(), v2->size());
    // F'in proj keeps changing, making these numbers dance around.
    EXPECT_GE(v->size(), 80u);
    EXPECT_LE(v->size(), 90u);
}

} // namespace pdal
