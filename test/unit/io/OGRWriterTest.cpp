/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/ErrorHandler.hpp>
#include <filters/HeadFilter.hpp>
#include <filters/RangeFilter.hpp>
#include <filters/SortFilter.hpp>
#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <io/OGRWriter.hpp>
#include <io/LasReader.hpp>
#include "Support.hpp"

#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <nlohmann/json.hpp>

#include <iostream>
#include <sstream>

namespace pdal
{

namespace
{

// Checks that actual holds everything in expected. Object keys that
// expected doesn't have are ignored, so it can leave out what changes
// between GDAL and PROJ versions, such as the file path and the full SRS.
// Arrays must be the same length. Numbers match within 1e-6, so float
// noise such as 849017.3200000001 doesn't count. Each difference is
// reported with its path, such as /layers/0/featureCount.
void expectSubset(const NL::ordered_json& expected,
    const NL::ordered_json& actual, const std::string& path = "")
{
    if (expected.is_number() && actual.is_number())
    {
        EXPECT_NEAR(actual.get<double>(), expected.get<double>(), 1e-6)
            << "at " << path;
    }
    else if (expected.type() != actual.type())
    {
        ADD_FAILURE() << "at " << path << ": expected " << expected.dump()
            << ", got " << actual.dump();
    }
    else if (expected.is_object())
    {
        for (auto it = expected.begin(); it != expected.end(); ++it)
        {
            std::string keyPath = path + "/" + it.key();
            if (actual.contains(it.key()))
                expectSubset(it.value(), actual[it.key()], keyPath);
            else
                ADD_FAILURE() << keyPath << " is missing";
        }
    }
    else if (expected.is_array())
    {
        if (expected.size() != actual.size())
        {
            ADD_FAILURE() << "at " << path << ": expected "
                << expected.size() << " elements, got " << actual.size();
            return;
        }
        for (size_t i = 0; i < expected.size(); ++i)
            expectSubset(expected[i], actual[i],
                path + "/" + std::to_string(i));
    }
    else if (expected != actual)
    {
        ADD_FAILURE() << "at " << path << ": expected " << expected.dump()
            << ", got " << actual.dump();
    }
}

void runOgrWriterInfo(const Options& wo, const std::string& infile,
    const std::string& infofile, const std::string suffix, int featureCount = 10)
{
    FileUtils::createDirectory(Support::temppath("ogr"));
    std::string outfile(Support::temppath(std::string("ogr/") + FileUtils::stem(FileUtils::stem(FileUtils::getFilename(infofile))) + suffix));
    std::string outinfofile(outfile + ".json");

    if(FileUtils::isDirectory(outfile)) {
        FileUtils::deleteDirectory(outfile);
    } else {
        FileUtils::deleteFile(outfile);
    }
    FileUtils::deleteFile(outinfofile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    LasReader reader;
    reader.setOptions(readerOpts);

    Options sortOpts;
    SortFilter sortFilter;
    sortOpts.add("dimension", "X");
    sortFilter.setOptions(sortOpts);
    sortFilter.setInput(reader);

    Options writerOpts;
    writerOpts.add("filename", outfile);

    OGRWriter writer;
    writer.setOptions(wo);
    writer.addOptions(writerOpts);

    HeadFilter headFilter;
    if (featureCount > 0) {
        Options headOpts;
        headOpts.add("count", featureCount);
        headFilter.setOptions(headOpts);
        headFilter.setInput(sortFilter);
        writer.setInput(headFilter);
    } else {
        writer.setInput(sortFilter);
    }

    PointTable t;
    writer.prepare(t);
    writer.execute(t);

    std::string cmd = "ogrinfo -json -nomd " + outfile;
    if (featureCount != 0) {
        cmd += " -features";
    }
    // Only stdout is captured, so GDAL's warnings can't get into the JSON
    std::string info;
    ASSERT_EQ(Utils::run_shell_command(cmd, info), 0)
        << "error running " << cmd;

    auto handle = FileUtils::createFile(outinfofile);
    *handle << info;
    FileUtils::closeFile(handle);

    SCOPED_TRACE(infofile + " <-> " + outinfofile);
    ASSERT_TRUE(FileUtils::fileExists(infofile));
    expectSubset(
        NL::ordered_json::parse(FileUtils::readFileIntoString(infofile)),
        NL::ordered_json::parse(info));
}

// ogrinfo's JSON gives points as GeoJSON, which has no M, so this checks
// the M of each point in filename's layer through GDAL
void expectMeasures(const std::string& filename,
    const std::vector<double>& expected)
{
    GDALDatasetUniquePtr ds(GDALDataset::Open(filename.c_str(),
        GDAL_OF_VECTOR | GDAL_OF_READONLY));
    ASSERT_TRUE(ds != nullptr) << "can't open " << filename;
    ASSERT_EQ(ds->GetLayerCount(), 1);

    std::vector<double> measures;
    for (auto& feat : *ds->GetLayer(0))
    {
        const OGRGeometry *geom = feat->GetGeometryRef();
        ASSERT_TRUE(geom && geom->IsMeasured() &&
            wkbFlatten(geom->getGeometryType()) == wkbPoint)
            << "feature " << feat->GetFID() << " isn't a measured point";
        measures.push_back(geom->toPoint()->getM());
    }
    EXPECT_EQ(measures, expected);
}

}

TEST(OGRWriterTest, shapefile)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/shapefile.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, json)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/json.geojson.json");

    Options wo;
    wo.add("ogrdriver", "GeoJSON");

    runOgrWriterInfo(wo, infile, infofile, ".geojson");
}

TEST(OGRWriterTest, geopackage)
{
    std::string infile = Support::datapath("las/utm15.las");
    std::string infofile = Support::datapath("ogr/geopackage.gpkg.json");

    Options wo;
    wo.add("ogrdriver", "GPKG");

    runOgrWriterInfo(wo, infile, infofile, ".gpkg");
}

TEST(OGRWriterTest, creation_options)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/creation_options.geojson.json");

        Options wo;
        wo.add("ogrdriver", "GeoJSON");
        wo.add("ogr_options", "WRITE_BBOX=YES");
        wo.add("ogr_options", "COORDINATE_PRECISION=1");

        runOgrWriterInfo(wo, infile, infofile, ".geojson");
}

TEST(OGRWriterTest, shapefile_measure)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/shapefile_measure.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("measure_dim", "Classification");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
    expectMeasures(Support::temppath("ogr/shapefile_measure.shp"),
        { 1, 1, 2, 1, 1, 1, 2, 2, 1, 1 });
}

TEST(OGRWriterTest, attrs_all)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/attrs_all.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("attr_dims", "all");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, geopackage_attrs_all)
{
    std::string infile = Support::datapath("las/simple.las");

    #if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,9,0)
        std::string infofile = Support::datapath("ogr/geopackage_attrs_all_gdal_gt_390.gpkg.json");
    #else
        std::string infofile = Support::datapath("ogr/geopackage_attrs_all.gpkg.json");
    #endif


    Options wo;
    wo.add("ogrdriver", "GPKG");
    wo.add("attr_dims", "all");

    runOgrWriterInfo(wo, infile, infofile, ".gpkg");
}

TEST(OGRWriterTest, attrs)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/attrs.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("attr_dims", "Classification,Red,Green,Blue");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, attrs_measure)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/attrs_measure.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("measure_dim", "Classification");
    wo.add("attr_dims", "all");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
    expectMeasures(Support::temppath("ogr/attrs_measure.shp"),
        { 1, 1, 2, 1, 1, 1, 2, 2, 1, 1 });
}

TEST(OGRWriterTest, multicount)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/multicount.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("multicount", "3");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, multicount_2)
{
    // feature count < multicount
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/multicount_2.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("multicount", "20");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, multicount_3)
{
    // feature count == exact multiple of multicount
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/multicount_3.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("multicount", "5");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
}

TEST(OGRWriterTest, error_multicount_attrs)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/error");

    Options wo;
    wo.add("ogrdriver", "GeoJSON");
    wo.add("multicount", "3");
    wo.add("attr_dims", "all");

    try
    {
        runOgrWriterInfo(wo, infile, infofile, ".geojson");
        FAIL() << "Expected an exception for an invalid arg combination";
    }
    catch (pdal_error const & err)
    {
        EXPECT_EQ("writers.ogr: multicount > 1 incompatible with attr_dims",
            std::string(err.what()));
    }
}

TEST(OGRWriterTest, error_unknown_attr)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/error");

    Options wo;
    wo.add("ogrdriver", "GeoJSON");
    wo.add("attr_dims", "bananas");

    try
    {
        runOgrWriterInfo(wo, infile, infofile, ".geojson");
        FAIL() << "Expected an exception for an invalid arg combination";
    }
    catch (pdal_error const & err)
    {
        EXPECT_EQ("writers.ogr: Dimension 'bananas' (attr_dims) not found.",
            std::string(err.what()));
    }
}

TEST(OGRWriterTest, error_ogr)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/error");

    Options wo;
    wo.add("ogrdriver", "GeoJSON");

    // RFC7946 GeoJSON doesn't allow non-WGS84 CRS, and will throw an error
    wo.add("ogr_options", "RFC7946=YES");

    try
    {
        {
            gdal::ErrorHandlerSuspender devnull;

            runOgrWriterInfo(wo, infile, infofile, ".geojson");
        }
        FAIL() << "Expected an exception from OGR";
    }
    catch (pdal_error const & err)
    {
        EXPECT_EQ("writers.ogr: Can't create OGR layer: Failed to create coordinate transformation between the input coordinate system and WGS84.",
            std::string(err.what()));
    }
}

} // namespace pdal
