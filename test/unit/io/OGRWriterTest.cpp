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

#include <cmath>
#include <iostream>
#include <sstream>

// These tests write points with writers.ogr, then open the result with GDAL
// and write what it holds as JSON: the layer's geometry type, feature
// count, extent, SRS and fields, and each feature's values and points.
// That's compared with the expected JSON in test/data/ogr.

namespace pdal
{

namespace
{

// Rounds to 6 decimal places, so the last bits of a double, which can
// differ between drivers, don't change the JSON text
double rounded(double v)
{
    return std::round(v * 1e6) / 1e6;
}

// The OGC name of a geometry type, such as PointZM
const char *geomTypeName(OGRwkbGeometryType type)
{
    return OGRToOGCGeomType(type, /*bCamelCase=*/true, /*bAddZM=*/true);
}

// The layer in filename as JSON, with its features unless summaryOnly
std::string layerInfo(const std::string& filename, bool summaryOnly)
{
    GDALDatasetUniquePtr ds(GDALDataset::Open(filename.c_str(),
        GDAL_OF_VECTOR | GDAL_OF_READONLY));
    if (!ds || ds->GetLayerCount() != 1)
    {
        ADD_FAILURE() << "can't read one layer from " << filename;
        return "";
    }
    OGRLayer *layer = ds->GetLayer(0);
    OGRFeatureDefn *defn = layer->GetLayerDefn();

    NL::ordered_json info;
    info["name"] = layer->GetName();
    info["geometryType"] = geomTypeName(layer->GetGeomType());
    info["featureCount"] = layer->GetFeatureCount();
    OGREnvelope env;
    if (layer->GetExtent(&env) == OGRERR_NONE)
        info["extent"] = { rounded(env.MinX), rounded(env.MinY),
            rounded(env.MaxX), rounded(env.MaxY) };
    // Only the SRS name, as the rest changes with the PROJ version
    const OGRSpatialReference *srs = layer->GetSpatialRef();
    if (srs)
        info["srs"] = srs->GetName();
    else
        info["srs"] = nullptr;

    info["fields"] = NL::ordered_json::array();
    for (int i = 0; i < defn->GetFieldCount(); ++i)
    {
        const OGRFieldDefn *fd = defn->GetFieldDefn(i);
        info["fields"].push_back({
            { "name", fd->GetNameRef() },
            { "type", OGRFieldDefn::GetFieldTypeName(fd->GetType()) },
            { "width", fd->GetWidth() },
            { "precision", fd->GetPrecision() } });
    }
    if (summaryOnly)
        return info.dump(2) + "\n";

    info["features"] = NL::ordered_json::array();
    for (auto& feat : *layer)
    {
        NL::ordered_json f;
        f["fid"] = feat->GetFID();
        f["properties"] = NL::ordered_json::object();
        for (int i = 0; i < defn->GetFieldCount(); ++i)
        {
            const char *name = defn->GetFieldDefn(i)->GetNameRef();
            switch (defn->GetFieldDefn(i)->GetType())
            {
            case OFTInteger:
            case OFTInteger64:
                f["properties"][name] = feat->GetFieldAsInteger64(i);
                break;
            case OFTReal:
                f["properties"][name] = rounded(feat->GetFieldAsDouble(i));
                break;
            default:
                f["properties"][name] = feat->GetFieldAsString(i);
            }
        }

        // Points as [x, y, z] or [x, y, z, m]
        const OGRGeometry *geom = feat->GetGeometryRef();
        if (!geom)
        {
            ADD_FAILURE() << "feature " << feat->GetFID() << " of "
                << filename << " has no geometry";
            continue;
        }
        f["geometryType"] = geomTypeName(geom->getGeometryType());
        f["points"] = NL::ordered_json::array();
        auto addPoint = [&f](const OGRPoint *p)
        {
            NL::ordered_json c = { rounded(p->getX()), rounded(p->getY()) };
            if (p->Is3D())
                c.push_back(rounded(p->getZ()));
            if (p->IsMeasured())
                c.push_back(rounded(p->getM()));
            f["points"].push_back(c);
        };
        if (wkbFlatten(geom->getGeometryType()) == wkbMultiPoint)
            for (const OGRPoint *p : *geom->toMultiPoint())
                addPoint(p);
        else
            addPoint(geom->toPoint());
        info["features"].push_back(f);
    }
    return info.dump(2) + "\n";
}

void runOgrWriterInfo(const Options& wo, const std::string& infile,
    const std::string& infofile, const std::string suffix, int featureCount = 10,
    uint32_t(*compare)(const std::string&, const std::string&, int32_t) = &Support::diff_text_files
)
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

    std::string info = layerInfo(outfile, featureCount == 0);

    auto handle = FileUtils::createFile(outinfofile);
    *handle << info;
    FileUtils::closeFile(handle);

    EXPECT_EQ(compare(infofile, outinfofile, -1), 0u) \
        << "  (" << infofile << " <-> " << outinfofile << ")";
}

uint32_t diff_geojson(const std::string& file1, const std::string& file2, int32_t ignoreLine1=-1)
{
    // GeoJSON files depend on PROJ version, later versions write a much different
    // SRS format. Crudely ignore lines between ^GEOGCRS[ and ^OGRFeature

    if (!Utils::fileExists(file1) || !Utils::fileExists(file2))
        return (std::numeric_limits<uint32_t>::max)();
    if (!Utils::fileExists(file1) || !Utils::fileExists(file2))
        return (std::numeric_limits<uint32_t>::max)();

    std::istream *istr1 = Utils::openFile(file1, false);
    std::istream *istr2 = Utils::openFile(file2, false);
    std::istream& str1 = *istr1;
    std::istream& str2 = *istr2;

    uint32_t numdiffs = 0;
    int32_t currLine = 1;
    bool buf1ignore = false;
    bool buf2ignore = false;
    std::string buf1;
    std::string buf2;

    while (!str1.eof() && !str2.eof())
    {
        if (currLine == ignoreLine1)
        {
            std::getline(str1, buf1);
            std::getline(str2, buf2);
            ++currLine;
            continue;
        }

        if (!(buf1ignore ^ buf2ignore))
        {
            std::getline(str1, buf1);
            std::getline(str2, buf2);
        }
        else if (!buf2ignore)
            std::getline(str1, buf1);
        else if (!buf1ignore)
            std::getline(str2, buf2);

        // EOF handling won't work properly if we're in an ignored block. NBD
        if (str1.eof() && str2.eof())
        {
            // hit end on both together
            break;
        }
        else if (str1.eof() && !str2.eof())
        {
            // str1 ended, but str2 still going
            while (!str2.eof())
            {
                std::getline(str2, buf2);
                ++numdiffs;
            }
            break;
        }
        else if (!str1.eof() && str2.eof())
        {
            // str2 ended, but str1 still going
            while (!str1.eof())
            {
                std::getline(str1, buf1);
                ++numdiffs;
            }
            break;
        }

        if (buf1ignore && buf1.rfind("OGRFeature", 0) == 0)
            buf1ignore = false;
        else if (buf1.rfind("GEOGCRS[", 0) == 0)
            buf1ignore = true;

        if (buf2ignore && buf2.rfind("OGRFeature", 0) == 0)
            buf2ignore = false;
        else if (buf2.rfind("GEOGCRS[", 0) == 0)
            buf2ignore = true;

        // remove line endings from test comparisons
        buf1.erase(std::remove(buf1.begin(), buf1.end(), '\r' ), buf1.end());
        buf1.erase(std::remove(buf1.begin(), buf1.end(), '\n' ), buf1.end());
        buf2.erase(std::remove(buf2.begin(), buf2.end(), '\r' ), buf2.end());
        buf2.erase(std::remove(buf2.begin(), buf2.end(), '\n' ), buf2.end());

        if (!buf1ignore && !buf2ignore && buf1 != buf2)
            ++numdiffs;

        ++currLine;
    }

    assert(str1.eof());
    assert(str2.eof());

    Utils::closeFile(istr1);
    Utils::closeFile(istr2);

    return numdiffs;
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

    runOgrWriterInfo(wo, infile, infofile, ".geojson", 10, &diff_geojson);
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
    #if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,9,0)
        std::string infofile = Support::datapath("ogr/creation_options_gdal_gt_390.geojson.json");
    #else
        std::string infofile = Support::datapath("ogr/creation_options.geojson.json");
    #endif

        Options wo;
        wo.add("ogrdriver", "GeoJSON");
        wo.add("ogr_options", "WRITE_BBOX=YES");
        wo.add("ogr_options", "COORDINATE_PRECISION=1");

        runOgrWriterInfo(wo, infile, infofile, ".geojson", 10, &diff_geojson);
}

TEST(OGRWriterTest, shapefile_measure)
{
    std::string infile = Support::datapath("las/simple.las");
    std::string infofile = Support::datapath("ogr/shapefile_measure.shp.json");

    Options wo;
    wo.add("ogrdriver", "ESRI Shapefile");
    wo.add("measure_dim", "Classification");

    runOgrWriterInfo(wo, infile, infofile, ".shp");
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
