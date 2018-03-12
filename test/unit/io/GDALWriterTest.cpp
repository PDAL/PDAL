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
#include <pdal/GDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <filters/RangeFilter.hpp>
#include <io/GDALWriter.hpp>
#include <io/LasReader.hpp>
#include <io/TextReader.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>

using namespace pdal;

namespace
{

void runGdalWriter(const Options& wo, const std::string& infile,
    const std::string& outfile, const std::string& values)
{
    FileUtils::deleteFile(outfile);

    Options ro;
    ro.add("filename", infile);

    TextReader r;
    r.setOptions(ro);

    GDALWriter w;
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    using namespace gdal;

    std::istringstream iss(values);

    std::vector<double> arr;
    while (true)
    {
        double d;
        iss >> d;
        if (!iss)
            break;
        arr.push_back(d);
    }

    registerDrivers();
    Raster raster(outfile, "GTiff");
    if (raster.open() != GDALError::None)
    {
        throw pdal_error(raster.errorMsg());
    }
    std::vector<double> data;
    raster.readBand(data, 1);
    for (size_t i = 0; i < arr.size(); ++i)
        EXPECT_NEAR(arr[i], data[i], .001);
}

void runGdalWriter2(const Options& wo, const std::string& outfile,
    const std::string& values, bool stream)
{
    FileUtils::deleteFile(outfile);

    Options ro;
    ro.add("filename", Support::datapath("gdal/grid.txt"));

    Options ro2;
    ro2.add("filename", Support::datapath("gdal/grid2.txt"));

    TextReader r;
    r.setOptions(ro);

    TextReader r2;
    r2.setOptions(ro2);

    GDALWriter w;
    w.setOptions(wo);
    w.setInput(r);
    w.setInput(r2);

    if (!stream)
    {
        PointTable t;

        w.prepare(t);
        w.execute(t);
    }
    else
    {
        FixedPointTable t(10);

        w.prepare(t);
        w.execute(t);
    }

    using namespace gdal;

    std::istringstream iss(values);

    std::vector<double> arr;
    while (true)
    {
        double d;
        iss >> d;
        if (!iss)
            break;
        arr.push_back(d);
    }

    registerDrivers();
    Raster raster(outfile, "GTiff");
    if (raster.open() != GDALError::None)
    {
        throw pdal_error(raster.errorMsg());
    }
    std::vector<double> data;
    raster.readBand(data, 1);
    for (size_t i = 0; i < arr.size(); ++i)
        EXPECT_NEAR(arr[i], data[i], .001);
}

}

TEST(GDALWriterTest, min)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "min");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     8.900 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.400     6.400 "
        "2.000     3.000     4.000     4.400     5.400 "
        "1.000     2.000     3.000     4.000     5.000 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, min2)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "min");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    Options wo2 = wo;
    wo2.add("bounds", "([-2, 4.7],[-2, 6.5])");

    const std::string output =
        "-9999.000 -9999.00 -9999.00 -9999.00 -9999.00 -9999.00    -1.00"
        "-9999.000 -9999.00 -9999.00 -9999.00 -9999.00 -9999.00 -9999.00 "
        "-9999.000 -9999.00     5.00 -9999.00     7.00     8.00     8.90 "
        "-9999.000 -9999.00     4.00 -9999.00     6.00     7.00     8.00 "
        "-9999.000 -9999.00     3.00     4.00     5.00     5.40     6.40 "
        "-9999.000 -9999.00     2.00     3.00     4.00     4.40     5.40 "
        "-9999.000 -9999.00     1.00     2.00     3.00     4.00     5.00 "
        "   -1.000    -1.00 -9999.00 -9999.00 -9999.00 -9999.00 -9999.00 "
        "   -1.000    -1.00 -9999.00 -9999.00 -9999.00 -9999.00 -9999.00";

    runGdalWriter2(wo, outfile, output, false);
    runGdalWriter2(wo2, outfile, output, false);
    runGdalWriter2(wo2, outfile, output, true);
}

TEST(GDALWriterTest, minWindow)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "min");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output =
        "5.000     5.457     7.000     8.000     8.900 "
        "4.000     4.848     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.400     6.400 "
        "2.000     3.000     4.000     4.400     5.400 "
        "1.000     2.000     3.000     4.000     5.000 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, max)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "max");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     9.100 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.400     6.400 "
        "1.000     2.000     3.000     4.400     5.400 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, maxWindow)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "max");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output =
        "5.000     5.500     7.000     8.000     9.100 "
        "4.000     4.924     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.400     6.400 "
        "1.000     2.000     3.000     4.400     5.400 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, mean)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "mean");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     8.967 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.700     6.700 "
        "2.000     3.000     4.000     4.800     5.800 "
        "1.000     2.000     3.000     4.200     5.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, meanWindow)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "mean");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output =
        "5.000     5.478     7.000     8.000     8.967 "
        "4.000     4.881     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.700     6.700 "
        "2.000     3.000     4.000     4.800     5.800 "
        "1.000     2.000     3.000     4.200     5.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, idw)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "idw");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     9.000 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.000     6.000 "
        "1.000     2.000     3.000     4.000     5.000 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, idwWindow)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "idw");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output =
        "5.000     5.500     7.000     8.000     9.000 "
        "4.000     4.905     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.000     6.000 "
        "1.000     2.000     3.000     4.000     5.000 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, count)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "count");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "1.000     0.000     1.000     1.000     3.000 "
        "1.000     0.000     1.000     1.000     1.000 "
        "1.000     1.000     1.000     2.000     2.000 "
        "1.000     1.000     1.000     4.000     4.000 "
        "1.000     1.000     1.000     2.000     2.000 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, stdev)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "stdev");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output =
        "0.000 -9999.000     0.000     0.000     0.094 "
        "0.000 -9999.000     0.000     0.000     0.000 "
        "0.000     0.000     0.000     0.300     0.300 "
        "0.000     0.000     0.000     0.424     0.424 "
        "0.000     0.000     0.000     0.200     0.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, stdevWindow)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "stdev");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output =
        "0.000     0.021     0.000     0.000     0.094 "
        "0.000     0.034     0.000     0.000     0.000 "
        "0.000     0.000     0.000     0.300     0.300 "
        "0.000     0.000     0.000     0.424     0.424 "
        "0.000     0.000     0.000     0.200     0.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

TEST(GDALWriterTest, additionalDim)
{
    std::string outfile(Support::temppath("out.tif"));

    FileUtils::deleteFile(outfile);

    LasReader r;
    Options ro;

    ro.add("filename", Support::datapath("las/extrabytes.las"));

    GDALWriter w;
    Options wo;

    wo.add("dimension", "Flags1");
    wo.add("resolution", 2);
    wo.add("radius", 2.7);
    wo.add("filename", outfile);

    r.setOptions(ro);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    EXPECT_NO_THROW(w.prepare(t));

    Options wo2;

    wo2.add("dimension", "Flag55");
    wo2.add("resolution", 2);
    wo2.add("radius", 2.7);
    wo2.add("filename", outfile);

    w.setOptions(wo2);

    PointTable t2;
    EXPECT_THROW(w.prepare(t2), pdal_error);
}

TEST(GDALWriterTest, btbad)
{
    std::string infile = Support::datapath("gdal/grid.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "BT");
    wo.add("output_type", "min");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("data_type", "double");
    wo.add("filename", outfile);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     8.900 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.000     6.000 "
        "2.000     3.000     4.000     4.000     5.000 "
        "1.000     2.000     3.000     4.000     5.000 ";

    EXPECT_THROW(runGdalWriter(wo, infile, outfile, output), pdal_error);
}

TEST(GDALWriterTest, btint)
{
    std::string outfile = Support::temppath("tmp.bt");
    FileUtils::deleteFile(outfile);

    Options ro;
    ro.add("filename", Support::datapath("gdal/grid.txt"));

    TextReader r;
    r.setOptions(ro);

    GDALWriter w;
    Options wo;
    wo.add("gdaldriver", "BT");
    wo.add("output_type", "min");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("data_type", "int32");
    wo.add("filename", outfile);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    using namespace gdal;

    const std::string values =
        "5.000 -9999.000     7.000     8.000     9.000 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.000     6.000 "
        "2.000     3.000     4.000     4.000     5.000 "
        "1.000     2.000     3.000     4.000     5.000 ";
    std::istringstream iss(values);

    std::vector<double> arr;
    while (true)
    {
        double d;
        iss >> d;
        if (!iss)
            break;
        arr.push_back(d);
    }

    registerDrivers();
    Raster raster(outfile, "BT");
    if (raster.open() != GDALError::None)
    {
        throw pdal_error(raster.errorMsg());
    }
    std::vector<int32_t> data;
    raster.readBand(data, 1);
    for (size_t i = 0; i < arr.size(); ++i)
        EXPECT_NEAR(arr[i], data[i], .001);
}

TEST(GDALWriterTest, no_points)
{
    std::string outfile(Support::temppath("out.tif"));
    FileUtils::deleteFile(outfile);

    LasReader r;
    Options ro;
    ro.add("filename", Support::datapath("las/no-points.las"));
    r.setOptions(ro);

    RangeFilter f;
    f.setInput(r);
    Options fo;
    fo.add("limits", "Classification[2:2]");
    f.setOptions(fo);

    GDALWriter w;
    w.setInput(f);
    Options wo;
    wo.add("resolution", 2);
    wo.add("filename", outfile);
    w.setOptions(wo);

    PointTable t;
    w.prepare(t);
    EXPECT_THROW(w.execute(t), pdal_error);
}

// Check that we don't crash with bad X/Y's in grid_bounds.txt and a preset
// grid bounds.
TEST(GDALWriterTest, bounds)
{
    std::string infile = Support::datapath("gdal/grid_bounds.txt");
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "mean");
    wo.add("resolution", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("bounds", "([0, 4.5],[0, 4.5])");

    const std::string output =
        "5.000 -9999.000     7.000     8.000     8.967 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.700     6.700 "
        "2.000     3.000     4.000     4.800     5.800 "
        "1.000     2.000     3.000     4.200     5.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

