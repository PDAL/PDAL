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
#include <pdal/private/gdal/Raster.hpp>
#include <filters/RangeFilter.hpp>
#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <io/GDALWriter.hpp>
#include <io/LasReader.hpp>
#include <io/TextReader.hpp>
#include <io/private/GDALGrid.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>

namespace pdal
{

namespace
{

void runGdalWriter(const Options& wo, const std::string& infile,
    const std::string& outfile, const std::string& values)
{
    auto run = [=](bool streamMode)
    {
        FileUtils::deleteFile(outfile);

        Options ro;
        ro.add("filename", infile);

        TextReader r;
        r.setOptions(ro);

        GDALWriter w;
        w.setOptions(wo);
        w.setInput(r);

        if (streamMode)
        {
            FixedPointTable t(100);

            w.prepare(t);
            w.execute(t);
        }
        else
        {
            PointTable t;

            w.prepare(t);
            w.execute(t);
        }

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

        gdal::Raster raster(outfile, "GTiff");
        if (raster.open() != gdal::GDALError::None)
        {
            throw pdal_error(raster.errorMsg());
        }
        std::vector<double> data;
        raster.readBand(data, 1);
        int row = 0;
        int col = 0;

        for (size_t i = 0; i < arr.size(); ++i)
        {
            EXPECT_NEAR(arr[i], data[i], .001) << "Error row/col = " <<
                row << "/" << col << std::endl;
            if (++col == raster.width())
            {
                col = 0;
                row++;
            }
        }
    };

    run(false);
    run(true);
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

    gdal::Raster raster(outfile, "GTiff");
    if (raster.open() != gdal::GDALError::None)
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
//    wo2.add("bounds", "([-2, 4.7],[-2, 6.5])");
    wo2.add("origin_x", -2);
    wo2.add("origin_y", -2);
    wo2.add("width", 7);
    wo2.add("height", 9);

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
        "2.000     3.000     4.400     5.400     6.400 "
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
        "4.000     4.942     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.400     5.400     6.400 "
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
        "2.000     3.000     4.200     4.920     5.800 "
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
        "4.000     4.896     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.700     6.700 "
        "2.000     3.000     4.200     4.920     5.800 "
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
        "1.000     1.000     2.000     5.000     4.000 "
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
        "0.000     0.000     0.200     0.449     0.424 "
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
        "0.000     0.045     0.000     0.000     0.000 "
        "0.000     0.000     0.000     0.300     0.300 "
        "0.000     0.000     0.200     0.449     0.424 "
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

    gdal::Raster raster(outfile, "BT");
    if (raster.open() != gdal::GDALError::None)
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
//    wo.add("bounds", "([0, 4.5],[0, 4.5])");
    wo.add("origin_x", 0);
    wo.add("origin_y", 0);
    wo.add("height", 5);
    wo.add("width", 5);

    const std::string output =
        "5.000 -9999.000     7.000     8.000     8.967 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.700     6.700 "
        "2.000     3.000     4.200     4.920     5.800 "
        "1.000     2.000     3.000     4.200     5.200 ";

    runGdalWriter(wo, infile, outfile, output);
}

// Make sure we reset bounds when starting a new file.
TEST(GDALWriterTest, issue_2074)
{
    FileUtils::deleteFile(Support::temppath("gdal.tif"));
    FileUtils::deleteFile(Support::temppath("gdal1.tif"));
    FileUtils::deleteFile(Support::temppath("gdal2.tif"));

    PointTable t;
    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v1(new PointView(t));
    PointViewPtr v2(new PointView(t));

    v1->setField(Dimension::Id::X, 0, 0);
    v1->setField(Dimension::Id::Y, 0, 0);
    v1->setField(Dimension::Id::Z, 0, 0);
    v1->setField(Dimension::Id::X, 1, 1);
    v1->setField(Dimension::Id::Y, 1, 1);
    v1->setField(Dimension::Id::Z, 1, 0);

    v2->setField(Dimension::Id::X, 0, 2);
    v2->setField(Dimension::Id::Y, 0, 2);
    v2->setField(Dimension::Id::Z, 0, 0);
    v2->setField(Dimension::Id::X, 1, 3);
    v2->setField(Dimension::Id::Y, 1, 3);
    v2->setField(Dimension::Id::Z, 1, 0);

    BufferReader r;
    r.addView(v1);
    r.addView(v2);

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "mean");
    wo.add("filename", Support::temppath("gdal#.tif"));
    wo.add("resolution", .5);

    GDALWriter w;

    w.setOptions(wo);
    w.setInput(r);

    w.prepare(t);
    w.execute(t);

    gdal::Raster raster(Support::temppath("gdal1.tif"), "GTiff");
    if (raster.open() != gdal::GDALError::None)
    {
        throw pdal_error(raster.errorMsg());
    }

    gdal::Raster raster2(Support::temppath("gdal2.tif"), "GTiff");
    if (raster2.open() != gdal::GDALError::None)
    {
        throw pdal_error(raster2.errorMsg());
    }
    // Make sure the output files are the same size.
    EXPECT_EQ(raster.width(), 3);
    EXPECT_EQ(raster.height(), 3);
    EXPECT_EQ(raster.width(), raster2.width());
    EXPECT_EQ(raster.height(), raster2.height());

    // Make sure that we accumulate data for two views to one output raster.
    Options wo2;
    wo2.add("gdaldriver", "GTiff");
    wo2.add("output_type", "mean");
    wo2.add("filename", Support::temppath("gdal.tif"));
    wo2.add("resolution", .5);

    GDALWriter w2;
    w2.setOptions(wo2);
    w2.setInput(r);

    w2.prepare(t);
    w2.execute(t);

    gdal::Raster raster3(Support::temppath("gdal.tif"), "GTiff");
    if (raster3.open() != gdal::GDALError::None)
    {
        throw pdal_error(raster3.errorMsg());
    }
    EXPECT_EQ(raster3.width(), 7);
    EXPECT_EQ(raster3.height(), 7);
}

// If the radius is sufficiently large, make sure the grid is filled.
TEST(GDALWriterTest, issue_2545)
{
    TextReader r;
    Options rOpts;

    rOpts.add("filename", Support::datapath("gdal/issue_2545.txt"));
    r.setOptions(rOpts);

    std::string outfile(Support::temppath("gdal.tif"));

    GDALWriter w;
    Options wOpts;
    wOpts.add("resolution", 1);
    wOpts.add("radius", 10);
    wOpts.add("output_type", "idw");
    wOpts.add("gdaldriver", "GTiff");
    wOpts.add("origin_x", .5);
    wOpts.add("origin_y", .5);
    wOpts.add("width", 7);
    wOpts.add("height", 7);
//    wOpts.add("bounds", BOX2D(.5, .5, 6.5, 6.5));
    wOpts.add("filename", outfile);

    w.setOptions(wOpts);
    w.setInput(r);

    PointTable t;
    w.prepare(t);
    w.execute(t);

    gdal::Raster raster(outfile, "GTiff");
    if (raster.open() != gdal::GDALError::None)
        throw pdal_error(raster.errorMsg());

    EXPECT_EQ(raster.width(), 7);
    EXPECT_EQ(raster.height(), 7);

    auto index = [](size_t row, size_t col)
    {
        return (row * 7) + col;
    };

    std::vector<double> data;
    raster.readBand(data, 1);
    EXPECT_EQ(data[index(1, 0)], 3.0);
    EXPECT_EQ(data[index(2, 4)], 0.);
    EXPECT_EQ(data[index(5, 5)], 10.0);
    EXPECT_EQ(data[index(6, 0)], 5.0);
    for (size_t i = 0; i < data.size(); ++i)
        EXPECT_TRUE(data[i] <= 10.0 && data[i] >= 0);
}

// Test that using the alternate grid specification works
TEST(GDALWriterTest, alternate_grid)
{
    FauxReader r;
    Options opts;
    opts.add("count", 1000);
    opts.add("mode", "constant");
    r.setOptions(opts);

    std::string outfile(Support::temppath("test.tif"));

    {
        Options opts;
        opts.add("origin_x", "10.0");
        opts.add("filename", outfile);
        opts.add("resolution", 1.1);

        PointTable t;
        GDALWriter w;
        w.setOptions(opts);
        w.setInput(r);
        EXPECT_THROW(w.prepare(t), pdal_error);
    }

    {
        Options opts;
        opts.add("origin_x", "10.0");
        opts.add("origin_y", "20.0");
        opts.add("width", 10);
        opts.add("height", 5);
        opts.add("bounds", "([1, 10],[5, 25])");
        opts.add("filename", outfile);
        opts.add("resolution", 1.1);

        PointTable t;
        GDALWriter w;
        w.setOptions(opts);
        w.setInput(r);
        EXPECT_THROW(w.prepare(t), pdal_error);
    }

    {
        Options opts;
        opts.add("origin_x", "10.0");
        opts.add("origin_y", "20.0");
        opts.add("width", 10);
        opts.add("height", 5);
        opts.add("filename", outfile);
        opts.add("resolution", 1);

        PointTable t;
        GDALWriter w;
        w.setOptions(opts);
        w.setInput(r);
        w.prepare(t);
        w.execute(t);

        gdal::Raster raster(outfile, "GTiff");
        raster.open();
        EXPECT_EQ(raster.width(), 10);
        EXPECT_EQ(raster.height(), 5);
    }
}

TEST(GDALWriterTest, srs)
{
    auto test = [](const std::string& sourceSrs, const std::string& defaultSrs,
        const std::string& overrideSrs, const std::string& testSrs)
    {
        std::string outfile(Support::temppath("out.tif"));

        TextReader t;
        Options to;
        to.add("filename", Support::datapath("text/with_edgeofflightline.txt"));
        if (sourceSrs.size())
            to.add("override_srs", sourceSrs);
        t.setOptions(to);

        GDALWriter w;
        Options wo;
        wo.add("filename", outfile);
        wo.add("origin_x", 0);
        wo.add("origin_y", 0);
        wo.add("width", 1000);
        wo.add("height", 1000);
        wo.add("resolution", 10);
        if (defaultSrs.size())
            wo.add("default_srs", defaultSrs);
        if (overrideSrs.size())
            wo.add("override_srs", overrideSrs);
        w.setOptions(wo);
        w.setInput(t);

        FileUtils::deleteFile(outfile);
        PointTable table;
        w.prepare(table);
        w.execute(table);

        gdal::Raster raster(outfile);
        raster.open();
        EXPECT_EQ(raster.getSpatialRef(), testSrs);
    };

    test("", "EPSG:4326", "", "EPSG:4326");
    test("", "", "EPSG:4326", "EPSG:4326");
    test("EPSG:4326", "EPSG:2030", "", "EPSG:4326");
    test("EPSG:4326", "", "EPSG:2030", "EPSG:2030");
    EXPECT_THROW(test("EPSG:4326", "EPSG:4326", "EPSG:2030", "EPSG:2030"), pdal_error);
}

} // namespace pdal
