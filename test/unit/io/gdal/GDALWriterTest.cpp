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

#include "GDALWriter.hpp"
#include "TextReader.hpp"
#include "Support.hpp"

#include <iostream>
#include <sstream>

using namespace pdal;

namespace
{

void RunGdalReader(const Options& wo, const std::string& outfile,
    const std::string& values)
{
    FileUtils::deleteFile(outfile);

    Options ro;
    ro.add("filename", Support::datapath("gdal/grid.txt"));

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
    std::vector<uint8_t> data;
    raster.readBand(data, 1);
    double *d = reinterpret_cast<double *>(data.data());
    for (size_t i = 0; i < arr.size(); ++i)
        EXPECT_NEAR(arr[i], *d++, .001);
}

}

TEST(GDALWriterTest, min)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "min");
    wo.add("edge_length", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output = 
        "5.000 -9999.000     7.000     8.000     9.000 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.500     6.500 "
        "2.000     3.000     4.000     4.500     5.500 "
        "1.000     2.000     3.000     4.000     5.000 ";

    RunGdalReader(wo, outfile, output);
}

TEST(GDALWriterTest, minWindow)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "min");
    wo.add("edge_length", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output = 
        "5.000     5.464     7.000     8.000     9.000 "
        "4.000     4.857     6.000     7.000     8.000 "
        "3.000     4.000     5.000     5.500     6.500 "
        "2.000     3.000     4.000     4.500     5.500 "
        "1.000     2.000     3.000     4.000     5.000 ";

    RunGdalReader(wo, outfile, output);
}

TEST(GDALWriterTest, max)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "max");
    wo.add("edge_length", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output = 
        "5.000 -9999.000     7.000     8.000     9.000 "
        "4.000 -9999.000     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.500     6.500 "
        "1.000     2.000     3.000     4.500     5.500 ";

    RunGdalReader(wo, outfile, output);
}

TEST(GDALWriterTest, maxWindow)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "max");
    wo.add("edge_length", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);
    wo.add("window_size", 2);

    const std::string output = 
        "5.000     5.500     7.000     8.000     9.000 "
        "4.000     4.929     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.500     6.500 "
        "1.000     2.000     3.000     4.500     5.500 ";

    RunGdalReader(wo, outfile, output);
}

/**
TEST(GDALWriterTest, mean)
{
    std::string outfile = Support::temppath("tmp.tif");

    Options wo;
    wo.add("gdaldriver", "GTiff");
    wo.add("output_type", "mean");
    wo.add("edge_length", 1);
    wo.add("radius", .7071);
    wo.add("filename", outfile);

    const std::string output = 
        "5.000     5.500     7.000     8.000     9.000 "
        "4.000     4.929     6.000     7.000     8.000 "
        "3.000     4.000     5.000     6.000     7.000 "
        "2.000     3.000     4.000     5.500     6.500 "
        "1.000     2.000     3.000     4.500     5.500 ";

    RunGdalReader(wo, outfile, output);
}
**/

