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

#include <pdal/util/FileUtils.hpp>
#include <LasReader.hpp>

#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>

using namespace pdal;

static std::string appName()
{
    return Support::binpath(Support::exename("pdal") + " translate");
}


#ifdef PDAL_COMPILER_MSVC
TEST(pc2pcTest, pc2pcTest_test_no_input)
{
    std::string cmd = appName();

    std::string output;
    int stat = Utils::run_shell_command(cmd, output);
    EXPECT_EQ(stat, 1);

    const std::string expected = "Usage error: --input";
    EXPECT_EQ(output.substr(0, expected.length()), expected);
}
#endif


TEST(pc2pcTest, pc2pcTest_test_common_opts)
{
    std::string cmd = appName();

    std::string output;
    int stat = Utils::run_shell_command(cmd + " -h", output);
    EXPECT_EQ(stat, 0);

    stat = Utils::run_shell_command(cmd + " --version", output);
    EXPECT_EQ(stat, 0);
}


static bool fileIsOkay(const std::string& name)
{
    if (!FileUtils::fileExists(name))
        return false;
    if (FileUtils::fileSize(name) < 1000)
        return false;
    return true;
}


static bool fileIsCompressed(const std::string& name)
{
    PointTable table;

    Options ops;
    ops.add("filename", name);
    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(ops);
    reader->prepare(table);
    return reader->header().compressed();
}


static bool fileHasSrs(const std::string& name)
{
    PointTable table;

    Options ops;
    ops.add("filename", name);
    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(ops);
    reader->prepare(table);
    reader->execute(table);
    return !reader->getSpatialReference().empty();
}


TEST(pc2pcTest, pc2pc_test_switches)
{
    std::string cmd = appName();

    std::string inputLas = Support::datapath("apps/simple.las");
    std::string inputLaz = Support::datapath("apps/simple.laz");
    std::string outputLas = Support::temppath("temp.las");
    std::string outputLaz = Support::temppath("temp.laz");

    std::string output;

    int stat = 0;

    // We don't generally test the output files against a reference file
    // because we don't want to have to update the reference files every time
    // we change the driver implementation -- those issues are covered by the
    // unit tests for the drivers.  Instead, here we just check certain rough
    // characteristics of the output file.

    // do --input and --output work?
    std::string fullCmd = cmd + " --input=" + inputLas + " --output=" +
        outputLas;
    stat = Utils::run_shell_command(fullCmd, output);
    EXPECT_EQ(stat, 0);
    EXPECT_TRUE(fileIsOkay(outputLas));
    EXPECT_TRUE(!fileIsCompressed(outputLas));
#ifdef PDAL_HAVE_LIBGEOTIFF
    EXPECT_TRUE(!fileHasSrs(outputLas));
#else
    (void)fileHasSrs(outputLas);
#endif

#ifdef PDAL_HAVE_LASZIP
    // does --compress make a compressed file?
    stat = Utils::run_shell_command(cmd + " --input=" + inputLas +
        " --output=" + outputLas + " --writers.las.compression=true", output);
    EXPECT_EQ(stat, 0);
    EXPECT_TRUE(fileIsOkay(outputLas));
    EXPECT_TRUE(fileIsCompressed(outputLas));
#endif

#ifdef PDAL_HAVE_LASZIP
    // does "--output foo.laz" make a compressed output?
    stat = Utils::run_shell_command(cmd + " --input=" + inputLas +
        " --output=" + outputLaz, output);
    EXPECT_EQ(stat, 0);
    EXPECT_TRUE(fileIsOkay(outputLaz));
    EXPECT_TRUE(fileIsCompressed(outputLaz));
#endif

    // does --a_srs add an SRS?
    fullCmd = cmd + " --input=" + inputLas + " --output=" + outputLas +
        " --readers.las.spatialreference=epsg:4326";
    stat = Utils::run_shell_command(fullCmd, output);
    EXPECT_EQ(stat, 0);
    EXPECT_TRUE(fileIsOkay(outputLas));
#ifdef PDAL_HAVE_LIBGEOTIFF
    EXPECT_TRUE(fileHasSrs(outputLas));
#endif

    FileUtils::deleteFile(outputLas);
    FileUtils::deleteFile(outputLaz);
}
