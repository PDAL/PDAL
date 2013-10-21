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

#include <boost/test/unit_test.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>


BOOST_AUTO_TEST_SUITE(pc2pcTest)


static std::string appName()
{
    const std::string app = Support::binpath(Support::exename("pdal translate"));
    return app;
}


#ifdef PDAL_COMPILER_MSVC
BOOST_AUTO_TEST_CASE(pc2pcTest_test_no_input)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd, output);
    BOOST_CHECK_EQUAL(stat, 1);

    const std::string expected = "Usage error: --input";
    BOOST_CHECK_EQUAL(output.substr(0, expected.length()), expected);

    return;
}
#endif


BOOST_AUTO_TEST_CASE(pc2pcTest_test_common_opts)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd + " -h", output);
    BOOST_CHECK_EQUAL(stat, 0);

    stat = pdal::Utils::run_shell_command(cmd + " --version", output);
    BOOST_CHECK_EQUAL(stat, 0);

    return;
}


static bool fileIsOkay(const std::string& name)
{
    if (!pdal::FileUtils::fileExists(name)) return false;
    if (pdal::FileUtils::fileSize(name) < 1000) return false;
    return true;
}


static bool fileIsCompressed(const std::string& name)
{
    pdal::drivers::las::Reader reader(name);
    reader.initialize();
    return reader.getLasHeader().Compressed();
}


static bool fileHasSrs(const std::string& name)
{
    pdal::drivers::las::Reader reader(name);
    reader.initialize();
    return !reader.getSpatialReference().empty();
}


BOOST_AUTO_TEST_CASE(pc2pc_test_switches)
{
    const std::string cmd = appName();

    std::string inputLas = Support::datapath("apps/simple.las");
    std::string inputLaz = Support::datapath("apps/simple.laz");
    std::string outputLas = Support::temppath("temp.las");
    std::string outputLaz = Support::temppath("temp.laz");

    std::string output;

    int stat = 0;

    // We don't generally test the outputted files against a reference file, because
    // we don't want to have to update the reference files everytime we change the driver
    // implementation -- those issues are covered by the unit tests for the drivers.
    // Instead, here we just check certain rough characteristics of the outputted file.

    // do --input and --output work?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLas, output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLas));
    BOOST_CHECK(!fileIsCompressed(outputLas));
    BOOST_CHECK(!fileHasSrs(outputLas));

#ifdef PDAL_HAVE_LASZIP
    // does --compress make a compressed file?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLas + " --compress", output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLas));
    BOOST_CHECK(fileIsCompressed(outputLas));
#endif

#ifdef PDAL_HAVE_LASZIP
    // does "--output foo.laz" make a compressed output?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLaz, output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLaz));
    BOOST_CHECK(fileIsCompressed(outputLaz));
#endif


#ifdef PDAL_HAVE_LIBLAS
    // does --liblas work?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLas + " --liblas", output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLas));
    BOOST_CHECK(!fileIsCompressed(outputLas));
#endif

#ifdef PDAL_HAVE_LIBLAS
#ifdef PDAL_HAVE_LASZIP
    // do --liblas and --compress work together?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLas + " --compress --liblas", output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLas));
    BOOST_CHECK(fileIsCompressed(outputLas));
#endif
#endif

#ifdef PDAL_HAVE_GDAL
    // does --a_srs add an SRS?
    stat = pdal::Utils::run_shell_command(cmd + " --input=" + inputLas + " --output=" + outputLas + " --a_srs=epsg:4326", output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(fileIsOkay(outputLas));
    BOOST_CHECK(fileHasSrs(outputLas));
#endif

    pdal::FileUtils::deleteFile(outputLas);
    pdal::FileUtils::deleteFile(outputLaz);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
