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
#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>


BOOST_AUTO_TEST_SUITE(pcinfoTest)


static std::string appName()
{
    const std::string app = Support::binpath(Support::exename("pcinfo"));
    BOOST_CHECK(pdal::FileUtils::fileExists(app));
    return app;
}


#ifdef PDAL_COMPILER_MSVC
BOOST_AUTO_TEST_CASE(pcinfoTest_no_input)
{
    const std::string cmd = appName();

    std::string output;
    int stat = Support::run_command(cmd, output);
    BOOST_CHECK_EQUAL(stat, 1);

    const std::string expected = "Usage error: input file name required";
    BOOST_CHECK_EQUAL(output.substr(0, expected.length()), expected);

    return;
}
#endif


BOOST_AUTO_TEST_CASE(pcinfo_test_common_opts)
{
    const std::string cmd = appName();

    std::string output;
    int stat = Support::run_command(cmd + " -h", output);
    BOOST_CHECK_EQUAL(stat, 0);

    stat = Support::run_command(cmd + " --version", output);
    BOOST_CHECK_EQUAL(stat, 0);

    return;
}


BOOST_AUTO_TEST_CASE(pcinfo_test_switches)
{
    const std::string cmd = appName();

    std::string inputLas = Support::datapath("apps/simple.las");
    std::string inputLaz = Support::datapath("apps/simple.laz");

    std::string output;

    int stat = 0;
    
    // does the default work?
    stat = Support::run_command(cmd + " " + inputLas, output);
    BOOST_CHECK_EQUAL(stat, 0);

    // does --input work?
    stat = Support::run_command(cmd + " --input=" + inputLas, output);
    BOOST_CHECK_EQUAL(stat, 0);

    // does -i work?
    stat = Support::run_command(cmd + " -i " + inputLas, output);
    BOOST_CHECK_EQUAL(stat, 0);

#ifdef PDAL_HAVE_LASZIP
    // does it work for .laz?
    stat = Support::run_command(cmd + " " + inputLaz, output);
    BOOST_CHECK_EQUAL(stat, 0);
#endif

#ifdef PDAL_HAVE_LIBLAS
    // does --liblas work?
    stat = Support::run_command(cmd + " --liblas " + inputLas, output);
    BOOST_CHECK_EQUAL(stat, 0);
#endif

#ifdef PDAL_HAVE_LIBLAS
#ifdef PDAL_HAVE_LASZIP
    // does --liblas work for .laz too?
    stat = Support::run_command(cmd + " --liblas " + inputLaz, output);
    BOOST_CHECK_EQUAL(stat, 0);
#endif
#endif

    return;
}


BOOST_AUTO_TEST_CASE(pcinfo_test_dumps)
{
    const std::string cmd = appName();

    const std::string inputLas = Support::datapath("apps/simple.las");
    const std::string inputLaz = Support::datapath("apps/simple.laz");
    const std::string outputTxt = Support::temppath("pcinfo.txt");

    std::string output;

    int stat = 0;

	std::ostringstream command;
    
    // dump a single point to json
	
	command << cmd + " --output=" + outputTxt + " --point=1 " + inputLas;
    stat = Support::run_command(command.str(), output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(Support::compare_text_files(outputTxt, Support::datapath("apps/pcinfo_point.txt")));

    // dump summary of all points to json
	command.str("");
	command << cmd + " --output=" + outputTxt + " --stats " + inputLas;
    stat = Support::run_command(command.str(), output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(Support::compare_text_files(outputTxt, Support::datapath("apps/pcinfo_stats.txt")));

    // dump schema to json
	command.str("");
	command << cmd + " --output=" + outputTxt + " --schema " + inputLas;
    stat = Support::run_command(command.str(), output);
    BOOST_CHECK_EQUAL(stat, 0);
    BOOST_CHECK(Support::compare_text_files(outputTxt, Support::datapath("apps/pcinfo_schema.txt")));

    // dump stage info to json
	command.str("");
    stat = Support::run_command(command.str(), output);
    BOOST_CHECK_EQUAL(stat, 0);

#ifdef PDAL_HAVE_GDAL
    BOOST_CHECK_EQUAL(Support::diff_text_files(outputTxt, Support::datapath("apps/pcinfo_stage.txt"), 15), 0u);
#else
    BOOST_CHECK_EQUAL(Support::diff_text_files(outputTxt, Support::datapath("apps/pcinfo_stage_nosrs.txt"), 15), 0u);
#endif

    pdal::FileUtils::deleteFile(outputTxt);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
