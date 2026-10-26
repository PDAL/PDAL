/******************************************************************************
* Copyright (c) 2026, Hobu, Inc. (howard@hobu.co)
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

#include <iostream>
#include <string>

#include <pdal/pdal_features.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

namespace pdal
{

#ifdef PDAL_HAVE_OGR_PARQUET
// Testing stac-geoparquet
TEST(TIndex, test9)
{
    std::string inSpec(Support::datapath("tindex/t1.txt"));
    std::string outSpec(Support::temppath("tindex.parquet"));

    std::string cmd = Support::binpath("pdal") + " stacindex create " +
        outSpec + " \"" + inSpec + "\" --log=stdout " +
        "-f GeoJSON 2>&1";

    FileUtils::deleteFile(outSpec);
    std::string output;
    Utils::run_shell_command(cmd, output);
    std::string::size_type pos =
        output.find("Can't specify OGR driver when outputting to STAC GeoParquet");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " stacindex create " +
        outSpec + " \"" + inSpec + "\" --threshold=1 " +
        "--resolution=1.0 --log=stdout " +
        "--statistics=true 2>&1";

    FileUtils::deleteFile(outSpec);
    Utils::run_shell_command(cmd, output);

    cmd = "ogrinfo -al -q " + outSpec + " 2>&1";
    std::string info;
    if (Utils::run_shell_command(cmd, info)) {
        std::cerr << "WARNING: error running ogrinfo, skipping test" << std::endl;
        return;
    }

    // Not checking full filename (assets.data.href) since it depends on relative path
    pos = info.find("pc:encoding (String) = .txt");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("pc:count (Integer64) = 4");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("pc:statistics (String(JSON)) = [");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("pc:encoding (String) = .txt");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("pc:schemas (String(JSON)) = [");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("proj:projjson (String(JSON)) = {");
    EXPECT_NE(pos, std::string::npos);

    pos = info.find("id (String) = t1.txt");
    EXPECT_NE(pos, std::string::npos);

    FileUtils::deleteFile(outSpec);
}

// Testing adding static fields
TEST(TIndex, test10)
{
    std::string inSpec(Support::datapath("tindex/*.txt"));
    std::string outSpec(Support::temppath("tindex.parquet"));

    // Testing string
    std::string cmd = Support::binpath("pdal") + " stacindex create " +
        outSpec + " \"" + inSpec + "\" --log=stdout " +
        "--static_fields=\"{\\\"foo\\\": \\\"bar\\\"}\" 2>&1";

    FileUtils::deleteFile(outSpec);
    std::string output;
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0) 
        << "failed with output: " << output;

    cmd = "ogrinfo -al -q " + outSpec + " 2>&1";
    std::string info;
    if (Utils::run_shell_command(cmd, info)) {
        std::cerr << "WARNING: error running ogrinfo, skipping test" << std::endl;
        return;
    }
    std::string::size_type pos = info.find("foo (String) = bar");
    EXPECT_NE(pos, std::string::npos);

    // Testing list
    cmd = Support::binpath("pdal") + " stacindex create " +
        outSpec + " \"" + inSpec + "\" --log=stdout " +
        "--static_fields=\"{\\\"foo\\\": [\\\"bar\\\"],"
        "\\\"test\\\": 2 }\" 2>&1";

    FileUtils::deleteFile(outSpec);
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0)
        << "failed with output: " << output;

    cmd = "ogrinfo -al -q " + outSpec + " 2>&1";
    if (Utils::run_shell_command(cmd, info)) {
        std::cerr << "WARNING: error running ogrinfo, skipping test" << std::endl;
        return;
    }
    pos = info.find("foo (StringList) = (1:bar)");
    EXPECT_NE(pos, std::string::npos);
    pos = info.find("test (Integer64) = 2");
    EXPECT_NE(pos, std::string::npos);

    // Testing file input
    cmd = Support::binpath("pdal") + " stacindex create " +
        outSpec + " \"" + inSpec + "\" --log=stdout " +
        "--static_fields=\"" +
        Support::datapath("tindex/fields.json") + "\" 2>&1";

    FileUtils::deleteFile(outSpec);
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0)
        << "failed with output: " << output;

    cmd = "ogrinfo -al -q " + outSpec + " 2>&1";
    if (Utils::run_shell_command(cmd, info)) {
        std::cerr << "WARNING: error running ogrinfo, skipping test" << std::endl;
        return;
    }
    EXPECT_NE(pos, std::string::npos);
    pos = info.find("foo (Integer64) = -100");
    EXPECT_NE(pos, std::string::npos);
}
#endif // PDAL_HAVE_OGR_PARQUET
} // namespace pdal