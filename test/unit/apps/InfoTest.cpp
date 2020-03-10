/******************************************************************************
* Copyright (c) 2018, Hobu Inc., (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{
std::string appName()
{
    return Support::binpath("pdal info");
}
}

void test(const std::string options, const std::string validation)
{
    std::string cmd;

    std::string output;
    cmd = appName() + " " + options + " " +
        Support::datapath("las/autzen_trim.las") + " 2>&1";

    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0);
    EXPECT_NE(output.find(validation), std::string::npos)
        << "Found: '" << output << "'" << std::endl
        << "expected: '" << validation<<"'" << std::endl;
}

TEST(Info, point)
{
std::string r = R"foo(
  "points":
  {
    "point":
    {
      "Blue": 90,
      "Classification": 2,
      "EdgeOfFlightLine": 0,
      "GpsTime": 245379.401,
      "Green": 98,
      "Intensity": 1,
      "NumberOfReturns": 1,
      "PointId": 5,
      "PointSourceId": 7326,
      "Red": 82,
      "ReturnNumber": 1,
      "ScanAngleRank": -17,
      "ScanDirectionFlag": 1,
      "UserData": 130,
      "X": 637176.73,
      "Y": 849397.08,
      "Z": 410.89
    }
  },
)foo";

    test("-p 5", r);
}

TEST(Info, query)
{
std::string r = R"foo(
      {
        "Blue": 100,
        "Classification": 1,
        "EdgeOfFlightLine": 0,
        "GpsTime": 245385.9092,
        "Green": 127,
        "Intensity": 84,
        "NumberOfReturns": 1,
        "PointId": 109811,
        "PointSourceId": 7326,
        "Red": 114,
        "ReturnNumber": 1,
        "ScanAngleRank": -1,
        "ScanDirectionFlag": 0,
        "UserData": 122,
        "X": 636125.88,
        "Y": 848971.06,
        "Z": 428.05
      },
)foo";

    test("--query 0,0/5", r);
}

TEST(Info, stats)
{
std::string r = R"foo(
    "statistic":
    [
      {
        "average": 636546.405,
        "count": 110000,
        "maximum": 637179.22,
        "minimum": 636001.76,
        "name": "X",
        "position": 0,
        "stddev": 314.9304199,
        "variance": 99181.16939
      },
      {
        "average": 849145.7857,
        "count": 110000,
        "maximum": 849497.9,
        "minimum": 848935.2,
        "name": "Y",
        "position": 1,
        "stddev": 124.3281512,
        "variance": 15457.48917
      },
)foo";

    test("", r);

// 10-Jan-20 - Broken by a change to proj which converts meters to ft, I think.
/**
std::string s = R"foo(
      "EPSG:4326":
      {
        "bbox":
        {
          "maxx": -123.0689038,
          "maxy": 44.0515451,
          "maxz": 158.651448,
          "minx": -123.0734481,
          "miny": 44.04990077,
          "minz": 123.828048
        },
)foo";
    test("", s);
**/
}

TEST(Info, schema)
{
std::string r = R"foo(
      {
        "name": "Z",
        "size": 8,
        "type": "floating"
      },
      {
        "name": "Intensity",
        "size": 2,
        "type": "unsigned"
      },
)foo";
    test("--schema", r);
}

TEST(Info, all)
{
std::string r = R"foo(
      {
        "name": "Z",
        "size": 8,
        "type": "floating"
      },
      {
        "name": "Intensity",
        "size": 2,
        "type": "unsigned"
      },
)foo";
    test("--all", r);
}
