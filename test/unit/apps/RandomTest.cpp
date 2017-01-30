/******************************************************************************
* Copyright (c) 2015, Hobu Inc., (info@hobu.co)
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
#include <pdal/PDALUtils.hpp>
#include <io/LasReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(Random, extra_ops)
{
    std::string outfile(Support::temppath("out.las"));

    const std::string cmd = Support::binpath("pdal") + " random "
        "--count=100 --writers.las.minor_version=3 " + outfile;

    FileUtils::deleteFile(outfile);
    std::string output;
    Utils::run_shell_command(cmd, output);

    Options o;
    o.add("filename", outfile);

    PointTable t;

    LasReader r;
    r.setOptions(o);
    r.prepare(t);

    MetadataNode n = r.getMetadata();
    EXPECT_EQ(n.findChild("minor_version").value<uint8_t>(), 3);
}

