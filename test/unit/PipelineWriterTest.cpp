/******************************************************************************
* Copyright (c) 2019, Michael P. Gerlek (mpg@flaxen.com)
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

#include <sstream>

#include "Support.hpp"

#include <pdal/PipelineWriter.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

// Make sure we handle duplicate stages properly.
TEST(PipelineManagerTest, issue_2458)
{
    std::string in = R"(
        [
            "in.las",
            "in2.las",
            "out.las"
        ]
    )";

    PipelineManager mgr;
    std::istringstream iss(in);
    mgr.readPipeline(iss);

    std::ostringstream oss;
    PipelineWriter::writePipeline(mgr.getStage(), oss);

    std::string out = oss.str();
    EXPECT_TRUE(out.find("readers_las1") != std::string::npos);
    EXPECT_TRUE(out.find("readers_las2") != std::string::npos);
    EXPECT_TRUE(out.find("writers_las1") != std::string::npos);
}
