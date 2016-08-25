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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include "Support.hpp"

#include <pdal/plang/Environment.hpp>

#include <iostream>
#include <sstream>
#include <string>

// most pipelines (those with a writer) will be invoked via `pdal pipeline`
static void run_pipeline(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") +
        " pipeline");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

class jsonWithProgrammable : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithProgrammable, pipeline)
{
    pdal::plang::Environment::get();
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("filters.programmable");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create filters.programmable, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithProgrammable,
                        testing::Values(
                            // "pipeline/programmable-hag.json",
                            "pipeline/programmable-update-y-dims.json"
                        ));

class jsonWithPredicate : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithPredicate, pipeline)
{
    pdal::plang::Environment::get();
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("filters.predicate");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create filters.predicate, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithPredicate,
                        testing::Values(
                            "pipeline/crop_wkt_2d_classification.json",
                            "pipeline/from-module.json",
                            "pipeline/predicate-embed.json",
                            "pipeline/predicate-keep-ground-and-unclass.json",
                            "pipeline/predicate-keep-last-return.json",
                            "pipeline/predicate-keep-specified-returns.json",
                            "pipeline/reproject.json"
                        ));
