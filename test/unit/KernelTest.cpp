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

#include <pdal/Kernel.hpp>

#include "Support.hpp"

namespace pdal
{

/**
class TestKernel : public Kernel
{
public:
    virtual std::string getName()
        { return "TestKernel"; }
    int execute()
        { return 0; }
    bool test_parseStageOption(std::string o, std::string& stage,
        std::string& option, std::string& value)
    { return Kernel::parseStageOption(o, stage, option, value); }
}
**/

TEST(KernelTest, parseOption)
{
    std::string stage;
    std::string option;
    std::string value;
    Kernel::ParseStageResult res;

    res = Kernel::test_parseStageOption("--readers.p2g.foobar=baz",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Ok);
    EXPECT_EQ(stage, "readers.p2g");
    EXPECT_EQ(option, "foobar");
    EXPECT_EQ(value, "baz");

    res = Kernel::test_parseStageOption("--readers.2pg.foobar=baz",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Unknown);

    res = Kernel::test_parseStageOption("--read1ers.las.foobar=baz",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Unknown);

    res = Kernel::test_parseStageOption("--readers.p2g.foobar",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Ok);
    EXPECT_EQ(value, "");

    res = Kernel::test_parseStageOption("--readers.p2g.foobar=",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Invalid);

    res = Kernel::test_parseStageOption("--readers.p2g.foobar!",
        stage, option, value);
    EXPECT_EQ(res, Kernel::ParseStageResult::Invalid);
}

}
