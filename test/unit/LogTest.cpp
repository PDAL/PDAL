/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "gtest/gtest.h"
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(LogTest, test_one)
{
    StageFactory f;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);

    Option opt1("bounds", bounds);
    Option opt2("log", Support::temppath("mylog_one.txt"));
    Option opt3("num_points", 750);
    Option opt4("mode", "constant");

    Options opts;
    opts.add(opt1);
    opts.add(opt2);
    opts.add(opt3);
    opts.add(opt4);

    {
        PointContext ctx;

        ReaderPtr reader(f.createReader("readers.faux"));
        reader->setOptions(opts);
        reader->prepare(ctx);

        EXPECT_EQ(reader->log()->getLevel(), LogLevel::Error);
        reader->log()->setLevel(LogLevel::Debug5);
        EXPECT_EQ(reader->log()->getLevel(), LogLevel::Debug5);

        PointBufferSet pbSet = reader->execute(ctx);
        EXPECT_EQ(pbSet.size(), 1u);
        PointBufferPtr buf = *pbSet.begin();
        EXPECT_EQ(buf->size(), 750u);
    }
    bool ok = Support::compare_text_files(
        Support::temppath("mylog_one.txt"),
        Support::datapath("logs/logtest.txt"));

    if (ok)
        FileUtils::deleteFile(Support::temppath("mylog_one.txt"));
    
    EXPECT_TRUE(ok);
}

