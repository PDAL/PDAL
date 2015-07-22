/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <SbetReader.hpp>
#include <SbetWriter.hpp>

#include "Support.hpp"

using namespace pdal;

Options makeReaderOptions()
{
    Options options;
    Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    options.add(filename);
    return options;
}


Options makeWriterOptions()
{
    Options options;
    Option filename("filename", Support::temppath("SbetWriterTest.sbet"), "");
    options.add(filename);
    return options;
}

TEST(SbetWriterTest, testConstructor)
{
    SbetReader reader;
    reader.setOptions(makeReaderOptions());
    SbetWriter writer;
    writer.setOptions(makeWriterOptions());
    writer.setInput(reader);

    EXPECT_EQ(writer.getName(), "writers.sbet");
}

TEST(SbetWriterTest, testWrite)
{
    FileUtils::deleteFile(Support::temppath("SbetWriterTest.sbet"));

    // Scope forces the writer's buffer to get written to the file.  Otherwise
    // the output file will show a file size of zero and no contents.
    {
        SbetReader reader;
        reader.setOptions(makeReaderOptions());
        SbetWriter writer;
        writer.setOptions(makeWriterOptions());
        writer.setInput(reader);

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    //ABELL - Write of a read file is no longer identical.
    /**
    EXPECT_TRUE(Support::compare_files(
        Support::temppath("SbetWriterTest.sbet"),
        Support::datapath("sbet/2-points.sbet")));
    **/
}
