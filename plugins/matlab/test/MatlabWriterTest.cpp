/******************************************************************************
 * Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include "../io/MatlabWriter.hpp"

#include <FauxReader.hpp>
#include <pdal/StageFactory.hpp>
#include "Support.hpp"


namespace pdal
{


class MatlabWriterTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        Options options;
        options.add("mode", "ramp");
        options.add("count", 100);
        m_reader.setOptions(options);
    }

    FauxReader m_reader;

};


TEST_F(MatlabWriterTest, constructor)
{
    MatlabWriter writer;
}


TEST_F(MatlabWriterTest, findStage)
{
    StageFactory factory;
    Stage* stage(factory.createStage("writers.matlab"));
    EXPECT_TRUE(stage);
}


TEST_F(MatlabWriterTest, write)
{
    Options options;
    options.add("filename", Support::temppath("out.mat"));
    MatlabWriter writer;
    writer.setOptions(options);
    writer.setInput(m_reader);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);
}


TEST_F(MatlabWriterTest, outputDims)
{
    Options options;
    options.add("filename", Support::temppath("out.mat"));
    options.add("output_dims", "X,Y");
    MatlabWriter writer;
    writer.setOptions(options);
    writer.setInput(m_reader);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);
}


}
