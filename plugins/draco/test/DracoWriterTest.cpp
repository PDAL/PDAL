/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#define NOMINMAX

#include <stdio.h>
#include <sys/types.h>

#include <nlohmann/json.hpp>

#include <pdal/pdal_test_main.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"
#include "../io/DracoWriter.hpp"

namespace pdal
{
    size_t count = 100;
    class DracoWriterTest : public ::testing::Test
    {
    protected:
        virtual void SetUp()
        {
            Options options;
            options.add("mode", "ramp");
            options.add("count", count);
            m_reader.setOptions(options);
            m_reader2.setOptions(options);
        }

        FauxReader m_reader;
        FauxReader m_reader2;

    };

    TEST_F(DracoWriterTest, constructor)
    {
        DracoWriter writer;
    }

    TEST_F(DracoWriterTest, findStage)
    {
        StageFactory factory;
        Stage* stage(factory.createStage("writers.draco"));
        EXPECT_TRUE(stage);
        EXPECT_TRUE(stage->pipelineStreamable());
    }

    TEST_F(DracoWriterTest, write)
    {
        std::string pth = Support::temppath("draco_test_out");

        Options options;
//         options.add("chunk_size", 80);


        DracoWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);


    }

}
