/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include "gtest/gtest.h"
#include <pdal/PipelineReader.hpp>

#include <sstream>

#include <yaml-cpp/yaml.h>

#include <pdal/PipelineManager.hpp>
#include "Support.hpp"


namespace pdal
{


TEST(PipelineParserYamlTest, Writer)
{
    PipelineManager manager;
    PipelineReader pipelineReader(manager);
    bool answer = pipelineReader.readPipeline(Support::datapath("yaml/writer.yml"));
    EXPECT_TRUE(answer);
    EXPECT_TRUE(manager.isWriterPipeline());

    Stage* writer = manager.getStage();
    EXPECT_EQ("outfile.las",
            writer->getOptions().getValueOrThrow<std::string>("filename"));

    Stage* filter = *writer->getInputs().begin();
    EXPECT_EQ(2,
            filter->getOptions().getValueOrThrow<int>("step"));

    Stage* reader = *filter->getInputs().begin();
    EXPECT_EQ("infile.las",
            reader->getOptions().getValueOrThrow<std::string>("filename"));
}


TEST(PipelineParserYamlTest, NoWriter)
{
    PipelineManager manager;
    PipelineReader reader(manager);
    bool answer = reader.readPipeline(Support::datapath("yaml/no-writer.yml"));
    EXPECT_FALSE(answer);
    EXPECT_FALSE(manager.isWriterPipeline());
    EXPECT_TRUE(manager.getStage());
}


TEST(PipelineParserYamlTest, NoWriterWithFilter)
{
    PipelineManager manager;
    PipelineReader reader(manager);
    bool answer = reader.readPipeline(Support::datapath("yaml/no-writer-with-filter.yml"));
    EXPECT_FALSE(answer);
    EXPECT_FALSE(manager.isWriterPipeline());
    EXPECT_TRUE(manager.getStage());
}


TEST(PipelineParserYamlTest, BadFile)
{
    PipelineManager manager;
    PipelineReader reader(manager);
    EXPECT_THROW(
            reader.readPipeline(Support::datapath("yaml/not-a-yaml-file.yml")),
            YAML::Exception);
}


TEST(PipelineParserYamlTest, Stream)
{
    PipelineManager manager;
    PipelineReader pipelineReader(manager);
    std::stringstream ss;
    ss << "readers.las:\n   filename: infile.las\nwriters.las:\n    filename: outfile.las\n";
    pipelineReader.readPipeline(ss, PipelineReader::ParseAs::Yaml);

    Stage* writer = manager.getStage();
    EXPECT_EQ("outfile.las",
            writer->getOptions().getValueOrThrow<std::string>("filename"));

    Stage* reader = *writer->getInputs().begin();
    EXPECT_EQ("infile.las",
            reader->getOptions().getValueOrThrow<std::string>("filename"));
}


TEST(PipelineParserYamlTest, BaseOptions)
{
    PipelineManager manager;
    PipelineReader pipelineReader(manager, true, 1);
    pipelineReader.readPipeline(Support::datapath("yaml/writer.yml"));

    Stage* writer = manager.getStage();
    EXPECT_TRUE(writer->getOptions().getValueOrThrow<bool>("debug"));
    EXPECT_EQ(1, writer->getOptions().getValueOrThrow<int>("verbose"));

    Stage* filter = *writer->getInputs().begin();
    EXPECT_TRUE(filter->getOptions().getValueOrThrow<bool>("debug"));
    EXPECT_EQ(1, filter->getOptions().getValueOrThrow<int>("verbose"));

    Stage* reader = *filter->getInputs().begin();
    EXPECT_TRUE(reader->getOptions().getValueOrThrow<bool>("debug"));
    EXPECT_EQ(1, reader->getOptions().getValueOrThrow<int>("verbose"));
}


}
