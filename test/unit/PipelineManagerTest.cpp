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

#include "Support.hpp"

#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

TEST(PipelineManagerTest, basic)
{
    std::string outfile = Support::temppath("temp.las");
    FileUtils::deleteFile(outfile);

    PipelineManager mgr;

    Options optsR;
    optsR.add("filename", Support::datapath("las/1.2-with-color.las"));
    Stage& reader = mgr.addReader("readers.las");
    reader.setOptions(optsR);

    Options optsW;
    optsW.add("filename", outfile);
    Stage& writer = mgr.addWriter("writers.las");
    writer.setInput(reader);
    writer.setOptions(optsW);

    point_count_t np = mgr.execute();
    EXPECT_EQ(np, 1065U);

    EXPECT_TRUE(!std::ifstream(outfile).fail());
    FileUtils::deleteFile(outfile);
}

// Make sure that when we add an option at the command line, it overrides
// a pipeline option.
TEST(PipelineManagerTest, OptionOrder)
{
    std::string cmd = Support::binpath(Support::exename("pdal") +
        " pipeline");

    std::string file(Support::configuredpath("pipeline/sort2.json"));

    std::string output;
    int stat = Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(stat, 0);

    StageFactory f;
    Stage *r = f.createStage("readers.las");

    Options o;
    o.add("filename", Support::temppath("sorted.las"));
    r->setOptions(o);

    PointTable t;
    r->prepare(t);
    PointViewSet s = r->execute(t);
    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *(s.begin());

    double prev = std::numeric_limits<double>::lowest();
    for (PointId idx = 0; idx < v->size(); ++idx)
    {
        double d = v->getFieldAs<double>(Dimension::Id::X, idx);
        EXPECT_GE(d, prev);
        prev = d;
    }
    FileUtils::deleteFile(Support::temppath("sorted.las"));

    stat = Utils::run_shell_command(cmd + " " + file +
        " --filters.sort.dimension=Y", output);
    EXPECT_EQ(stat, 0);

    Stage *r2 = f.createStage("readers.las");
    r2->setOptions(o);

    PointTable t2;
    r2->prepare(t2);
    s = r2->execute(t2);
    EXPECT_EQ(s.size(), 1U);
    v = *(s.begin());

    prev = std::numeric_limits<double>::lowest();
    for (PointId idx = 0; idx < v->size(); ++idx)
    {
        double d = v->getFieldAs<double>(Dimension::Id::Y, idx);
        EXPECT_GE(d, prev);
        prev = d;
    }
    FileUtils::deleteFile(Support::temppath("sorted.las"));
}

// Make sure that when we add an option at the command line, it overrides
// a pipeline option.
TEST(PipelineManagerTest, InputGlobbing)
{
    std::string cmd = Support::binpath(Support::exename("pdal") +
        " pipeline");

    std::string file(Support::configuredpath("pipeline/glob.json"));

    std::string output;
    int stat = Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(stat, 0);

    StageFactory f;
    Stage *r = f.createStage("readers.las");

    Options o;
    o.add("filename", Support::temppath("globbed.las"));
    r->setOptions(o);

    PointTable t;
    r->prepare(t);
    PointViewSet s = r->execute(t);
    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *(s.begin());

    EXPECT_EQ(v->size(), 10653U);

    FileUtils::deleteFile(Support::temppath("globbed.las"));
}

// EPT addon writer options are objects and not strings
TEST(PipelineManagerTest, objects)
{
    std::string cmd = Support::binpath(Support::exename("pdal") +
                                       " pipeline --validate");
    std::string file = Support::configuredpath("pipeline/ept_addon.json");

    std::string output;
    EXPECT_NO_THROW(Utils::run_shell_command(cmd + " " + file, output));
}

TEST(PipelineManagerTest, arrayPipeline)
{
    std::string cmd = Support::binpath(Support::exename("pdal") +
        " pipeline");

    std::string file(Support::configuredpath("pipeline/array-pipeline.json"));

    std::string output;
    int stat = Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(stat, 0);

    StageFactory f;
    Stage *r = f.createStage("readers.las");

    Options o;
    o.add("filename", Support::temppath("array-pipeline.las"));
    r->setOptions(o);

    PointTable t;
    r->prepare(t);
    PointViewSet s = r->execute(t);
    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *(s.begin());

    EXPECT_EQ(v->size(), 10653U);

    FileUtils::deleteFile(Support::temppath("array-pipeline.las"));
}

TEST(PipelineManagerTest, replace)
{
    PipelineManager mgr;

    Stage& r = mgr.makeReader("in.las", "readers.las");
    Stage& f = mgr.makeFilter("filters.crop", r);
    Stage& w = mgr.makeWriter("out.las", "writers.las", f);

    StageFactory factory;
    Stage *r2 = factory.createStage("readers.bpf");
    mgr.replace(&r, r2);
    EXPECT_EQ(r2->getInputs().size(), 0U);

    EXPECT_EQ(f.getInputs().size(), 1U);
    EXPECT_EQ(f.getInputs().front(), r2);

    EXPECT_EQ(w.getInputs().size(), 1U);
    EXPECT_EQ(w.getInputs().front(), &f);

    Stage *f2 = factory.createStage("filters.range");

    mgr.replace(&f, f2);
    EXPECT_EQ(r2->getInputs().size(), 0U);

    EXPECT_EQ(f2->getInputs().size(), 1U);
    EXPECT_EQ(f2->getInputs().front(), r2);

    EXPECT_EQ(w.getInputs().size(), 1U);
    EXPECT_EQ(w.getInputs().front(), f2);

    Stage *w2 = factory.createStage("writers.bpf");

    mgr.replace(&w, w2);
    EXPECT_EQ(r2->getInputs().size(), 0U);

    EXPECT_EQ(f2->getInputs().size(), 1U);
    EXPECT_EQ(f2->getInputs().front(), r2);

    EXPECT_EQ(w2->getInputs().size(), 1U);
    EXPECT_EQ(w2->getInputs().front(), f2);
}
