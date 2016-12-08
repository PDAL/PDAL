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

#include <pdal/pdal_test_main.hpp>

#include <array>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <io/BufferReader.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

void compare_contents(const std::string& las_file, const std::string& ntf_file)
{
    //
    // read the LAS file
    //
    Option las_opt("filename", las_file);
    Options las_opts;
    las_opts.add(las_opt);

    PointTable lasPoints;

    StageFactory f;

    Stage* las_reader(f.createStage("readers.las"));
    EXPECT_TRUE(las_reader);
    las_reader->setOptions(las_opts);
    las_reader->prepare(lasPoints);
    PointViewSet lasViews = las_reader->execute(lasPoints);
    EXPECT_EQ(lasViews.size(), 1u);
    PointViewPtr lasView = *lasViews.begin();

    //
    // read the NITF file
    //
    Option ntf_opt("filename", ntf_file);
    Options ntf_opts;
    ntf_opts.add(ntf_opt);

    PointTable ntfPoints;
    Stage* ntf_reader(f.createStage("readers.nitf"));
    EXPECT_TRUE(ntf_reader);
    ntf_reader->setOptions(ntf_opts);
    ntf_reader->prepare(ntfPoints);
    PointViewSet ntfViews = ntf_reader->execute(ntfPoints);
    EXPECT_EQ(ntfViews.size(), 1u);
    PointViewPtr ntfView = *ntfViews.begin();

    MetadataNode root = ntf_reader->getMetadata();
    MetadataNode n = root.findChild("FH.FTITLE");
    EXPECT_EQ(n.value(), "LiDAR from somewhere");
    n = root.findChild("FH.ONAME");
    EXPECT_EQ(n.value(), "Howard Butler");
    n = root.findChild("FH.OPHONE");
    EXPECT_EQ(n.value(), "5155554628");
    n = root.findChild([](MetadataNode& m)
        { return m.name() == "IM:0.IDATIM"; }
    );
    EXPECT_EQ(n.value(), "20110516183337");

    //
    // compare the two views
    //
    EXPECT_EQ(ntfView->size(), lasView->size());

    for (PointId i = 0; i < ntfView->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(ntfView->getFieldAs<double>(Dimension::Id::X, i),
            lasView->getFieldAs<double>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(ntfView->getFieldAs<double>(Dimension::Id::Y, i),
            lasView->getFieldAs<double>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(ntfView->getFieldAs<double>(Dimension::Id::Z, i),
            lasView->getFieldAs<double>(Dimension::Id::Z, i));
    }
}

} // Unnamed namespace

TEST(NitfWriterTest, test1)
{
    StageFactory f;

    const std::string las_input(Support::datapath("las/1.2-with-color.las"));
    const std::string nitf_output(Support::temppath("temp_nitf.ntf"));

    FileUtils::deleteFile(nitf_output);

    //
    // write the NITF
    //
    {
        Options reader_opts;
        Option reader_opt1("filename", las_input);
        reader_opts.add(reader_opt1);

        Options writer_opts;
        Option writer_opt1("filename", nitf_output);
        Option debug("debug", true);
        Option verbose("verbose", 8);

        Option datetime("idatim", "20110516183337");
        writer_opts.add(datetime);

        Option cls("fsclas", "S");
        writer_opts.add(cls);

        Option phone("ophone", "5155554628");
        writer_opts.add(phone);

        Option name("oname", "Howard Butler");
        writer_opts.add(name);

        Option ftitle("ftitle", "LiDAR from somewhere");
        writer_opts.add(ftitle);

        // writer_opts.add(debug);
        // writer_opts.add(verbose);
        writer_opts.add(writer_opt1);

        Stage* reader(f.createStage("readers.las"));
        EXPECT_TRUE(reader);
        reader->setOptions(reader_opts);

        Stage* writer(f.createStage("writers.nitf"));
        EXPECT_TRUE(writer);
        writer->setOptions(writer_opts);
        writer->setInput(*reader);
        {
            // writer.setCompressed(false);
            // // writer.setDate(0, 0);
            // // writer.setPointFormat(::drivers::las::PointFormat3);
            // // writer.setSystemIdentifier("");
            // writer.setGeneratingSoftware("PDAL-NITF");
            // writer.setChunkSize(100);
        }
        PointTable table;
        writer->prepare(table);
        writer->execute(table);
    }

    //
    // check the LAS contents against the source image
    //
    compare_contents(las_input, nitf_output);

    FileUtils::deleteFile(Support::temppath(nitf_output));
}

// Test that data from three input views gets written to separate output files.
TEST(NitfWriterTest, flex)
{
    StageFactory f;

    std::array<std::string, 3> outname =
        {{ "test_1.ntf", "test_2.ntf", "test_3.ntf" }};

    Options readerOps;
    readerOps.add("filename", Support::datapath("nitf/autzen-utm10.ntf"));

    PointTable table;

    Stage* reader(f.createStage("readers.nitf"));
    reader->setOptions(readerOps);

    reader->prepare(table);
    PointViewSet views = reader->execute(table);
    PointViewPtr v = *(views.begin());

    PointViewPtr v1(new PointView(table));
    PointViewPtr v2(new PointView(table));
    PointViewPtr v3(new PointView(table));

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (PointId i = 0; i < v->size(); ++i)
        vs[i % 3]->appendPoint(*v, i);

    for (size_t i = 0; i < outname.size(); ++i)
        FileUtils::deleteFile(Support::temppath(outname[i]));

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", Support::temppath("test_#.ntf"));

    Stage* writer(f.createStage("writers.nitf"));
    writer->setOptions(writerOps);
    writer->setInput(reader2);

    writer->prepare(table);
    writer->execute(table);

    for (size_t i = 0; i < outname.size(); ++i)
    {
        std::string filename = Support::temppath(outname[i]);
        EXPECT_TRUE(FileUtils::fileExists(filename));

        Options ops;
        ops.add("filename", filename);

        Stage* r(f.createStage("readers.nitf"));
        r->setOptions(ops);
        EXPECT_EQ(r->preview().m_pointCount, vs[i]->size());
    }
}


// Test that data from three input views gets written to a single output file.
TEST(NitfWriterTest, flex2)
{
    StageFactory f;

    Options readerOps;
    readerOps.add("filename", Support::datapath("nitf/autzen-utm10.ntf"));

    PointTable table;

    Stage* reader(f.createStage("readers.nitf"));
    reader->setOptions(readerOps);

    reader->prepare(table);
    PointViewSet views = reader->execute(table);
    PointViewPtr v = *(views.begin());

    PointViewPtr v1(new PointView(table));
    PointViewPtr v2(new PointView(table));
    PointViewPtr v3(new PointView(table));

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (PointId i = 0; i < v->size(); ++i)
        vs[i % 3]->appendPoint(*v, i);

    std::string outfile(Support::temppath("test_flex.ntf"));
    FileUtils::deleteFile(outfile);

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", outfile);

    Stage* writer(f.createStage("writers.nitf"));
    writer->setOptions(writerOps);
    writer->setInput(reader2);

    writer->prepare(table);
    writer->execute(table);

    EXPECT_TRUE(FileUtils::fileExists(outfile));

    Options ops;
    ops.add("filename", outfile);

    Stage* r(f.createStage("readers.nitf"));
    r->setOptions(ops);
    EXPECT_EQ(r->preview().m_pointCount, v->size());
}

TEST(NitfWriterTest, longFtitle)
{
    StageFactory f;

    Stage *r = f.createStage("readers.las");

    Options ro;
    ro.add("filename", Support::datapath("nitf/autzen-utm10.las"));

    r->setOptions(ro);

    Stage *w = f.createStage("writers.nitf");

    Options wo;
    wo.add("filename", "aaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbbbbbccccccccccccccccccccccddddddddddddddeeeeeeeeeeee");

    w->setOptions(wo);
    w->setInput(*r);

    PointTable t;
    w->prepare(t);
    EXPECT_THROW(w->execute(t), pdal_error);
}
