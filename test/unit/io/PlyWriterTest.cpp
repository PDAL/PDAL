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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <io/PlyReader.hpp>
#include <io/PlyWriter.hpp>
#include "Support.hpp"


namespace pdal
{


TEST(PlyWriter, Constructor)
{
    PlyWriter writer1;

    StageFactory f;
    Stage* writer2(f.createStage("writers.ply"));
    EXPECT_TRUE(writer2);
}


TEST(PlyWriter, Write)
{
    Options readerOptions;
    readerOptions.add("count", 750);
    readerOptions.add("mode", "random");
    FauxReader reader;
    reader.setOptions(readerOptions);

    Options writerOptions;
    writerOptions.add("filename", Support::temppath("out.ply"));
    PlyWriter writer;
    writer.setOptions(writerOptions);
    writer.setInput(reader);

    PointTable table;
    // Executing prepare twice tests #2791
    writer.prepare(table);
    writer.prepare(table);
    writer.execute(table);
}

void testMesh(const std::string referenceFile, int precision)
{
    std::string outfile(Support::temppath("out.ply"));
    std::string testfile(Support::datapath("ply/mesh.ply"));

    FileUtils::deleteFile(outfile);

    PointTable t;

    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v(new PointView(t));
    v->setField(Dimension::Id::X, 0, 1);
    v->setField(Dimension::Id::Y, 0, 1);
    v->setField(Dimension::Id::Z, 0, 0);

    v->setField(Dimension::Id::X, 1, 2);
    v->setField(Dimension::Id::Y, 1, 1);
    v->setField(Dimension::Id::Z, 1, 0);

    v->setField(Dimension::Id::X, 2, 1);
    v->setField(Dimension::Id::Y, 2, 2);
    v->setField(Dimension::Id::Z, 2, 0);

    v->setField(Dimension::Id::X, 3, 2);
    v->setField(Dimension::Id::Y, 3, 2);
    v->setField(Dimension::Id::Z, 3, 2);

    TriangularMesh *mesh = v->createMesh("foo");
    mesh->add(0, 1, 2);
    mesh->add(1, 2, 3);

    BufferReader r;
    r.addView(v);

    PlyWriter w;
    Options wo;
    wo.add("filename", outfile);
    wo.add("storage_mode", "ascii");
    wo.add("faces", true);
    if (precision >= 0)
        wo.add("precision", precision);
    w.setInput(r);
    w.setOptions(wo);

    w.prepare(t);
    w.execute(t);

    EXPECT_TRUE(Support::compare_text_files(outfile, referenceFile));
}


TEST(PlyWriter, mesh)
{
    testMesh(Support::datapath("ply/mesh.ply"), -1);
    testMesh(Support::datapath("ply/mesh_fixed.ply"), 3);
}

// Make sure that you can't use precision with ascii output.
TEST(PlyWriter, precisionException)
{
    std::string outfile(Support::temppath("out.ply"));
    FileUtils::deleteFile(outfile);

    Options readerOptions;
    readerOptions.add("count", 750);
    readerOptions.add("mode", "random");
    FauxReader reader;
    reader.setOptions(readerOptions);

    Options writerOptions;
    writerOptions.add("filename", outfile);
    writerOptions.add("storage_mode", "little endian");
    writerOptions.add("precision", 3);
    PlyWriter writer;
    writer.setOptions(writerOptions);
    writer.setInput(reader);

    PointTable table;
    EXPECT_THROW(writer.prepare(table), pdal_error);
}

// Make sure we don't write ints as floats.
TEST(PlyWriter, issue_2421)
{
    std::string outfile(Support::temppath("out.ply"));
    std::string referenceFile(Support::datapath("ply/issue_2421.ply"));

    FileUtils::deleteFile(outfile);

    PointTable t;

    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);
    Dimension::Id iid = t.layout()->assignDim("I", Dimension::Type::Signed32);

    PointViewPtr v(new PointView(t));
    v->setField(Dimension::Id::X, 0, 1.23456789012345);
    v->setField(Dimension::Id::Y, 0, 12345.6789012345);
    v->setField(Dimension::Id::Z, 0, 1234567890.12345);
    v->setField(iid, 0, 1234567890);

    BufferReader r;
    r.addView(v);

    PlyWriter w;
    Options wo;
    wo.add("filename", outfile);
    wo.add("storage_mode", "ascii");
    wo.add("precision", 5);
    w.setInput(r);
    w.setOptions(wo);

    w.prepare(t);
    w.execute(t);

    EXPECT_TRUE(Support::compare_text_files(outfile, referenceFile));
}

TEST(PlyWriter, dimtypes)
{
    std::string outfile(Support::temppath("out.ply"));

    PointTable t;

    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);
    Dimension::Id iid = t.layout()->assignDim("I", Dimension::Type::Double);
    Dimension::Id jid = t.layout()->assignDim("J", Dimension::Type::Double);

    PointViewPtr v(new PointView(t));
    v->setField(Dimension::Id::X, 0, 1.23456789012345);
    v->setField(Dimension::Id::Y, 0, 12345.6789012345);
    v->setField(Dimension::Id::Z, 0, 1234567890.12345);
    v->setField(iid, 0, 1234567890);
    v->setField(jid, 0, 12345);

    BufferReader r;
    r.addView(v);

    {
        PlyWriter w;
        Options wo;
        wo.add("filename", outfile);
        wo.add("storage_mode", "ascii");
        wo.add("precision", 5);
        wo.add("dims", "X=int8_t,Y=ushort,J=int,I=double");
        w.setInput(r);
        w.setOptions(wo);

        FileUtils::deleteFile(outfile);
        w.prepare(t);
        w.execute(t);

        std::string referenceFile(Support::datapath("ply/sized_dims.ply"));
        EXPECT_TRUE(Support::compare_text_files(outfile, referenceFile));
    }

    {
        PlyWriter w;
        Options wo;
        wo.add("filename", outfile);
        wo.add("storage_mode", "ascii");
        wo.add("precision", 5);
        wo.add("sized_types", false);
        wo.add("dims", "Y=ushort,X=int8_t,I,J=int");
        w.setInput(r);
        w.setOptions(wo);

        FileUtils::deleteFile(outfile);
        w.prepare(t);
        w.execute(t);

        std::string referenceFile(Support::datapath("ply/unsized_dims.ply"));
        EXPECT_TRUE(Support::compare_text_files(outfile, referenceFile));
    }
}

// Test that data from three input views gets written to separate output files.
TEST(PlyWriter, flex)
{
    std::array<std::string, 3> outname = {
        {"test_1.ply", "test_2.ply", "test_3.ply"}};

    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v1(new PointView(table));
    v1->setField(Dimension::Id::X, 0, 1);
    v1->setField(Dimension::Id::Y, 0, 1);
    v1->setField(Dimension::Id::Z, 0, 1);

    PointViewPtr v2(new PointView(table));
    v2->setField(Dimension::Id::X, 0, 1);
    v2->setField(Dimension::Id::Y, 0, 1);
    v2->setField(Dimension::Id::Z, 0, 2);

    v2->setField(Dimension::Id::X, 1, 2);
    v2->setField(Dimension::Id::Y, 1, 1);
    v2->setField(Dimension::Id::Z, 1, 2);

    PointViewPtr v3(new PointView(table));
    v3->setField(Dimension::Id::X, 0, 1);
    v3->setField(Dimension::Id::Y, 0, 1);
    v3->setField(Dimension::Id::Z, 0, 3);

    v3->setField(Dimension::Id::X, 1, 2);
    v3->setField(Dimension::Id::Y, 1, 1);
    v3->setField(Dimension::Id::Z, 1, 3);

    v3->setField(Dimension::Id::X, 2, 1);
    v3->setField(Dimension::Id::Y, 2, 2);
    v3->setField(Dimension::Id::Z, 2, 3);

    TriangularMesh* mesh = v3->createMesh("foo");
    mesh->add(0, 1, 2);

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (size_t i = 0; i < outname.size(); ++i)
        FileUtils::deleteFile(Support::temppath(outname[i]));

    BufferReader reader;
    reader.addView(v1);
    reader.addView(v2);
    reader.addView(v3);

    Options writerOps;
    writerOps.add("filename", Support::temppath("test_#.ply"));
    writerOps.add("faces", true);

    PlyWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    writer.prepare(table);
    writer.execute(table);

    for (size_t i = 0; i < outname.size(); ++i)
    {
        std::string filename = Support::temppath(outname[i]);
        EXPECT_TRUE(FileUtils::fileExists(filename));

        Options ops;
        ops.add("filename", filename);

        PlyReader r;
        r.setOptions(ops);

        PointTable t;
        r.prepare(t);
        PointViewSet viewSet = r.execute(t);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();
        EXPECT_EQ(view->size(), i+1);
    }
}

// Test that data from three input views gets written to a single output file.
TEST(PlyWriter, flex2)
{
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v1(new PointView(table));
    v1->setField(Dimension::Id::X, 0, 1);
    v1->setField(Dimension::Id::Y, 0, 1);
    v1->setField(Dimension::Id::Z, 0, 1);

    PointViewPtr v2(new PointView(table));
    v2->setField(Dimension::Id::X, 0, 1);
    v2->setField(Dimension::Id::Y, 0, 1);
    v2->setField(Dimension::Id::Z, 0, 2);

    v2->setField(Dimension::Id::X, 1, 2);
    v2->setField(Dimension::Id::Y, 1, 1);
    v2->setField(Dimension::Id::Z, 1, 2);

    PointViewPtr v3(new PointView(table));
    v3->setField(Dimension::Id::X, 0, 1);
    v3->setField(Dimension::Id::Y, 0, 1);
    v3->setField(Dimension::Id::Z, 0, 3);

    v3->setField(Dimension::Id::X, 1, 2);
    v3->setField(Dimension::Id::Y, 1, 1);
    v3->setField(Dimension::Id::Z, 1, 3);

    v3->setField(Dimension::Id::X, 2, 1);
    v3->setField(Dimension::Id::Y, 2, 2);
    v3->setField(Dimension::Id::Z, 2, 3);

    TriangularMesh* mesh = v3->createMesh("foo");
    mesh->add(0, 1, 2);

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    std::string outfile(Support::temppath("test_flex.ply"));
    FileUtils::deleteFile(outfile);

    BufferReader reader;
    reader.addView(v1);
    reader.addView(v2);
    reader.addView(v3);

    Options writerOps;
    writerOps.add("filename", outfile);
    writerOps.add("faces", true);

    PlyWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    writer.prepare(table);
    writer.execute(table);

    EXPECT_TRUE(FileUtils::fileExists(outfile));

    Options ops;
    ops.add("filename", outfile);

    PlyReader r;
    r.setOptions(ops);
    
    PointTable t;
    r.prepare(t);
    PointViewSet viewSet = r.execute(t);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 6u);
}
} // namespace pdal
