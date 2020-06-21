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

} // namespace pdal
