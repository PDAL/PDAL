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
    writer.prepare(table);
    writer.execute(table);
}

TEST(PlyWriter, mesh)
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
    w.setInput(r);
    w.setOptions(wo);

    w.prepare(t);
    w.execute(t);

    EXPECT_TRUE(Support::compare_text_files(outfile, testfile));
}

}
