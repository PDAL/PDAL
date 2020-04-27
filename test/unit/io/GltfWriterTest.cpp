/******************************************************************************
 * Copyright (c) 2020, Emma Krantz <krantzemmaj@gmail.com>
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

#include <io/BufferReader.hpp>
#include <io/GltfWriter.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

namespace pdal
{


void testWrite(bool writeNormals, bool writeColors, std::string path)
{
    PointTable t;

    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);

    if (writeNormals) {
        t.layout()->registerDim(Dimension::Id::NormalX);
        t.layout()->registerDim(Dimension::Id::NormalY);
        t.layout()->registerDim(Dimension::Id::NormalZ);
    }

    if (writeColors) {
        t.layout()->registerDim(Dimension::Id::Red);
        t.layout()->registerDim(Dimension::Id::Blue);
        t.layout()->registerDim(Dimension::Id::Green);
    }

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


    if (writeNormals)
    {
        v->setField(Dimension::Id::NormalX, 0, 0);
        v->setField(Dimension::Id::NormalY, 0, 1);
        v->setField(Dimension::Id::NormalZ, 0, 0);

        v->setField(Dimension::Id::NormalX, 1, 0);
        v->setField(Dimension::Id::NormalY, 1, 1);
        v->setField(Dimension::Id::NormalZ, 1, 0);

        v->setField(Dimension::Id::NormalX, 2, 0);
        v->setField(Dimension::Id::NormalY, 2, 1);
        v->setField(Dimension::Id::NormalZ, 2, 0);

        v->setField(Dimension::Id::NormalX, 3, 1);
        v->setField(Dimension::Id::NormalY, 3, 0);
        v->setField(Dimension::Id::NormalZ, 3, 0);
    }

    if (writeColors)
    {
        v->setField(Dimension::Id::Red, 0, 255);
        v->setField(Dimension::Id::Blue, 0, 0);
        v->setField(Dimension::Id::Green, 0, 0);

        v->setField(Dimension::Id::Red, 1, 0);
        v->setField(Dimension::Id::Blue, 1, 255);
        v->setField(Dimension::Id::Green, 1, 0);

        v->setField(Dimension::Id::Red, 2, 0);
        v->setField(Dimension::Id::Blue, 2, 0);
        v->setField(Dimension::Id::Green, 2, 255);

        v->setField(Dimension::Id::Red, 3, 255);
        v->setField(Dimension::Id::Blue, 3, 0);
        v->setField(Dimension::Id::Green, 3, 0);
    }

    TriangularMesh* mesh = v->createMesh("foo");
    mesh->add(0, 1, 2);
    mesh->add(3, 2, 1);

    BufferReader r;
    r.addView(v);

    StageFactory factory;
    Stage *writer = factory.createStage("writers.gltf");

    Options writerOptions;
    writerOptions.add("filename", path);
    if (writeColors) {
        writerOptions.add("color_vertices", true);
    }
    writer->setOptions(writerOptions);
    writer->setInput(r);
    writer->prepare(t);
    writer->execute(t);
}


TEST(GltfWriter, Write)
{
    std::string path = Support::temppath("out.glb");
    testWrite(false, false, path);
    ASSERT_EQ(FileUtils::fileSize(path), 5100);
}


/**
TEST(GltfWriter, WriteWithNormals)
{
    std::string path = Support::temppath("out_normals.glb");
    testWrite(true, false, path);
    ASSERT_EQ(FileUtils::fileSize(path), 5148);
}


TEST(GltfWriter, WriteWithColors)
{
    std::string path = Support::temppath("out_colors.glb");
    testWrite(false, true, path);
    ASSERT_EQ(FileUtils::fileSize(path), 5148);
}


TEST(GltfWriter, WriteWithNormalsAndColors)
{
    std::string path = Support::temppath("out_normals_colors.glb");
    testWrite(true, true, path);
    ASSERT_EQ(FileUtils::fileSize(path), 5196);
}
**/

} // namespace pdal
