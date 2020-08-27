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

#include <pdal/Filter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>

#include <io/ObjReader.hpp>
#include "Support.hpp"

namespace pdal
{


void checkPoint(const PointViewPtr& view, point_count_t idx,
        double x, double y, double z)
{
    EXPECT_DOUBLE_EQ(x, view->getFieldAs<double>(Dimension::Id::X, idx));
    EXPECT_DOUBLE_EQ(y, view->getFieldAs<double>(Dimension::Id::Y, idx));
    EXPECT_DOUBLE_EQ(z, view->getFieldAs<double>(Dimension::Id::Z, idx));
}

void checkPoint(const PointViewPtr& view, point_count_t idx,
        double x, double y, double z,
        double tu, double tv, double tw,
        double nx, double ny, double nz)
{
    EXPECT_DOUBLE_EQ(x, view->getFieldAs<double>(Dimension::Id::X, idx));
    EXPECT_DOUBLE_EQ(y, view->getFieldAs<double>(Dimension::Id::Y, idx));
    EXPECT_DOUBLE_EQ(z, view->getFieldAs<double>(Dimension::Id::Z, idx));
    EXPECT_DOUBLE_EQ(nx, view->getFieldAs<double>(Dimension::Id::NormalX, idx));
    EXPECT_DOUBLE_EQ(ny, view->getFieldAs<double>(Dimension::Id::NormalY, idx));
    EXPECT_DOUBLE_EQ(nz, view->getFieldAs<double>(Dimension::Id::NormalZ, idx));
    EXPECT_DOUBLE_EQ(tu, view->getFieldAs<double>(Dimension::Id::TextureU, idx));
    EXPECT_DOUBLE_EQ(tv, view->getFieldAs<double>(Dimension::Id::TextureV, idx));
    EXPECT_DOUBLE_EQ(tw, view->getFieldAs<double>(Dimension::Id::TextureW, idx));
}


TEST(ObjReader, Constructor)
{
    ObjReader reader1;

    StageFactory f;
    Stage* reader2(f.createStage("readers.obj"));
    EXPECT_TRUE(reader2);
}

TEST(ObjReader, NoFace)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/simple_binary.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 0u);
}

TEST(ObjReader, NoVertex)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/no_vertex.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 0u);

}

TEST(ObjReader, Read)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/box.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 8u);

    // Test mesh size, which will let us know if triangulation worked
    EXPECT_EQ(view->mesh("obj")->size(), 12u);

    // Order isn't garanteed
    checkPoint(view, 0, -0.5,  0.5,  0.5);
    checkPoint(view, 1, -0.5,  0.5, -0.5);
    checkPoint(view, 2, -0.5, -0.5, -0.5);
}

TEST(ObjReader, FourDimensionRead)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/hyper_box.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 16u);

    // Test mesh size, which will let us know if triangulation worked
    EXPECT_EQ(view->mesh("obj")->size(), 24u);

    // Order isn't garanteed
    checkPoint(view, 0, -0.5,  0.5,  0.5);
    EXPECT_DOUBLE_EQ(0.5, view->getFieldAs<double>(Dimension::Id::W, 0)); 
    checkPoint(view, 1, -0.5,  0.5, -0.5);
    EXPECT_DOUBLE_EQ(0.5, view->getFieldAs<double>(Dimension::Id::W, 1)); 
    checkPoint(view, 2, -0.5, -0.5, -0.5);
    EXPECT_DOUBLE_EQ(0.5, view->getFieldAs<double>(Dimension::Id::W, 2)); 
}
 
TEST(ObjReader, TexturesAndNormals)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/box_texture.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 8u);

    // Test mesh size, which will let us know if triangulation worked
    EXPECT_EQ(view->mesh("obj")->size(), 12u);

    // Order isn't garanteed
    checkPoint(view, 0, -0.5,  0.5,  0.5, -0.5,  0.5, 0, -0.5,  0.5,  0.5);
    checkPoint(view, 1, -0.5,  0.5, -0.5, -0.5,  0.5, 0, -0.5,  0.5, -0.5);
    checkPoint(view, 2, -0.5, -0.5, -0.5, -0.5, -0.5, 0, -0.5, -0.5, -0.5);
}

TEST(ObjReader, LargeFile)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/1.2-with-color.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 911u);
    // keep in mind they can be out-of-order
    checkPoint(view, 0, 637342.0l, 849088.1875l, 411.579987l);
    checkPoint(view, 1, 637337.75l, 848956.25l, 422.540009l);
    checkPoint(view, 2, 637257.0625l, 849161.75l, 411.089996l);
}

}
