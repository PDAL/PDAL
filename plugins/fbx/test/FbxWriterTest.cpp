/******************************************************************************
* Copyright (c) 2018, Hobu Inc. <info@hobu.co>
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
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(FbxWriter, test1)
{
    std::string output(Support::temppath("out.fbx"));

    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    PointViewPtr view(new PointView(table));

    // Create points.
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
        {
            PointId id = (10 * i) + j;
            view->setField(Dimension::Id::X, id, i);
            view->setField(Dimension::Id::Y, id, j);
        }

    TriangularMesh *mesh = view->createMesh("test");
    // Create faces.
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
        {
            // This partitions grid into triangles, each grid square is
            // partitioned by an edge from upper right to lower left.
            PointId id = (10 * i) + j;
            mesh->add(id, id + 10, id + 11);
            mesh->add(id, id + 1, id + 11);
        }

    StageFactory f;

    BufferReader reader;
    reader.addView(view);

    Stage *fbx = f.createStage("writers.fbx");
    Options fbxOptions;
    fbxOptions.add("filename", output);
    fbxOptions.add("ascii", true);
    fbx->addOptions(fbxOptions);
    fbx->setInput(reader);

    FileUtils::deleteFile(output);
    fbx->prepare(table);
    fbx->execute(table);

    std::string out = FileUtils::readFileIntoString(output);
    std::string vertices = "a: 0,0,0,0,1,0,0,2,0,0,3,0,0,4,0,0,5,0,0,6,0,0,7,0,0,8,0,0,9,0,1,0,0,1,1,0,1,2,0,1,3,0,1,4,0,1,5,0,1,6,0,1,7,0,1,8,0,1,9,0,2,0,0,2,1,0,2,2,0,2,3,0,2,4,0,2,5,0,2,6,0,2,7,0,2,8,0,2,9,0,3,0,0,3,1,0,3,2,0,3,3,0,3,4,0,3,5,0,3,6,0,3,7,0,3,8,0,3,9,0,4,0,0,4,1,0,4,2,0,4,3,0,4,4,0,4,5,0,4,6,0,4,7,0,4,8,0,4,9,0,5,0,0,5,1,0,5,2,0,5,3,0,5,4,0,5,5,0,5,6,0,5,7,0,5,8,0,5,9,0,6,0,0,6,1,0,6,2,0,6,3,0,6,4,0,6,5,0,6,6,0,6,7,0,6,8,0,6,9,0,7,0,0,7,1,0,7,2,0,7,3,0,7,4,0,7,5,0,7,6,0,7,7,0,7,8,0,7,9,0,8,0,0,8,1,0,8,2,0,8,3,0,8,4,0,8,5,0,8,6,0,8,7,0,8,8,0,8,9,0,9,0,0,9,1,0,9,2,0,9,3,0,9,4,0,9,5,0,9,6,0,9,7,0,9,8,0,9,9,0";

    std::string polygons = "a: 0,10,-12,0,1,-12,1,11,-13,1,2,-13,2,12,-14,2,3,-14,3,13,-15,3,4,-15,4,14,-16,4,5,-16,5,15,-17,5,6,-17,6,16,-18,6,7,-18,7,17,-19,7,8,-19,8,18,-20,8,9,-20,10,20,-22,10,11,-22,11,21,-23,11,12,-23,12,22,-24,12,13,-24,13,23,-25,13,14,-25,14,24,-26,14,15,-26,15,25,-27,15,16,-27,16,26,-28,16,17,-28,17,27,-29,17,18,-29,18,28,-30,18,19,-30,20,30,-32,20,21,-32,21,31,-33,21,22,-33,22,32,-34,22,23,-34,23,33,-35,23,24,-35,24,34,-36,24,25,-36,25,35,-37,25,26,-37,26,36,-38,26,27,-38,27,37,-39,27,28,-39,28,38,-40,28,29,-40,30,40,-42,30,31,-42,31,41,-43,31,32,-43,32,42,-44,32,33,-44,33,43,-45,33,34,-45,34,44,-46,34,35,-46,35,45,-47,35,36,-47,36,46,-48,36,37,-48,37,47,-49,37,38,-49,38,48,-50,38,39,-50,40,50,-52,40,41,-52,41,51,-53,41,42,-53,42,52,-54,42,43,-54,43,53,-55,43,44,-55,44,54,-56,44,45,-56,45,55,-57,45,46,-57,46,56,-58,46,47,-58,47,57,-59,47,48,-59,48,58,-60,48,49,-60,50,60,-62,50,51,-62,51,61,-63,51,52,-63,52,62,-64,52,53,-64,53,63,-65,53,54,-65,54,64,-66,54,55,-66,55,65,-67,55,56,-67,56,66,-68,56,57,-68,57,67,-69,57,58,-69,58,68,-70,58,59,-70,60,70,-72,60,61,-72,61,71,-73,61,62,-73,62,72,-74,62,63,-74,63,73,-75,63,64,-75,64,74,-76,64,65,-76,65,75,-77,65,66,-77,66,76,-78,66,67,-78,67,77,-79,67,68,-79,68,78,-80,68,69,-80,70,80,-82,70,71,-82,71,81,-83,71,72,-83,72,82,-84,72,73,-84,73,83,-85,73,74,-85,74,84,-86,74,75,-86,75,85,-87,75,76,-87,76,86,-88,76,77,-88,77,87,-89,77,78,-89,78,88,-90,78,79,-90,80,90,-92,80,81,-92,81,91,-93,81,82,-93,82,92,-94,82,83,-94,83,93,-95,83,84,-95,84,94,-96,84,85,-96,85,95,-97,85,86,-97,86,96,-98,86,87,-98,87,97,-99,87,88,-99,88,98,-100,88,89,-100";

    EXPECT_NE(out.find(vertices), std::string::npos);
    EXPECT_NE(out.find(polygons), std::string::npos);
}

}
