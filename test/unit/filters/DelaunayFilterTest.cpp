/******************************************************************************
* Copyright (c) 2018, Bradley J Chambers (brad.chambers@gmail.com)
* Copyright (c) 2019, Peter L. Svendsen (peter.limkilde@gmail.com)
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

#include <pdal/PointView.hpp>
#include <io/TextReader.hpp>
#include <filters/DelaunayFilter.hpp>

#include <vector>
#include <algorithm> // for rotate_copy

#include "Support.hpp"

using namespace pdal;

TEST(DelaunayFilterTest, test1)
{
    // The triangles we expect from the Delaunay triangulation of the
    // input file. For each of these triangles, we will check for all
    // index permutations, as long as the vertices appear in
    // counterclockwise order.
    std::vector<Triangle> expectedTriangles =
    {
        {5, 2, 0},
        {2, 5, 4},
        {3, 2, 4},
        {2, 1, 0},
        {3, 1, 2}
    };

    // Number of detected occurrences of each of the expected triangles.
    // Initialize to zeros.
    std::vector<int> expectedTrianglesOccurrences(expectedTriangles.size(), 0);

    Options readerOps;
    readerOps.add("filename",
        Support::datapath("filters/delaunaytest.txt"));

    TextReader reader;
    reader.setOptions(readerOps);

    DelaunayFilter filter;
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 6u);
    TriangularMesh *mesh = view->mesh("delaunay2d");
    EXPECT_EQ(mesh->size(), expectedTriangles.size());

    // Loop through the triangles of the generated mesh...
    for (size_t i = 0; i < mesh->size(); i++)
    {
        Triangle triangle = (*mesh)[i];

        auto it = std::find(expectedTriangles.begin(), expectedTriangles.end(),
            triangle);
        bool found = (it != expectedTriangles.end());
        EXPECT_TRUE(found);
        expectedTriangles.erase(it);
    }
    EXPECT_EQ(expectedTriangles.size(), (size_t)0);
}

