/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <filters/NNDistanceFilter.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(NNDistanceTest, kdist)
{
    StageFactory f;
    Stage *reader(f.createStage("readers.faux"));
    Stage *filter(f.createStage("filters.nndistance"));

    // Make a 10x10x10 grid of points.
    Options rOpts;
    rOpts.add("mode", "grid");
    rOpts.add("bounds", "([0, 10],[0,10],[0,10])");
    rOpts.add("count", 1000);
    reader->setOptions(rOpts);

    // Kth distance with k == 3
    Options fOpts;
    fOpts.add("mode", "kth");
    fOpts.add("k", 3);
    filter->setOptions(fOpts);
    filter->setInput(*reader);
    
    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);
    PointViewPtr v = *(s.begin());

    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<double>(Dimension::Id::NNDistance, i), 1);

    // Kth distance with k == 4
    Options opts2;
    opts2.add("k", 4);
    filter->setOptions(opts2);

    PointTable t2;
    filter->prepare(t2);
    s = filter->execute(t2);
    v = *(s.begin());

    for (PointId i = 0; i < v->size(); ++i)
    {
        double d = v->getFieldAs<double>(Dimension::Id::NNDistance, i);
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        double y = v->getFieldAs<double>(Dimension::Id::Y, i);
        double z = v->getFieldAs<double>(Dimension::Id::Z, i);
        if ((x == 9 || x == 0) && (y == 9 || y == 0) && (z == 9 || z == 0))
            EXPECT_EQ(d, sqrt(2));
    }

    // Test for avg distance.
    Options opts3;
    opts3.add("mode", "avg");
    opts3.add("k", 6);
    filter->setOptions(opts3);

    PointTable t3;
    filter->prepare(t3);
    s = filter->execute(t3);
    v = *(s.begin());

    for (PointId i = 0; i < v->size(); ++i)
    {
        double d = v->getFieldAs<double>(Dimension::Id::NNDistance, i);
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        double y = v->getFieldAs<double>(Dimension::Id::Y, i);
        double z = v->getFieldAs<double>(Dimension::Id::Z, i);
        int xe = (x == 9 || x == 0) ? 1 : 0;
        int ye = (y == 9 || y == 0) ? 1 : 0;
        int ze = (z == 9 || z == 0) ? 1 : 0;
        bool corner = (xe + ye + ze == 3);
        bool edge = (xe + ye + ze == 2);
        bool face = (xe + ye + ze == 1);
        if (corner)
            EXPECT_EQ(d, (1 + std::sqrt(2)) / 2.0);
        else if (edge)
            EXPECT_EQ(d, (2 + std::sqrt(2)) / 3.0);
        else if (face)
            EXPECT_EQ(d, (5 + std::sqrt(2)) / 6.0);
        else
            EXPECT_EQ(d, 1);
    }
}

