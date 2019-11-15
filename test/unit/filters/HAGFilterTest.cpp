/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "Support.hpp"

// NOTE: The test data has an accompanying jpg that depicts the points,
//  their triangulation and the interesting barycentric calculation.

namespace pdal
{

TEST(HAGFilterTest, delaunay)
{
    Options ro;
    ro.add("filename", Support::datapath("filters/hagtest.txt"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    Options fo;
    fo.add("count", 10);
    fo.add("delaunay", true);
    Stage& f = *(factory.createStage("filters.hag"));
    f.setInput(r);
    f.setOptions(fo);

    PointTable t1;
    f.prepare(t1);
    PointViewSet s = f.execute(t1);
    PointViewPtr v = *s.begin();

    for (PointId i = 0; i < v->size(); ++i)
    {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        double y = v->getFieldAs<double>(Dimension::Id::Y, i);
        double z = v->getFieldAs<double>(Dimension::Id::Z, i);
        double hag = v->getFieldAs<double>(Dimension::Id::HeightAboveGround, i);
        int c = v->getFieldAs<int>(Dimension::Id::Classification, i);
        if (c == 2)
            EXPECT_EQ(hag, 0);
        if (i == 0)
        {
            EXPECT_EQ(x, -2);
            EXPECT_EQ(y, 4);
            EXPECT_EQ(z, 20);
            EXPECT_EQ(hag, 10);
        }
        if (i == 1)
        {
            EXPECT_EQ(x, 4);
            EXPECT_EQ(y, 1);
            EXPECT_EQ(z, 20);
            EXPECT_EQ(hag, 11);
        }
        if (i == 2)
        {
            EXPECT_EQ(x, 2);
            EXPECT_EQ(y, 3);
            EXPECT_EQ(z, 20);
            EXPECT_EQ(hag, 14);
        }
        if (i == 3)
        {
            EXPECT_EQ(x, 4);
            EXPECT_EQ(y, 4);
            EXPECT_EQ(z, 20);
            EXPECT_EQ(hag, 16);
        }
    }
}

} // namespace pdal
