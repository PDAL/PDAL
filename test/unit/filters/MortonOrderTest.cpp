/******************************************************************************
* Copyright (c) 2018, Paul Blottiere (blottiere.paul@gmail.com)
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
#include <filters/MortonOrderFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(MortonOrderTest, test_code)
{
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);

    PointViewPtr view(new PointView(table));

    int n = 0;
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            view->setField(Dimension::Id::X, n, i);
            view->setField(Dimension::Id::Y, n, j);
            n += 1;
        }
    }

    BufferReader r;
    r.addView(view);

    MortonOrderFilter filter;
    Options o;
    o.add("reverse", "true");
    filter.setInput(r);
    filter.setOptions(o);

    filter.prepare(table);
    PointViewSet s = filter.execute(table);
    PointViewPtr outView = *s.begin();

    /*
     ___________________
    |    |    |    |    |
    | 0  | 9  | 6  | 2  |
    |____|____|____|____|
    |    |    |    |    |
    | 12 | 15 | 11 | 10 |
    |____|____|____|____|
    |    |    |    |    |
    | 4  | 14 | 8  | 5  |
    |____|____|____|____|
    |    |    |    |    |
    | 1  | 13 | 7  | 3  |
    |____|____|____|____|
    */

    // index 0
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 0), 0);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 0), 0);

    // index 1
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 1), 0);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 1), 3);

    // index 2
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 2), 3);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 2), 0);

    // index 3
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 3), 3);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 3), 3);

    // index 4
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 4), 0);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 4), 2);

    // index 5
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::X, 5), 3);
    EXPECT_EQ(outView->getFieldAs<double>(Dimension::Id::Y, 5), 2);
}
