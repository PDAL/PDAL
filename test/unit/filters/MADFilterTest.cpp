/******************************************************************************
* Copyright (c) 2026, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <filters/MADFilter.hpp>
#include <io/BufferReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(MADFilterTest, defaultMultiplier)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    Options opts;
    opts.add("dimension", "Z");

    BufferReader reader;
    MADFilter filter;
    filter.setInput(reader);
    filter.setOptions(opts);
    filter.prepare(table);

    // Median is 0 and median absolute deviation is 1, so with k = 2 points
    // are kept when |Z| < 2 * 1.4826 = 2.9652.
    std::vector<double> z { -1, -1, -0.5, 0, 1, 1, 2.97 };
    PointViewPtr view(new PointView(table));
    for (PointId i = 0; i < z.size(); ++i)
        view->setField(Id::Z, i, z[i]);
    reader.addView(view);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();
    EXPECT_EQ(outView->size(), 6u);
    for (PointId i = 0; i < outView->size(); ++i)
        EXPECT_LT(outView->getFieldAs<double>(Id::Z, i), 2.97);
}
