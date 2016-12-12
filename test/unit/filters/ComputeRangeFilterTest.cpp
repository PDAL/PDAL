/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/util/FileUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <io/BufferReader.hpp>
#include <filters/ComputeRangeFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ComputeRangeFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.computerange"));
    EXPECT_TRUE(filter);
}

TEST(ComputeRangeFilterTest, compute)
{
    using namespace Dimension;

    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Id::X);
    layout->registerDim(Id::Y);
    layout->registerDim(Id::Z);
    Id pn = layout->registerOrAssignDim("Pixel Number", Type::Double);
    Id fn = layout->registerOrAssignDim("Frame Number", Type::Double);

    PointViewPtr view(new PointView(table));

    BufferReader r;
    r.addView(view);

    ComputeRangeFilter crop;
    crop.setInput(r);
    crop.prepare(table);

    view->setField(Id::X, 0, 0.0);
    view->setField(Id::Y, 0, 0.0);
    view->setField(Id::Z, 0, 0.0);
    view->setField(pn, 0, 0.0);
    view->setField(fn, 0, 0.0);

    view->setField(Id::X, 1, 0.0);
    view->setField(Id::Y, 1, 3.0);
    view->setField(Id::Z, 1, 4.0);
    view->setField(pn, 1, -5.0);
    view->setField(fn, 1, 0.0);

    PointViewSet s = crop.execute(table);
    EXPECT_EQ(1u, s.size());

    Id range = layout->findDim("Range");
    EXPECT_NE(Id::Unknown, range);

    PointViewPtr out = *s.begin();
    EXPECT_EQ(2u, out->size());

    EXPECT_EQ(5.0, out->getFieldAs<double>(range, 0));
    EXPECT_EQ(0.0, out->getFieldAs<double>(range, 1));
}
