/******************************************************************************
 * Copyright (c) 2020, Bradley J. Chambers (brad.chambers@gmail.com)
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

#include <filters/PlaneFitFilter.hpp>
#include <io/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_test_main.hpp>

using namespace pdal;

TEST(PlaneFitFilterTest, BasicTest)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader br;
    PlaneFitFilter filter;
    Options opts;
    opts.add("knn", 4);
    filter.setInput(br);
    filter.setOptions(opts);
    filter.prepare(table);

    PointViewPtr src(new PointView(table));
    src->setField(Dimension::Id::X, 0, 0.0);
    src->setField(Dimension::Id::Y, 0, 0.0);
    src->setField(Dimension::Id::Z, 0, 1.0);
    src->setField(Dimension::Id::X, 1, -1.0);
    src->setField(Dimension::Id::Y, 1, 0.0);
    src->setField(Dimension::Id::Z, 1, 0.0);
    src->setField(Dimension::Id::X, 2, 1.0);
    src->setField(Dimension::Id::Y, 2, 0.0);
    src->setField(Dimension::Id::Z, 2, 0.0);
    src->setField(Dimension::Id::X, 3, 0.0);
    src->setField(Dimension::Id::Y, 3, -1.0);
    src->setField(Dimension::Id::Z, 3, 0.0);
    src->setField(Dimension::Id::X, 4, 0.0);
    src->setField(Dimension::Id::Y, 4, 1.0);
    src->setField(Dimension::Id::Z, 4, 0.0);

    br.addView(src);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Id planefit = table.layout()->findDim("PlaneFit");

    ASSERT_FLOAT_EQ(1.0f, outView->getFieldAs<float>(planefit, 0));
}
