/******************************************************************************
 * Copyright (c) 2020 Bradley J Chambers (brad.chambers@gmail.com)
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
 *     * Neither the name of Helix Re Inc. nor the
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

#include <filters/NormalFilter.hpp>
#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <pdal/PointView.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(NormalFilterTest, XYPlane)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    FauxReader reader;
    Options readerOps;
    readerOps.add("mode", "grid");
    readerOps.add("bounds", "([0, 2], [0, 2], [0, 0])");
    readerOps.add("count", 4);
    reader.setOptions(readerOps);
    NormalFilter filter;
    Options filterOps;
    filterOps.add("knn", 3);
    filter.setInput(reader);
    filter.setOptions(filterOps);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Dimension::Id nx = table.layout()->findDim("NormalX");
    Dimension::Id ny = table.layout()->findDim("NormalY");
    Dimension::Id nz = table.layout()->findDim("NormalZ");
    Dimension::Id c = table.layout()->findDim("Curvature");

    for (auto const& p : *outView)
    {
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nx), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(ny), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nz), 1.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(c), 0.0);
    }
}

TEST(NormalFilterTest, XZPlane)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    FauxReader reader;
    Options readerOps;
    readerOps.add("mode", "grid");
    readerOps.add("bounds", "([0, 2], [0, 0], [0, 2])");
    readerOps.add("count", 4);
    reader.setOptions(readerOps);
    NormalFilter filter;
    Options filterOps;
    filterOps.add("knn", 3);
    filter.setInput(reader);
    filter.setOptions(filterOps);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Dimension::Id nx = table.layout()->findDim("NormalX");
    Dimension::Id ny = table.layout()->findDim("NormalY");
    Dimension::Id nz = table.layout()->findDim("NormalZ");
    Dimension::Id c = table.layout()->findDim("Curvature");

    for (auto const& p : *outView)
    {
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nx), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(ny), 1.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nz), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(c), 0.0);
    }
}

TEST(NormalFilterTest, YZPlane)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    FauxReader reader;
    Options readerOps;
    readerOps.add("mode", "grid");
    readerOps.add("bounds", "([0, 0], [0, 2], [0, 2])");
    readerOps.add("count", 4);
    reader.setOptions(readerOps);
    NormalFilter filter;
    Options filterOps;
    filterOps.add("knn", 3);
    filter.setInput(reader);
    filter.setOptions(filterOps);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Dimension::Id nx = table.layout()->findDim("NormalX");
    Dimension::Id ny = table.layout()->findDim("NormalY");
    Dimension::Id nz = table.layout()->findDim("NormalZ");
    Dimension::Id c = table.layout()->findDim("Curvature");

    for (auto const& p : *outView)
    {
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nx), 1.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(ny), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nz), 0.0);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(c), 0.0);
    }
}

TEST(NormalFilterTest, RampPlane)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader;
    NormalFilter filter;
    Options filterOps;
    filterOps.add("knn", 3);
    filter.setInput(reader);
    filter.setOptions(filterOps);
    filter.prepare(table);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 0);
    view->setField(Id::X, 1, 0);
    view->setField(Id::X, 2, 1);
    view->setField(Id::X, 3, 1);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 1);
    view->setField(Id::Y, 2, 0);
    view->setField(Id::Y, 3, 1);
    view->setField(Id::Z, 0, 0);
    view->setField(Id::Z, 1, 0);
    view->setField(Id::Z, 2, 1);
    view->setField(Id::Z, 3, 1);
    reader.addView(view);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Dimension::Id nx = table.layout()->findDim("NormalX");
    Dimension::Id ny = table.layout()->findDim("NormalY");
    Dimension::Id nz = table.layout()->findDim("NormalZ");
    Dimension::Id c = table.layout()->findDim("Curvature");

    double expected = std::sqrt(2.0) / 2.0;
    for (auto const& p : *outView)
    {
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nx), (float)-expected);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(ny), 0.0f);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nz), (float)expected);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(c), 0.0f);
    }
}

TEST(NormalFilterTest, RampPlane2)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader;
    NormalFilter filter;
    Options filterOps;
    filterOps.add("knn", 3);
    filter.setInput(reader);
    filter.setOptions(filterOps);
    filter.prepare(table);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 0);
    view->setField(Id::X, 1, 0);
    view->setField(Id::X, 2, 1);
    view->setField(Id::X, 3, 1);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 1);
    view->setField(Id::Y, 2, 0);
    view->setField(Id::Y, 3, 1);
    view->setField(Id::Z, 0, 0);
    view->setField(Id::Z, 1, 1);
    view->setField(Id::Z, 2, 0);
    view->setField(Id::Z, 3, 1);
    reader.addView(view);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    Dimension::Id nx = table.layout()->findDim("NormalX");
    Dimension::Id ny = table.layout()->findDim("NormalY");
    Dimension::Id nz = table.layout()->findDim("NormalZ");
    Dimension::Id c = table.layout()->findDim("Curvature");

    double expected = std::sqrt(2.0) / 2.0;
    for (auto const& p : *outView)
    {
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nx), 0.0f);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(ny), (float)-expected);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(nz), (float)expected);
        ASSERT_FLOAT_EQ(p.getFieldAs<float>(c), 0.0f);
    }
}

} // namespace pdal
