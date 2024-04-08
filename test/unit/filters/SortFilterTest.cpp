/******************************************************************************
* Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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

#include <random>

#include <pdal/PipelineManager.hpp>
#include <pdal/StageWrapper.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <filters/SortFilter.hpp>
#include "Support.hpp"

PDAL_DLL void setPdalView(pdal::PointView *);

using namespace pdal;

namespace
{

void doSort(point_count_t count, Dimension::Id dim,
    const std::string & order="")
{
    Options opts;

    opts.add("dimension", Dimension::name(dim));
    if (!order.empty())
        opts.add("order", order);

    SortFilter filter;
    filter.setOptions(opts);

    PointTable table;
    PointViewPtr view(new PointView(table));

    table.layout()->registerDim(dim);
    table.finalize();

    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist(0.0, (double)count);

    for (PointId i = 0; i < count; ++i)
        view->setField(dim, i, dist(generator));

    filter.prepare(table);
    FilterWrapper::ready(filter, table);
    FilterWrapper::filter(filter, *view.get());
    FilterWrapper::done(filter, table);

    EXPECT_EQ(count, view->size());
    for (PointId i = 1; i < count; ++i)
    {
        double d1 = view->getFieldAs<double>(dim, i - 1);
        double d2 = view->getFieldAs<double>(dim, i);
        if (order.empty() || order == "ASC")
            EXPECT_TRUE(d1 <= d2);
        else // DES(cending)
            EXPECT_TRUE(d1 >= d2);
    }
}

} // unnamed namespace

TEST(SortFilterTest, simple)
{
    // note that this also tests default sort order ASC /**
    for (point_count_t count = 3; count < 30; count++)
        doSort(count, Dimension::Id::X);
}

TEST(SortFilterTest, partial)
{
    const Dimension::Id dim = Dimension::Id::X;
    Options opts;

    opts.add("dimension", Dimension::name(dim));

    SortFilter filter;
    filter.setOptions(opts);

    PointTable table;
    PointViewPtr full(new PointView(table));

    table.layout()->registerDim(dim);
    table.finalize();

    const point_count_t count = 10;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist(0.0, (double)count * 2);

    for (PointId i = 0; i < count * 2; ++i)
        full->setField(dim, i, dist(generator));

    PointViewPtr view = full->makeNew();

    for (PointId i = 0; i < full->size(); ++i)
        if (i % 2 == 0) view->appendPoint(*full, i);
    ASSERT_EQ(count, view->size());

    filter.prepare(table);
    FilterWrapper::ready(filter, table);
    FilterWrapper::filter(filter, *view.get());
    FilterWrapper::done(filter, table);

    EXPECT_EQ(count, view->size());
    for (PointId i = 1; i < count; ++i)
    {
        double d1 = view->getFieldAs<double>(dim, i - 1);
        double d2 = view->getFieldAs<double>(dim, i);
        EXPECT_TRUE(d1 <= d2);
    }
}

TEST(SortFilterTest, testUnknownOptions)
{
    EXPECT_THROW(doSort(1, Dimension::Id::X, "not an order"), std::exception);
}

TEST(SortFilterTest, pipelineJSON)
{
    PipelineManager mgr;

    mgr.readPipeline(Support::configuredpath("filters/sort.json"));
    mgr.execute();

    PointViewSet viewSet = mgr.views();

    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    for (PointId i = 1; i < view->size(); ++i)
    {
        double d1 = view->getFieldAs<double>(Dimension::Id::X, i - 1);
        double d2 = view->getFieldAs<double>(Dimension::Id::X, i);
        EXPECT_TRUE(d1 <= d2);
    }
}

TEST(SortFilterTest, issue1382)
{
    LasReader r;
    Options ro;

    ro.add("filename", Support::datapath("autzen/autzen-utm.las"));
    r.setOptions(ro);

    SortFilter f;
    Options fo;

    fo.add("dimension", "Z");
    f.setOptions(fo);
    f.setInput(r);

    LasWriter w;
    Options wo;

    wo.add("filename", Support::temppath("out.las"));
    w.setOptions(wo);
    w.setInput(f);

    PointTable t;

    w.prepare(t);
    PointViewSet s = w.execute(t);
    PointViewPtr v = *s.begin();

    for (PointId i = 1; i < v->size(); ++i)
    {
        double d1 = v->getFieldAs<double>(Dimension::Id::Z, i - 1);
        double d2 = v->getFieldAs<double>(Dimension::Id::Z, i);
        EXPECT_TRUE(d1 <= d2);
    }
}

TEST(SortFilterTest, issue1121_simpleSortOrderDesc)
{
//        doSort(7, Dimension::Id::X, "DESC");
    point_count_t inc = 1;
    for (point_count_t count = 3; count < 100000; count += inc, inc *= 2)
    {
        doSort(count, Dimension::Id::X, "ASC");
        doSort(count, Dimension::Id::X, "DESC");
    }
}

/**
TEST(SortFilterTest, t1)
{
    PointTable t;
    PointView v(t);
    PointLayoutPtr l = v.layout();
    l->registerDims({ Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z });
    t.finalize();

    v.setField(Dimension::Id::X, 0, 10);
    v.setField(Dimension::Id::X, 1, 3);
    v.setField(Dimension::Id::X, 2, 5);
    v.setField(Dimension::Id::X, 3, 7);
    v.setField(Dimension::Id::X, 4, -5);
    v.setField(Dimension::Id::X, 5, 2);

    auto cmp = [](const PointRef& p1, const PointRef& p2)
    {
        return p1.compare(Dimension::Id::X, p2);
    };

    setPdalView(&v);

    std::sort(v.begin(), v.end(), cmp);

    setPdalView(nullptr);

    for (PointId i = 0; i < v.size(); ++i)
        std::cerr << "X = " << v.getFieldAs<double>(Dimension::Id::X, i) << "\n";
}

TEST(SortFilterTest, t2)
{
    PointTable t;
    PointView v(t);
    PointLayoutPtr l = v.layout();
    l->registerDims({ Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z });
    t.finalize();

    v.setField(Dimension::Id::X, 0, 'A');
    v.setField(Dimension::Id::X, 1, 'G');
    v.setField(Dimension::Id::X, 2, 'B');
    v.setField(Dimension::Id::X, 3, 'C');
    v.setField(Dimension::Id::X, 4, 'E');
    v.setField(Dimension::Id::X, 5, 'D');
    v.setField(Dimension::Id::X, 6, 'F');

    auto cmp = [](const PointRef& p1, const PointRef& p2)
    {
        return p1.compare(Dimension::Id::X, p2);
    };

    setPdalView(&v);

    std::sort(v.begin(), v.end(), cmp);

    setPdalView(nullptr);

    for (PointId i = 0; i < v.size(); ++i)
        std::cerr << "X = " << v.getFieldAs<char>(Dimension::Id::X, i) << "\n";
    for (PointId i = 1; i < v.size(); ++i)
        EXPECT_LT(std::cerr << "X = " << v.getFieldAs<char>(Dimension::Id::X, i) << "\n";
}
**/

