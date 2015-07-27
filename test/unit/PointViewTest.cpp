/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
* OF USE, view, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/pdal_test_main.hpp>

#include <random>

#include <boost/property_tree/xml_parser.hpp>

#include <pdal/PointView.hpp>
#include <pdal/PointViewIter.hpp>
#include <pdal/PDALUtils.hpp>
#include "Support.hpp"

using namespace pdal;

PointViewPtr makeTestView(PointTableRef table, point_count_t cnt = 17)
{
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::Classification);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    PointViewPtr view(new PointView(table));

    // write the data into the view
    for (PointId i = 0; i < cnt; i++)
    {
        const uint8_t x = (uint8_t)(i + 1);
        const int32_t y = i * 10;
        const double z = i * 100;

        view->setField(Dimension::Id::Classification, i, x);
        view->setField(Dimension::Id::X, i, y);
        view->setField(Dimension::Id::Y, i, z);
    }
    EXPECT_EQ(view->size(), cnt);
    return view;
}


void verifyTestView(const PointView& view, point_count_t cnt = 17)
{
    // read the view back out
    for (PointId i = 0; i < cnt; i++)
    {
        uint8_t x = view.getFieldAs<uint8_t>(
            Dimension::Id::Classification, i);
        int32_t y = view.getFieldAs<uint32_t>(Dimension::Id::X, i);
        double z = view.getFieldAs<double>(Dimension::Id::Y, i);

        EXPECT_EQ(x, (uint8_t)(i + 1));
        EXPECT_EQ(y, (int32_t)(i * 10));
        EXPECT_TRUE(Utils::compare_approx(z, static_cast<double>(i) * 100.0,
            (std::numeric_limits<double>::min)()));
    }
}

TEST(PointViewTest, getSet)
{
    PointTable table;
    PointViewPtr view = makeTestView(table, 1);
    verifyTestView(*view.get(), 1);
}

TEST(PointViewTest, getAsUint8)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    // read the view back out
    for (int i = 0; i < 3; i++)
    {
        uint8_t x = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        uint8_t y = view->getFieldAs<uint8_t>(Dimension::Id::X, i);
        uint8_t z = view->getFieldAs<uint8_t>(Dimension::Id::Y, i);

        EXPECT_EQ(x, i + 1u);
        EXPECT_EQ(y, i * 10u);
        EXPECT_EQ(z, i * 100u);
    }

    // read the view back out
    for (int i = 3; i < 17; i++)
    {
        uint8_t x = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        uint8_t y = view->getFieldAs<uint8_t>(Dimension::Id::X, i);
        EXPECT_THROW(view->getFieldAs<uint8_t>(Dimension::Id::Y, i),
            pdal_error);
        EXPECT_EQ(x, i + 1u);
        EXPECT_EQ(y, i * 10u);
    }
}

TEST(PointViewTest, getAsInt32)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    // read the view back out
    for (int i = 0; i < 17; i++)
    {
        int32_t x = view->getFieldAs<int32_t>(Dimension::Id::Classification, i);
        int32_t y = view->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t z = view->getFieldAs<int32_t>(Dimension::Id::Y, i);

        EXPECT_EQ(x, i + 1);
        EXPECT_EQ(y, i * 10);
        EXPECT_EQ(z, i * 100);
    }
}


TEST(PointViewTest, getFloat)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    // read the view back out
    for (int i = 0; i < 17; i++)
    {
        float x = view->getFieldAs<float>(Dimension::Id::Classification, i);
        float y = view->getFieldAs<float>(Dimension::Id::X, i);
        float z = view->getFieldAs<float>(Dimension::Id::Y, i);

        EXPECT_FLOAT_EQ(x, i + 1.0f);
        EXPECT_FLOAT_EQ(y, i * 10.0f);
        EXPECT_FLOAT_EQ(z, i * 100.0f);
    }
}


TEST(PointViewTest, copy)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    PointView d2(*view);

    // read the view back out
    {
        EXPECT_EQ(
            d2.getFieldAs<uint8_t>(Dimension::Id::Classification, 0),
            view->getFieldAs<uint8_t>(Dimension::Id::Classification, 0));
        EXPECT_EQ(d2.getFieldAs<int32_t>(Dimension::Id::X, 0),
            view->getFieldAs<int32_t>(Dimension::Id::X, 0));
        EXPECT_FLOAT_EQ(d2.getFieldAs<double>(Dimension::Id::Y, 0),
            view->getFieldAs<double>(Dimension::Id::Y, 0));
    }

    for (int i = 1; i < 17; i++)
    {
        uint8_t x = d2.getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        int32_t y = d2.getFieldAs<int32_t>(Dimension::Id::X, i);
        double z = d2.getFieldAs<double>(Dimension::Id::Y, i);

        EXPECT_EQ(x, i + 1u);
        EXPECT_EQ(y, i * 10);
        EXPECT_TRUE(Utils::compare_approx(z, i * 100.0,
            (std::numeric_limits<double>::min)()));
    }
    EXPECT_EQ(view->size(), d2.size());

}

TEST(PointViewTest, copyCtor)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    PointView d2(*view);
    verifyTestView(d2);
}

TEST(PointViewTest, assignment)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);

    PointView d2 = *view;
    verifyTestView(d2);
}


TEST(PointViewTest, metaview)
{
    PointTable table;
    PointViewPtr view = makeTestView(table, 2);

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);
    MetadataNode tree = Utils::toMetadata(view);

    std::ifstream str1(Support::datapath("pointbuffer/metaview.txt"));
    std::istringstream str2(Utils::toJSON(tree));
    //EXPECT_TRUE(Support::compare_text_files(str1, str2));
}


TEST(PointViewTest, bigfile)
{
    PointTable table;

    point_count_t NUM_PTS = 1000000;

    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    PointView view(table);

    for (PointId id = 0; id < NUM_PTS; ++id)
    {
        view.setField(Dimension::Id::X, id, id);
        view.setField(Dimension::Id::Y, id, 2 * id);
        view.setField(Dimension::Id::Z, id, -(int)id);
    }

    for (PointId id = 0; id < NUM_PTS; ++id)
    {
        EXPECT_EQ(
            view.getFieldAs<PointId>(Dimension::Id::X, id), id);
        EXPECT_EQ(
            view.getFieldAs<PointId>(Dimension::Id::Y, id), id * 2);
        EXPECT_EQ(
            view.getFieldAs<int>(Dimension::Id::Z, id), -(int)id);
    }

    // Test some random access.
    std::unique_ptr<PointId[]> ids(new PointId[NUM_PTS]);
    for (PointId idx = 0; idx < NUM_PTS; ++idx)
        ids[idx] = idx;
    // Do a bunch of random swaps.
    std::default_random_engine generator;
    std::uniform_int_distribution<PointId> distribution(0, NUM_PTS-1);
    for (PointId idx = 0; idx < NUM_PTS; ++idx)
    {
        PointId y = distribution(generator);
        PointId temp = std::move(ids[idx]);
        ids[idx] = std::move(ids[y]);
        ids[y] = std::move(temp);
    }

    for (PointId idx = 0; idx < NUM_PTS; ++idx)
    {
        PointId id = ids[idx];
        view.setField(Dimension::Id::X, id, idx);
        view.setField(Dimension::Id::Y, id, 2 * idx);
        view.setField(Dimension::Id::Z, id, -(int)idx);
    }

    for (PointId idx = 0; idx < NUM_PTS; ++idx)
    {
        PointId id = ids[idx];
        EXPECT_EQ(
            view.getFieldAs<PointId>(Dimension::Id::X, id), idx);
        EXPECT_EQ(
            view.getFieldAs<PointId>(Dimension::Id::Y, id), idx * 2);
        EXPECT_EQ(
            view.getFieldAs<int>(Dimension::Id::Z, id), -(int)idx);
    }
}


//ABELL - Move to KdIndex
/**
TEST(PointViewTest, kdindex)
{
    LasReader reader(Support::viewpath("1.2-with-color.las"));
    reader.prepare();

    const Schema& schema = reader.getSchema();
    uint32_t capacity(1000);
    PointView view(schema, capacity);

    StageSequentialIterator* iter = reader.createSequentialIterator(view);

    {
        uint32_t numRead = iter->read(view);
        EXPECT_EQ(numRead, capacity);
    }

    EXPECT_EQ(view.getCapacity(), capacity);
    EXPECT_EQ(view.getSchema(), schema);


    IndexedPointView iview(view);
    EXPECT_EQ(iview.getCapacity(), capacity);
    EXPECT_EQ(iview.getSchema(), schema);

    iview.build();

    unsigned k = 8;

    // If the query distance is 0, just return the k nearest neighbors
    std::vector<size_t> ids = idata.neighbors(636199, 849238, 428.05, k);
    EXPECT_EQ(ids.size(), k);
    EXPECT_EQ(ids[0], 8u);
    EXPECT_EQ(ids[1], 7u);
    EXPECT_EQ(ids[2], 9u);
    EXPECT_EQ(ids[3], 42u);
    EXPECT_EQ(ids[4], 40u);

    std::vector<size_t> dist_ids = idata.neighbors(636199, 849238, 428.05, 3);

    EXPECT_EQ(dist_ids.size(), 3u);
    EXPECT_EQ(dist_ids[0], 8u);

    std::vector<size_t> nids = idata.neighbors(636199, 849238, 428.05, k);

    EXPECT_EQ(nids.size(), k);
    EXPECT_EQ(nids[0], 8u);
    EXPECT_EQ(nids[1], 7u);
    EXPECT_EQ(nids[2], 9u);
    EXPECT_EQ(nids[3], 42u);
    EXPECT_EQ(nids[4], 40u);

    std::vector<size_t> rids = iview.radius(637012.24, 849028.31,
        431.66, 100000);
    EXPECT_EQ(rids.size(), 11u);

    delete iter;
}
**/


static void check_bounds(const BOX3D& box,
                         double minx, double maxx,
                         double miny, double maxy,
                         double minz, double maxz)
{
    EXPECT_FLOAT_EQ(box.minx, minx);
    EXPECT_FLOAT_EQ(box.maxx, maxx);
    EXPECT_FLOAT_EQ(box.miny, miny);
    EXPECT_FLOAT_EQ(box.maxy, maxy);
    EXPECT_FLOAT_EQ(box.minz, minz);
    EXPECT_FLOAT_EQ(box.maxz, maxz);
}


TEST(PointViewTest, calcBounds)
{
    auto set_points = [](PointViewPtr view, PointId i, double x, double y,
        double z)
    {
        view->setField(Dimension::Id::X, i, x);
        view->setField(Dimension::Id::Y, i, y);
        view->setField(Dimension::Id::Z, i, z);
    };

    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    const double lim_min = (std::numeric_limits<double>::lowest)();
    const double lim_max = (std::numeric_limits<double>::max)();
    PointViewPtr b0(new PointView(table));
    BOX3D box_b0;
    b0->calculateBounds(box_b0);
    check_bounds(box_b0, lim_max, lim_min, lim_max, lim_min, lim_max, lim_min);

    PointViewPtr b1(new PointView(table));
    set_points(b1, 0, 0.0, 0.0, 0.0);
    set_points(b1, 1, 2.0, 2.0, 2.0);

    PointViewPtr b2(new PointView(table));
    set_points(b2, 0, 3.0, 3.0, 3.0);
    set_points(b2, 1, 1.0, 1.0, 1.0);

    PointViewSet bs;
    bs.insert(b1);
    bs.insert(b2);

    BOX3D box_b1;
    b1->calculateBounds(box_b1);
    check_bounds(box_b1, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0);

    BOX3D box_b2;
    b2->calculateBounds(box_b2);
    check_bounds(box_b2, 1.0, 3.0, 1.0, 3.0, 1.0, 3.0);

    BOX3D box_bs;
    PointView::calculateBounds(bs, box_bs);
    check_bounds(box_bs, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0);
}

TEST(PointViewTest, order)
{
    PointTable table;

    const size_t COUNT(1000);
    std::array<PointViewPtr, COUNT> views;

    std::random_device dev;
    std::mt19937 generator(dev());
   
    for (size_t i = 0; i < COUNT; ++i)
        views[i] = PointViewPtr(new PointView(table));
    std::shuffle(views.begin(), views.end(), generator);

    PointViewSet set;
    for (size_t i = 0; i < COUNT; ++i)
        set.insert(views[i]);

    PointViewSet::iterator pi;
    for (auto si = set.begin(); si != set.end(); ++si)
    {
        if (si != set.begin())
           EXPECT_TRUE((*pi)->id() < (*si)->id());
        pi = si;
    }
}
