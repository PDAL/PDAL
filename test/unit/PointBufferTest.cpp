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
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "UnitTest.hpp"

#include <random>

#include <boost/property_tree/xml_parser.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/PointBufferIter.hpp>
#include <LasReader.hpp>
#include <pdal/PDALUtils.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PointBufferTest)

PointBuffer* makeTestBuffer(PointContext ctx)
{
    ctx.registerDim(Dimension::Id::Classification);
    ctx.registerDim(Dimension::Id::X);
    ctx.registerDim(Dimension::Id::Y);

    PointBuffer* data = new PointBuffer(ctx);

    // write the data into the buffer
    for (uint32_t i = 0; i < 17; i++)
    {
        const uint8_t x = (uint8_t)(i + 1);
        const int32_t y = i * 10;
        const double z = i * 100;

        data->setField(Dimension::Id::Classification, i, x);
        data->setField(Dimension::Id::X, i, y);
        data->setField(Dimension::Id::Y, i, z);
    }
    BOOST_CHECK(data->size() == 17);
    return data;
}


static void verifyTestBuffer(const PointBuffer& data)
{
    // read the data back out
    for (int i = 0; i < 17; i++)
    {
        const uint8_t x = data.getFieldAs<uint8_t>(
            Dimension::Id::Classification, i);
        const int32_t y = data.getFieldAs<uint32_t>(Dimension::Id::X, i);
        const double z = data.getFieldAs<double>(Dimension::Id::Y, i);

        BOOST_CHECK_EQUAL(x, i + 1);
        BOOST_CHECK_EQUAL(y, i * 10);
        BOOST_CHECK(Utils::compare_approx(z, static_cast<double>(i) * 100.0,
            (std::numeric_limits<double>::min)()));
    }
}

BOOST_AUTO_TEST_CASE(getSet)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);
    verifyTestBuffer(*data);
    delete data;
}

BOOST_AUTO_TEST_CASE(getAsUint8)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    // read the data back out
    for (int i = 0; i < 3; i++)
    {
        uint8_t x = data->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        uint8_t y = data->getFieldAs<uint8_t>(Dimension::Id::X, i);
        uint8_t z = data->getFieldAs<uint8_t>(Dimension::Id::Y, i);

        BOOST_CHECK_EQUAL(x, i + 1);
        BOOST_CHECK_EQUAL(y, i * 10);
        BOOST_CHECK_EQUAL(z, i * 100);
    }

    // read the data back out
    for (int i = 3; i < 17; i++)
    {
        uint8_t x = data->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        uint8_t y = data->getFieldAs<uint8_t>(Dimension::Id::X, i);
        BOOST_CHECK_THROW(data->getFieldAs<uint8_t>(Dimension::Id::Y, i),
            pdal_error);
        BOOST_CHECK(x == i + 1);
        BOOST_CHECK(y == i * 10);
    }
    delete data;
}

BOOST_AUTO_TEST_CASE(getAsInt32)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    // read the data back out
    for (int i = 0; i < 17; i++)
    {
        int32_t x = data->getFieldAs<int32_t>(Dimension::Id::Classification, i);
        int32_t y = data->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t z = data->getFieldAs<int32_t>(Dimension::Id::Y, i);

        BOOST_CHECK_EQUAL(x, i + 1);
        BOOST_CHECK_EQUAL(y, i * 10);
        BOOST_CHECK_EQUAL(z, i * 100);
    }
    delete data;
}


BOOST_AUTO_TEST_CASE(getFloat)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    // read the data back out
    for (int i = 0; i < 17; i++)
    {
        float x = data->getFieldAs<float>(Dimension::Id::Classification, i);
        float y = data->getFieldAs<float>(Dimension::Id::X, i);
        float z = data->getFieldAs<float>(Dimension::Id::Y, i);

        BOOST_CHECK_CLOSE(x, i + 1.0f, std::numeric_limits<float>::min());
        BOOST_CHECK_CLOSE(y, i * 10.0f, std::numeric_limits<float>::min());
        BOOST_CHECK_CLOSE(z, i * 100.0f, std::numeric_limits<float>::min());
    }
    delete data;
}


BOOST_AUTO_TEST_CASE(copy)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    PointBuffer d2(*data);

    // read the data back out
    {
        BOOST_CHECK_EQUAL(
            d2.getFieldAs<uint8_t>(Dimension::Id::Classification, 0),
            data->getFieldAs<uint8_t>(Dimension::Id::Classification, 0));
        BOOST_CHECK_EQUAL(d2.getFieldAs<int32_t>(Dimension::Id::X, 0),
            data->getFieldAs<int32_t>(Dimension::Id::X, 0));
        BOOST_CHECK_EQUAL(d2.getFieldAs<double>(Dimension::Id::Y, 0),
            data->getFieldAs<double>(Dimension::Id::Y, 0));
    }

    for (int i = 1; i < 17; i++)
    {
        uint8_t x = d2.getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        int32_t y = d2.getFieldAs<int32_t>(Dimension::Id::X, i);
        double z = d2.getFieldAs<double>(Dimension::Id::Y, i);

        BOOST_CHECK_EQUAL(x, i + 1);
        BOOST_CHECK_EQUAL(y, i * 10);
        BOOST_CHECK(Utils::compare_approx(z, i * 100.0,
            (std::numeric_limits<double>::min)()));
    }
    BOOST_CHECK_EQUAL(data->size(), d2.size());

    delete data;
}

BOOST_AUTO_TEST_CASE(copyCtor)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    PointBuffer d2(*data);
    verifyTestBuffer(d2);
    delete data;
}

BOOST_AUTO_TEST_CASE(assignment)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    PointBuffer d2 = *data;
    verifyTestBuffer(d2);
    delete data;
}


BOOST_AUTO_TEST_CASE(ptree)
{
    PointContext ctx;
    PointBuffer* data = makeTestBuffer(ctx);

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);
    boost::property_tree::ptree tree = pdal::utils::toPTree(*data);
    delete data;

    boost::property_tree::write_xml(ss1, tree);
    std::string out1 = ss1.str();
    std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    std::string ref = xml_header + "<0><X>0</X><Y>0</Y><Classification>1</Classification></0><1><X>10</X><Y>100</Y><Classification>2</Classification></1>";

    BOOST_CHECK_EQUAL(ref, out1.substr(0, ref.length()));
}


BOOST_AUTO_TEST_CASE(bigfile)
{
    PointContext ctx;

    point_count_t NUM_PTS = 1000000;

    ctx.registerDim(Dimension::Id::X);
    ctx.registerDim(Dimension::Id::Y);
    ctx.registerDim(Dimension::Id::Z);
    PointBuffer buf(ctx);

    for (PointId id = 0; id < NUM_PTS; ++id)
    {
        buf.setField(Dimension::Id::X, id, id);
        buf.setField(Dimension::Id::Y, id, 2 * id);
        buf.setField(Dimension::Id::Z, id, -(int)id);
    }

    for (PointId id = 0; id < NUM_PTS; ++id)
    {
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<PointId>(Dimension::Id::X, id), id);
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<PointId>(Dimension::Id::Y, id), id * 2);
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<int>(Dimension::Id::Z, id), -(int)id);
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
        buf.setField(Dimension::Id::X, id, idx);
        buf.setField(Dimension::Id::Y, id, 2 * idx);
        buf.setField(Dimension::Id::Z, id, -(int)idx);
    }

    for (PointId idx = 0; idx < NUM_PTS; ++idx)
    {
        PointId id = ids[idx];
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<PointId>(Dimension::Id::X, id), idx);
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<PointId>(Dimension::Id::Y, id), idx * 2);
        BOOST_CHECK_EQUAL(
            buf.getFieldAs<int>(Dimension::Id::Z, id), -(int)idx);
    }
}


//ABELL - Move to KdIndex
/**
BOOST_AUTO_TEST_CASE(kdindex)
{
    LasReader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare();

    const Schema& schema = reader.getSchema();
    boost::uint32_t capacity(1000);
    PointBuffer data(schema, capacity);

    StageSequentialIterator* iter = reader.createSequentialIterator(data);

    {
        uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, capacity);
    }

    BOOST_CHECK_EQUAL(data.getCapacity(), capacity);
    BOOST_CHECK_EQUAL(data.getSchema(), schema);


    IndexedPointBuffer idata(data);
    BOOST_CHECK_EQUAL(idata.getCapacity(), capacity);
    BOOST_CHECK_EQUAL(idata.getSchema(), schema);

    idata.build();

    unsigned k = 8;

    // If the query distance is 0, just return the k nearest neighbors
    std::vector<size_t> ids = idata.neighbors(636199, 849238, 428.05, 0.0, k);
    BOOST_CHECK_EQUAL(ids.size(), k);
    BOOST_CHECK_EQUAL(ids[0], 8u);
    BOOST_CHECK_EQUAL(ids[1], 7u);
    BOOST_CHECK_EQUAL(ids[2], 9u);
    BOOST_CHECK_EQUAL(ids[3], 42u);
    BOOST_CHECK_EQUAL(ids[4], 40u);

    std::vector<size_t> dist_ids = idata.neighbors(636199, 849238, 428.05, 100.0, 3);

    BOOST_CHECK_EQUAL(dist_ids.size(), 3u);
    BOOST_CHECK_EQUAL(dist_ids[0], 8u);

    std::vector<size_t> nids = idata.neighbors(636199, 849238, 428.05, 0.0, k);

    BOOST_CHECK_EQUAL(nids.size(), k);
    BOOST_CHECK_EQUAL(nids[0], 8u);
    BOOST_CHECK_EQUAL(nids[1], 7u);
    BOOST_CHECK_EQUAL(nids[2], 9u);
    BOOST_CHECK_EQUAL(nids[3], 42u);
    BOOST_CHECK_EQUAL(nids[4], 40u);

    std::vector<size_t> rids = idata.radius(637012.24, 849028.31,
        431.66, 100000);
    BOOST_CHECK_EQUAL(rids.size(), 11u);

    delete iter;
}
**/


static void check_bounds(const BOX3D& box,
                         double minx, double maxx,
                         double miny, double maxy,
                         double minz, double maxz)
{
    const double eps = std::numeric_limits<double>::epsilon();
    BOOST_CHECK_CLOSE(box.minx, minx, eps);
    BOOST_CHECK_CLOSE(box.maxx, maxx, eps);
    BOOST_CHECK_CLOSE(box.miny, miny, eps);
    BOOST_CHECK_CLOSE(box.maxy, maxy, eps);
    BOOST_CHECK_CLOSE(box.minz, minz, eps);
    BOOST_CHECK_CLOSE(box.maxz, maxz, eps);
}


BOOST_AUTO_TEST_CASE(calcBounds)
{
    auto set_points = [](PointBufferPtr buf, PointId i, double x, double y,
        double z)
    {
        buf->setField(Dimension::Id::X, i, x);
        buf->setField(Dimension::Id::Y, i, y);
        buf->setField(Dimension::Id::Z, i, z);
    };

    PointContext ctx;
    ctx.registerDim(Dimension::Id::X);
    ctx.registerDim(Dimension::Id::Y);
    ctx.registerDim(Dimension::Id::Z);

    const double lim_min = (std::numeric_limits<double>::lowest)();
    const double lim_max = (std::numeric_limits<double>::max)();
    PointBufferPtr b0(new PointBuffer(ctx));
    const BOX3D box_b0 = b0->calculateBounds(true);
    check_bounds(box_b0, lim_max, lim_min, lim_max, lim_min, lim_max, lim_min);

    PointBufferPtr b1(new PointBuffer(ctx));
    set_points(b1, 0, 0.0, 0.0, 0.0);
    set_points(b1, 1, 2.0, 2.0, 2.0);

    PointBufferPtr b2(new PointBuffer(ctx));
    set_points(b2, 0, 3.0, 3.0, 3.0);
    set_points(b2, 1, 1.0, 1.0, 1.0);
    
    PointBufferSet bs;
    bs.insert(b1);
    bs.insert(b2);

    const BOX3D box_b1 = b1->calculateBounds(true);
    check_bounds(box_b1, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0);
    
    const BOX3D box_b2 = b2->calculateBounds(true);
    check_bounds(box_b2, 1.0, 3.0, 1.0, 3.0, 1.0, 3.0);

    BOX3D box_bs = PointBuffer::calculateBounds(bs);
    check_bounds(box_bs, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0);
}

BOOST_AUTO_TEST_CASE(sort)
{
    PointContext ctx;
    PointBuffer buf(ctx);
    const PointId NUM_PTS = 10000;

    auto cmp = [](const PointRef& p1, const PointRef& p2)
    {
        return p1.compare(Dimension::Id::X, p2);
    };

    ctx.registerDim(Dimension::Id::X);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist(0.0, 10000.0);
    for (PointId i = 0; i < NUM_PTS; ++i)
        buf.setField(Dimension::Id::X, i, dist(generator));    

    std::sort(buf.begin(), buf.end(), cmp);

    for (PointId i = 1; i < NUM_PTS; ++i)
    {
        double d1 = buf.getFieldAs<double>(Dimension::Id::X, i - 1);
        double d2 = buf.getFieldAs<double>(Dimension::Id::X, i);
        BOOST_CHECK(d1 <= d2);
    }
}

BOOST_AUTO_TEST_SUITE_END()
