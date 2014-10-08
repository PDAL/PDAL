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

#include <boost/test/unit_test.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/Bounds.hpp>

BOOST_AUTO_TEST_SUITE(FauxReaderTest)

using namespace pdal;

BOOST_AUTO_TEST_CASE(test_constant_mode_sequential_iter)
{
    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "constant");
    drivers::faux::Reader reader;
    reader.setOptions(ops);

    PointContext ctx;
    reader.prepare(ctx);
    BOOST_CHECK_EQUAL(reader.getDescription(), "Faux Reader");
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 750);
    for (point_count_t i = 0; i < buf->size(); i++)
    {
        double x = buf->getFieldAs<double>(Dimension::Id::X, i);
        double y = buf->getFieldAs<double>(Dimension::Id::Y, i);
        double z = buf->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
        BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
        BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        BOOST_CHECK_EQUAL(t, i);
    }
}


BOOST_AUTO_TEST_CASE(test_random_mode)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "constant");
    drivers::faux::Reader reader;
    reader.setOptions(ops);

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 750);

    for (point_count_t i = 0; i < buf->size(); ++i)
    {
        double x = buf->getFieldAs<double>(Dimension::Id::X, i);
        double y = buf->getFieldAs<double>(Dimension::Id::Y, i);
        double z = buf->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        BOOST_CHECK_GE(x, 1.0);
        BOOST_CHECK_LE(x, 101.0);

        BOOST_CHECK_GE(y, 2.0);
        BOOST_CHECK_LE(y, 102.0);

        BOOST_CHECK_GE(z, 3.0);
        BOOST_CHECK_LE(z, 103.0);

        BOOST_CHECK_EQUAL(t, i);
    }
}


BOOST_AUTO_TEST_CASE(test_ramp_mode_1)
{
    using namespace pdal;

    BOX3D bounds(0, 0, 0, 4, 4, 4);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 2);
    ops.add("mode", "ramp");

    drivers::faux::Reader reader;
    reader.setOptions(ops);

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 2);

    double x0 = buf->getFieldAs<double>(Dimension::Id::X, 0);
    double y0 = buf->getFieldAs<double>(Dimension::Id::Y, 0);
    double z0 = buf->getFieldAs<double>(Dimension::Id::Z, 0);
    uint64_t t0 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 0);

    double x1 = buf->getFieldAs<double>(Dimension::Id::X, 1);
    double y1 = buf->getFieldAs<double>(Dimension::Id::Y, 1);
    double z1 = buf->getFieldAs<double>(Dimension::Id::Z, 1);
    uint64_t t1 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 1);

    BOOST_CHECK_CLOSE(x0, 0.0, 0.00001);
    BOOST_CHECK_CLOSE(y0, 0.0, 0.00001);
    BOOST_CHECK_CLOSE(z0, 0.0, 0.00001);
    BOOST_CHECK_EQUAL(t0, 0u);

    BOOST_CHECK_CLOSE(x1, 4.0, 0.00001);
    BOOST_CHECK_CLOSE(y1, 4.0, 0.00001);
    BOOST_CHECK_CLOSE(z1, 4.0, 0.00001);
    BOOST_CHECK_EQUAL(t1, 1u);
}


BOOST_AUTO_TEST_CASE(test_ramp_mode_2)
{
    using namespace pdal;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 152.0, 203.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "ramp");
    drivers::faux::Reader reader;
    reader.setOptions(ops);

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 750);

    double delX = (101.0 - 1.0) / (750.0 - 1.0);
    double delY = (152.0 - 2.0) / (750.0 - 1.0);
    double delZ = (203.0 - 3.0) / (750.0 - 1.0);

    for (point_count_t i = 0; i < buf->size(); ++i)
    {
        double x = buf->getFieldAs<double>(Dimension::Id::X, i);
        double y = buf->getFieldAs<double>(Dimension::Id::Y, i);
        double z = buf->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        BOOST_CHECK_CLOSE(x, 1.0 + delX * i, 0.00001);
        BOOST_CHECK_CLOSE(y, 2.0 + delY * i, 0.00001);
        BOOST_CHECK_CLOSE(z, 3.0 + delZ * i, 0.00001);
        BOOST_CHECK_EQUAL(t, i);
    }
}


BOOST_AUTO_TEST_CASE(test_return_number)
{
    using namespace pdal;

    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("count", 100);
    ops.add("mode", "constant");
    ops.add("number_of_returns", 9);
    drivers::faux::Reader reader;
    reader.setOptions(ops);

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 100);

    for (point_count_t i = 0; i < buf->size(); i++)
    {
        uint8_t returnNumber = buf->getFieldAs<uint8_t>(
            Dimension::Id::ReturnNumber, i);
        uint8_t numberOfReturns =
            buf->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, i);

        BOOST_CHECK_EQUAL(returnNumber, (i % 9) + 1);
        BOOST_CHECK_EQUAL(numberOfReturns, 9);
    }
}

BOOST_AUTO_TEST_SUITE_END()
