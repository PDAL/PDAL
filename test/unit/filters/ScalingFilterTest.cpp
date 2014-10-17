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

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/filters/Scaling.hpp>

#include "../StageTester.hpp"
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ScalingFilterTest)

BOOST_AUTO_TEST_CASE(ScalingFilterTest_test_1)
{
    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(Support::datapath("filters/scaling.xml"));

    point_count_t count = mgr.execute();
    PointBufferSet pbSet = mgr.buffers();
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    Schema *schema = mgr.schema();
    DimensionPtr xDim = schema->getDimension("X", "filters.scaling");
    DimensionPtr yDim = schema->getDimension("Y", "filters.scaling");
    
    int32_t x = buf->getFieldAs<int32_t>(xDim, 0, false);
    int32_t y = buf->getFieldAs<int32_t>(yDim, 0, false);
    BOOST_CHECK_EQUAL(x, 6370112);
    BOOST_CHECK_EQUAL(y, 8490263);

    x = buf->getFieldAs<int32_t>(xDim, 10, false);
    y = buf->getFieldAs<int32_t>(yDim, 10, false);
    BOOST_CHECK_EQUAL(x, 6360365);
    BOOST_CHECK_EQUAL(y, 8493365);

    DimensionPtr xDimRaw = schema->getDimension("X", "drivers.las.reader");
    DimensionPtr yDimRaw = schema->getDimension("Y", "drivers.las.reader");

    x = buf->getFieldAs<int32_t>(xDimRaw, 0, false);
    y = buf->getFieldAs<int32_t>(yDimRaw, 0, false);
    BOOST_CHECK_EQUAL(x, 63701224);
    BOOST_CHECK_EQUAL(y, 84902831);

    x = buf->getFieldAs<int32_t>(xDimRaw, 10, false);
    y = buf->getFieldAs<int32_t>(yDimRaw, 10, false);
    BOOST_CHECK_EQUAL(x, 63603753);
    BOOST_CHECK_EQUAL(y, 84933845);
}


BOOST_AUTO_TEST_CASE(ScalingFilterFloat_test)
{

    const Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Option opt1("bounds", bounds);
    Option opt2("mode", "conSTanT");
    Option opt3("num_points", 1000);
    Option opt4("id", 90210);
    Options opts;
    opts.add(opt1);
    opts.add(opt2);
    opts.add(opt3);
    opts.add(opt4);

    Option scalex("scale", 0.00001, "fpscale");
    Option offsetx("offset", 12345, "offset");
    Option xdim("dimension", "X", "dimension to scale");
    Option debug("debug", true, "");
    Option verbose("verbose", 7, "");
    // opts.add(debug);
    // opts.add(verbose);
    Options xs;
    xs.add(scalex);
    xs.add(offsetx);
    xdim.setOptions(xs);
    opts.add(xdim);

    Option scaley("scale", 0.001, "fpscale");
    Option offsety("offset", 12345, "offset");
    Option sizey("size", 4, "size");
    Option typey("type", "SignedInteger", "tye");
    Option ydim("dimension", "Y", "dimension to scale");
    Options ys;
    ys.add(scaley);
    ys.add(offsety);
    ys.add(sizey);
    ys.add(typey);
    ydim.setOptions(ys);
    opts.add(ydim);

    Option scalet("scale", 0.1, "fpscale");
    Option offsett("offset", 0, "offset");
    Option sizet("size", 4, "size");
    Option typet("type", "UnsignedInteger", "tye");
    Option tdim("dimension", "Time", "dimension to scale");
    Options ts;
    ts.add(scalet);
    ts.add(offsett);
    ts.add(sizet);
    ts.add(typet);
    tdim.setOptions(ts);
    opts.add(tdim);

    PointContext ctx;
    PointBuffer buf(ctx);

    drivers::faux::Reader reader(opts);
    filters::Scaling scaling(opts);
    scaling.setInput(&reader);
    scaling.prepare(ctx);

    StageSequentialIterator* iter = reader.createSequentialIterator();
    point_count_t numRead = iter->read(buf, 750);
    BOOST_CHECK_EQUAL(numRead, 750u);

    FilterTester::ready(&scaling, ctx);
    FilterTester::filter(&scaling, buf);
    FilterTester::done(&scaling, ctx);

    Schema *schema = ctx.schema();
    DimensionPtr dimX = schema->getDimension("X");
    DimensionPtr dimY = schema->getDimension("Y");
    DimensionPtr dimZ = schema->getDimension("Z");
    DimensionPtr dimTime = schema->getDimension("Time");

    for (PointId i = 0; i < 2; ++i)
    {
        int32_t x = buf.getFieldAs<int32_t>(dimX, i, false);
        int32_t y = buf.getFieldAs<int32_t>(dimY, i, false);
        double z = buf.getFieldAs<double>(dimZ, i, false);
        int32_t t = buf.getFieldAs<int32_t>(dimTime, i, false);

        BOOST_CHECK_EQUAL(x, -1234400000);
        BOOST_CHECK_EQUAL(y, -12343000);
        BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        BOOST_CHECK_EQUAL(t, i * 10); // because of scaling
    }

    delete iter;
}


BOOST_AUTO_TEST_SUITE_END()
