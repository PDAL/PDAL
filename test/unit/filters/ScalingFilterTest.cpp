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

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ScalingFilterTest)

BOOST_AUTO_TEST_CASE(ScalingFilterTest_test_1)
{
    pdal::Option option("filename", Support::datapath("filters/scaling.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();

    pdal::filters::Scaling const* filter = static_cast<pdal::filters::Scaling const*>(reader.getManager().getStage());
    pdal::Options opt = filter->getOptions();
    // std::cout << "filter ops: " << opt << std::endl;

    const pdal::Schema& schema = filter->getSchema();
    pdal::PointBuffer data(schema, 1);

    pdal::StageSequentialIterator* iter = filter->createSequentialIterator(data);

    boost::uint32_t numRead = iter->read(data);
    BOOST_CHECK(numRead == 1);
    delete iter;

    pdal::Schema const& schema2 = data.getSchema();

    boost::optional<pdal::Dimension const&> scaledDimX = schema2.getDimension("X", "filters.scaling");
    boost::optional<pdal::Dimension const&> scaledDimY = schema2.getDimension("Y", "filters.scaling");

    if (!scaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    boost::int32_t x = data.getField<boost::int32_t>(*scaledDimX, 0);
    boost::int32_t y = data.getField<boost::int32_t>(*scaledDimY, 0);

    BOOST_CHECK_EQUAL(x, 6370112);
    BOOST_CHECK_EQUAL(y, 8490263);

    boost::optional<pdal::Dimension const&> unscaledDimX = schema2.getDimension("X", "drivers.las.reader");
    boost::optional<pdal::Dimension const&> unscaledDimY = schema2.getDimension("Y", "drivers.las.reader");

    if (!unscaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    x = data.getField<boost::int32_t>(*unscaledDimX, 0);
    y = data.getField<boost::int32_t>(*unscaledDimY, 0);

    BOOST_CHECK_EQUAL(x, 63701224);
    BOOST_CHECK_EQUAL(y, 84902831);

    return;
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

    Option scalex("scale", 0.00001f, "fpscale");
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

    Option scaley("scale", 0.001f, "fpscale");
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

    Option scalet("scale", 0.1f, "fpscale");
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

    pdal::drivers::faux::Reader reader(opts);
    pdal::filters::Scaling scaling(reader, opts);
    scaling.initialize();

    const Schema& schema = scaling.getSchema();

    PointBuffer data(schema, 750);

    StageSequentialIterator* iter = scaling.createSequentialIterator(data);
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 750u);

    Schema const& buffer_schema = data.getSchema();
    Dimension const& dimX = buffer_schema.getDimension("X");
    Dimension const& dimY = buffer_schema.getDimension("Y");
    Dimension const& dimZ = buffer_schema.getDimension("Z");
    Dimension const& dimTime = buffer_schema.getDimension("Time");

    for (boost::uint32_t i=0; i<2; i++)
    {
        boost::int32_t x = data.getField<boost::int32_t>(dimX, i);
        boost::int32_t y = data.getField<boost::int32_t>(dimY, i);
        double z = data.getField<double>(dimZ, i);
        boost::uint64_t t = data.getField<boost::uint64_t>(dimTime, i);

        BOOST_CHECK_EQUAL(x, -1234400030);
        BOOST_CHECK_EQUAL(y, -12342999);
        BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        BOOST_CHECK_EQUAL(t, i*10); // because of scaling
    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(ScalingFilterTest_test_2)
{
    pdal::Option option("filename", Support::datapath("filters/scaling.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();

    pdal::filters::Scaling const* filter = static_cast<pdal::filters::Scaling const*>(reader.getManager().getStage());
    pdal::Options opt = filter->getOptions();
    // std::cout << "filter ops: " << opt << std::endl;

    const pdal::Schema& schema = filter->getSchema();
    pdal::PointBuffer data(schema, 1);

    pdal::StageRandomIterator* iter = filter->createRandomIterator(data);
    
    if (!iter) throw std::runtime_error("createRandomIterator returned NULL");
    iter->seek(10);
    boost::uint32_t numRead = iter->read(data);
    BOOST_CHECK(numRead == 1);
    delete iter;

    pdal::Schema const& schema2 = data.getSchema();

    boost::optional<pdal::Dimension const&> scaledDimX = schema2.getDimension("X", "filters.scaling");
    boost::optional<pdal::Dimension const&> scaledDimY = schema2.getDimension("Y", "filters.scaling");

    if (!scaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    boost::int32_t x = data.getField<boost::int32_t>(*scaledDimX, 0);
    boost::int32_t y = data.getField<boost::int32_t>(*scaledDimY, 0);

    BOOST_CHECK_EQUAL(x, 6360365);
    BOOST_CHECK_EQUAL(y, 8493364);

    boost::optional<pdal::Dimension const&> unscaledDimX = schema2.getDimension("X", "drivers.las.reader");
    boost::optional<pdal::Dimension const&> unscaledDimY = schema2.getDimension("Y", "drivers.las.reader");

    if (!unscaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    x = data.getField<boost::int32_t>(*unscaledDimX, 0);
    y = data.getField<boost::int32_t>(*unscaledDimY, 0);

    BOOST_CHECK_EQUAL(x, 63603753);
    BOOST_CHECK_EQUAL(y, 84933845);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
