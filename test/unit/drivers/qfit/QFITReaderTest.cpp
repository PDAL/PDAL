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

#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/qfit/Reader.hpp>
#include "Support.hpp"

#include <iostream>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(QFITReaderTest)


#define Compare(x,y)    BOOST_CHECK_CLOSE(x,y,0.00001);


void Check_Point(const pdal::PointBuffer& data,
                 std::size_t index,
                 double xref, double yref, double zref,
                 int32_t tref)
{
    double x = data.getFieldAs<double>(Dimension::Id::X, index);
    double y = data.getFieldAs<double>(Dimension::Id::Y, index);
    double z = data.getFieldAs<double>(Dimension::Id::Z, index);
    int32_t t = data.getFieldAs<int32_t>(Dimension::Id::OffsetTime, index);

    Compare(x, xref);
    Compare(y, yref);
    Compare(z, zref);
    BOOST_CHECK_EQUAL(t, tref);
}

BOOST_AUTO_TEST_CASE(test_10_word)
{
    Options options;

    options.add("filename", Support::datapath("qfit/10-word.qi"),
        "Input filename for reader to use");
    options.add("flip_coordinates", false,
        "Flip coordinates from 0-360 to -180-180");
    options.add("scale_z", 0.001f, "Z scale from mm to m");
    options.add("count", 3);

    drivers::qfit::Reader reader;
    reader.setOptions(options);
    BOOST_CHECK(reader.getDescription() == "QFIT Reader");
    BOOST_CHECK_EQUAL(reader.getName(), "drivers.qfit.reader");

    PointContext ctx;
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 3);

    Check_Point(*buf, 0, 221.826822, 59.205160, 32.0900, 0);
    Check_Point(*buf, 1, 221.826740, 59.205161, 32.0190, 0);
    Check_Point(*buf, 2, 221.826658, 59.205164, 32.0000, 0);
}

BOOST_AUTO_TEST_CASE(test_14_word)
{
    Options options;

    options.add("filename", Support::datapath("qfit/14-word.qi"),
        "Input filename for reader to use");
    options.add("flip_coordinates", false,
        "Flip coordinates from 0-360 to -180-180");
    options.add("scale_z", 0.001f, "Z scale from mm to m");
    options.add("count", 3);

    PointContext ctx;
    drivers::qfit::Reader reader;
    reader.setOptions(options);
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 3);

    Check_Point(*buf, 0, 244.306337, 35.623317, 1056.830000000, 903);
    Check_Point(*buf, 1, 244.306260, 35.623280, 1056.409000000, 903);
    Check_Point(*buf, 2, 244.306204, 35.623257, 1056.483000000, 903);
}

BOOST_AUTO_TEST_SUITE_END()
