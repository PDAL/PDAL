/******************************************************************************
* Copyright (c) 2011, Howard Butler <hobu.inc@gmail.com>
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
#include <boost/cstdint.hpp>
#include <pdal/Endian.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>
#include <pdal/filters/ByteSwapFilter.hpp>

#include <boost/scoped_ptr.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Endian.hpp>
#include <iostream>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ByteSwapFilterTest)

BOOST_AUTO_TEST_CASE(test_swapping)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    boost::uint32_t buffer_size = 20;

    Options options;
    options.add("bounds", srcBounds);
    options.add("num_points", buffer_size);
    options.add("mode", "ramp");

    DataStagePtr reader(new pdal::drivers::faux::Reader(options));

    DataStagePtr filter(new pdal::filters::ByteSwapFilter(reader, Options::empty()));
    BOOST_CHECK_EQUAL(filter->getName(), "filters.byteswap");

    boost::scoped_ptr<StageSequentialIterator> unflipped_iter(reader->createSequentialIterator());
    boost::scoped_ptr<StageSequentialIterator> flipped_iter(filter->createSequentialIterator());

    const Schema& schema = reader->getSchema();
    
    PointBuffer flipped(filter->getSchema(), buffer_size);
    const boost::uint32_t fliped_read = flipped_iter->read(flipped);
    BOOST_CHECK_EQUAL(fliped_read, buffer_size);

    PointBuffer unflipped(schema, buffer_size);
    const boost::uint32_t unfliped_read = unflipped_iter->read(unflipped);
    BOOST_CHECK_EQUAL(unfliped_read, buffer_size);
    

    int offsetX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Double);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Double);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Double);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Uint64);
    
    BOOST_CHECK_EQUAL(offsetX, 0);
    BOOST_CHECK_EQUAL(offsetY, 1);
    BOOST_CHECK_EQUAL(offsetZ, 2);
    BOOST_CHECK_EQUAL(offsetT, 3);
    
    for (boost::uint32_t i = 0 ; i < buffer_size; ++i)
    {

        double unflipped_x = unflipped.getField<double>(i, offsetX);
        double unflipped_y = unflipped.getField<double>(i, offsetY);
        double unflipped_z = unflipped.getField<double>(i, offsetZ);
        boost::uint64_t unflipped_t = unflipped.getField<boost::uint64_t>(i, offsetT);
        

        double flipped_x = flipped.getField<double>(i, offsetX);
        double flipped_y = flipped.getField<double>(i, offsetY);
        double flipped_z = flipped.getField<double>(i, offsetZ);
        boost::uint64_t flipped_t = flipped.getField<boost::uint64_t>(i, offsetT);

        double reflipped_x = flipped_x;
        SWAP_ENDIANNESS(reflipped_x);
        BOOST_CHECK_EQUAL(unflipped_x, reflipped_x);

        double reflipped_y = flipped_y;
        SWAP_ENDIANNESS(reflipped_y);
        BOOST_CHECK_EQUAL(unflipped_y, reflipped_y);

        double reflipped_z = flipped_z;
        SWAP_ENDIANNESS(reflipped_z);
        BOOST_CHECK_EQUAL(unflipped_z, reflipped_z);        

        boost::uint64_t reflipped_t = flipped_t;
        SWAP_ENDIANNESS(reflipped_t);
        BOOST_CHECK_EQUAL(unflipped_t, reflipped_t);
    }
    // 
    // // 1000 * 1/3 = 333, plus or minus a bit for rounding
    // BOOST_CHECK(Utils::compare_approx<double>(static_cast<double>(numWritten), 333, 6));
    // 
    // const double minX = writer.getMinX();
    // const double minY = writer.getMinY();
    // const double minZ = writer.getMinZ();
    // const double maxX = writer.getMaxX();
    // const double maxY = writer.getMaxY();
    // const double maxZ = writer.getMaxZ();
    // const double avgX = writer.getAvgX();
    // const double avgY = writer.getAvgY();
    // const double avgZ = writer.getAvgZ();
    // 
    // const double delX = 10.0 / 999.0;
    // const double delY = 100.0 / 999.0;
    // const double delZ = 1000.0 / 999.0;
    // 
    // BOOST_CHECK(Utils::compare_approx<double>(minX, 3.33333, delX));
    // BOOST_CHECK(Utils::compare_approx<double>(minY, 33.33333, delY));
    // BOOST_CHECK(Utils::compare_approx<double>(minZ, 333.33333, delZ));
    // BOOST_CHECK(Utils::compare_approx<double>(maxX, 6.66666, delX));
    // BOOST_CHECK(Utils::compare_approx<double>(maxY, 66.66666, delY));
    // BOOST_CHECK(Utils::compare_approx<double>(maxZ, 666.66666, delZ));
    // BOOST_CHECK(Utils::compare_approx<double>(avgX, 5.00000, delX));
    // BOOST_CHECK(Utils::compare_approx<double>(avgY, 50.00000, delY));
    // BOOST_CHECK(Utils::compare_approx<double>(avgZ, 500.00000, delZ));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
