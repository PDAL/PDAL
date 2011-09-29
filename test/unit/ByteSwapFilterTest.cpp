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

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ByteSwapFilterTest)

BOOST_AUTO_TEST_CASE(test_swapping)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    
    boost::uint32_t buffer_size = 20;
    pdal::drivers::faux::Reader reader(srcBounds, buffer_size, pdal::drivers::faux::Reader::Ramp);

    pdal::filters::ByteSwapFilter filter(reader);
    BOOST_CHECK_EQUAL(filter.getName(), "filters.byteswap");

    filter.initialize();

    BOOST_CHECK_EQUAL(reader.getSchema().getDimension(DimensionId::X_f64).getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(reader.getSchema().getDimension(DimensionId::Y_f64).getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(reader.getSchema().getDimension(DimensionId::Z_f64).getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(reader.getSchema().getDimension(DimensionId::Time_u64).getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(filter.getSchema().getDimension(DimensionId::X_f64).getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filter.getSchema().getDimension(DimensionId::Y_f64).getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filter.getSchema().getDimension(DimensionId::Z_f64).getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filter.getSchema().getDimension(DimensionId::Time_u64).getEndianness(), pdal::Endian_Big);

    boost::scoped_ptr<StageSequentialIterator> unflipped_iter(reader.createSequentialIterator());
    boost::scoped_ptr<StageSequentialIterator> flipped_iter(filter.createSequentialIterator());

    const Schema& schema = filter.getSchema();
    
    PointBuffer flipped(schema, buffer_size);
    const boost::uint32_t fliped_read = flipped_iter->read(flipped);
    BOOST_CHECK_EQUAL(fliped_read, buffer_size);

    PointBuffer unflipped(schema, buffer_size);
    const boost::uint32_t unfliped_read = unflipped_iter->read(unflipped);
    BOOST_CHECK_EQUAL(unfliped_read, buffer_size);
    

    int offsetX = schema.getDimensionIndex(DimensionId::X_f64);
    int offsetY = schema.getDimensionIndex(DimensionId::Y_f64);
    int offsetZ = schema.getDimensionIndex(DimensionId::Z_f64);
    int offsetT = schema.getDimensionIndex(DimensionId::Time_u64);
    
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

    return;
}

BOOST_AUTO_TEST_SUITE_END()
