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
#include <pdal/filters/ByteSwap.hpp>

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

    pdal::filters::ByteSwap filter;
    filter.setInput(&reader);
    BOOST_CHECK_EQUAL(filter.getName(), "filters.byteswap");
    filter.prepare();

    Schema const& readerSchema = reader.getSchema();
    boost::optional<Dimension const&> readerdimX = readerSchema.getDimension("X");
    boost::optional<Dimension const&> readerdimY = readerSchema.getDimension("Y");
    boost::optional<Dimension const&> readerdimZ = readerSchema.getDimension("Z");
    boost::optional<Dimension const&> readerdimTime = readerSchema.getDimension("Time");

    BOOST_CHECK_EQUAL(readerdimX->getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(readerdimY->getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(readerdimZ->getEndianness(), pdal::Endian_Little);
    BOOST_CHECK_EQUAL(readerdimTime->getEndianness(), pdal::Endian_Little);


    Schema const& filterSchema = filter.getSchema();
    boost::optional<Dimension const&> filterdimX = filterSchema.getDimension("X");
    boost::optional<Dimension const&> filterdimY = filterSchema.getDimension("Y");
    boost::optional<Dimension const&> filterdimZ = filterSchema.getDimension("Z");
    boost::optional<Dimension const&> filterdimTime = filterSchema.getDimension("Time");

    BOOST_CHECK_EQUAL(filterdimX->getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filterdimY->getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filterdimZ->getEndianness(), pdal::Endian_Big);
    BOOST_CHECK_EQUAL(filterdimTime->getEndianness(), pdal::Endian_Big);


    const Schema& schema = filter.getSchema();

    PointBuffer flipped(schema, buffer_size);
    boost::scoped_ptr<StageSequentialIterator> flipped_iter(filter.createSequentialIterator(flipped));
    const boost::uint32_t fliped_read = flipped_iter->read(flipped);
    BOOST_CHECK_EQUAL(fliped_read, buffer_size);

    PointBuffer unflipped(schema, buffer_size);
    boost::scoped_ptr<StageSequentialIterator> unflipped_iter(reader.createSequentialIterator(unflipped));
    const boost::uint32_t unfliped_read = unflipped_iter->read(unflipped);
    BOOST_CHECK_EQUAL(unfliped_read, buffer_size);


    boost::optional<Dimension const&> unflippeddimX = unflipped.getSchema().getDimension("X");
    boost::optional<Dimension const&> unflippeddimY = unflipped.getSchema().getDimension("Y");
    boost::optional<Dimension const&> unflippeddimZ = unflipped.getSchema().getDimension("Z");
    boost::optional<Dimension const&> unflippeddimTime = unflipped.getSchema().getDimension("Time");

    boost::optional<Dimension const&> flippeddimX = flipped.getSchema().getDimension("X");
    boost::optional<Dimension const&> flippeddimY = flipped.getSchema().getDimension("Y");
    boost::optional<Dimension const&> flippeddimZ = flipped.getSchema().getDimension("Z");
    boost::optional<Dimension const&> flippeddimTime = flipped.getSchema().getDimension("Time");

    for (boost::uint32_t i = 0 ; i < buffer_size; ++i)
    {

        double unflipped_x = unflipped.getField<double>(*unflippeddimX, i);
        double unflipped_y = unflipped.getField<double>(*unflippeddimY, i);
        double unflipped_z = unflipped.getField<double>(*unflippeddimZ, i);
        boost::uint64_t unflipped_t = unflipped.getField<boost::uint64_t>(*unflippeddimTime, i);


        double flipped_x = flipped.getField<double>(*flippeddimX, i);
        double flipped_y = flipped.getField<double>(*flippeddimY, i);
        double flipped_z = flipped.getField<double>(*flippeddimZ, i);
        boost::uint64_t flipped_t = flipped.getField<boost::uint64_t>(*flippeddimTime, i);

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
}

BOOST_AUTO_TEST_SUITE_END()
