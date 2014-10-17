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

#include <boost/cstdint.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(LasReaderTest)


BOOST_AUTO_TEST_CASE(test_base_options)
{
    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const pdal::Option opt_filename("filename", file);
    const pdal::Option opt_verbose_string("verbose", "99");
    const pdal::Option opt_verbose_uint8("verbose", 99);
    const pdal::Option opt_debug_string("debug", "true");
    const pdal::Option opt_debug_bool("debug", true);

    {
        pdal::Options opts;
        opts.add(opt_filename);

        pdal::drivers::las::Reader reader;
        reader.setOptions(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 0);
        BOOST_CHECK(reader.isDebug() == false);
    }

    {
        pdal::Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_string);
        opts.add(opt_debug_string);
        pdal::drivers::las::Reader reader;
        reader.setOptions(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 99);
        BOOST_CHECK(reader.isDebug() == true);
    }

    {
        pdal::Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_uint8);
        opts.add(opt_debug_bool);
        pdal::drivers::las::Reader reader;
        reader.setOptions(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 99);
        BOOST_CHECK(reader.isDebug() == true);
    }
}


BOOST_AUTO_TEST_CASE(header)
{
    PointContext ctx;
    Options ops;
    ops.add("filename", Support::datapath("las/simple.las"));
    pdal::drivers::las::Reader reader;
    reader.setOptions(ops);

    reader.prepare(ctx);
    // This tests the copy ctor, too.
    drivers::las::LasHeader h = reader.getLasHeader();

    BOOST_CHECK_EQUAL(h.GetFileSignature(), "LASF");
    BOOST_CHECK_EQUAL(h.GetFileSourceId(), 0);
    BOOST_CHECK_EQUAL(h.GetReserved(), 0);
    BOOST_CHECK(h.GetProjectId().is_nil());
    BOOST_CHECK_EQUAL(h.GetVersionMajor(), 1);
    BOOST_CHECK_EQUAL(h.GetVersionMinor(), 2);
    BOOST_CHECK_EQUAL(h.GetCreationDOY(), 0);
    BOOST_CHECK_EQUAL(h.GetCreationYear(), 0);
    BOOST_CHECK_EQUAL(h.GetHeaderSize(), 227);
    BOOST_CHECK_EQUAL(h.getPointFormat(), 3);
    BOOST_CHECK_EQUAL(h.GetPointRecordsCount(), 1065);
    BOOST_CHECK_EQUAL(h.GetScaleX(), .01);
    BOOST_CHECK_EQUAL(h.GetScaleY(), .01);
    BOOST_CHECK_EQUAL(h.GetScaleZ(), .01);
    BOOST_CHECK_EQUAL(h.GetOffsetX(), 0);
    BOOST_CHECK_EQUAL(h.GetOffsetY(), 0);
    BOOST_CHECK_EQUAL(h.GetOffsetZ(), 0);
    BOOST_CHECK_CLOSE(h.GetMaxX(), 638982.55, .01);
    BOOST_CHECK_CLOSE(h.GetMaxY(), 853535.43, .01);
    BOOST_CHECK_CLOSE(h.GetMaxZ(), 586.38, .01);
    BOOST_CHECK_CLOSE(h.GetMinX(), 635619.85, .01);
    BOOST_CHECK_CLOSE(h.GetMinY(), 848899.70, .01);
    BOOST_CHECK_CLOSE(h.GetMinZ(), 406.59, .01);
    BOOST_CHECK_EQUAL(h.Compressed(), false);
    BOOST_CHECK_EQUAL(h.getVLRBlockSize(), 0);
    BOOST_CHECK_EQUAL(h.getCompressionInfo(), "");
    BOOST_CHECK_EQUAL(h.GetHeaderPadding(), 0);
}


BOOST_AUTO_TEST_CASE(test_sequential)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    ops1.add("count", 103);
    pdal::drivers::las::Reader reader;
    reader.setOptions(ops1);

    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);
    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    Support::check_p0_p1_p2(*buf);
    PointBufferPtr buf2 = buf->makeNew();
    buf2->appendPoint(*buf, 100);
    buf2->appendPoint(*buf, 101);
    buf2->appendPoint(*buf, 102);
    Support::check_p100_p101_p102(*buf2);
}


static void test_a_format(const std::string& file, boost::uint8_t majorVersion, boost::uint8_t minorVersion, int pointFormat,
                          double xref, double yref, double zref, double tref, boost::uint16_t rref,  boost::uint16_t gref,  boost::uint16_t bref)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath(file));
    ops1.add("count", 1);
    pdal::drivers::las::Reader reader;
    reader.setOptions(ops1);
    reader.prepare(ctx);

    BOOST_CHECK_EQUAL(reader.getLasHeader().getPointFormat(), pointFormat);
    BOOST_CHECK_EQUAL(reader.getLasHeader().GetVersionMajor(), majorVersion);
    BOOST_CHECK_EQUAL(reader.getLasHeader().GetVersionMinor(), minorVersion);

    PointBufferSet pbSet = reader.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 1);

    Support::check_pN(*buf, 0, xref, yref, zref, tref, rref, gref, bref);
}

BOOST_AUTO_TEST_CASE(test_different_formats)
{
    test_a_format("las/permutations/1.0_0.las", 1, 0, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.0_1.las", 1, 0, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("las/permutations/1.1_0.las", 1, 1, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.1_1.las", 1, 1, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("las/permutations/1.2_0.las", 1, 2, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("las/permutations/1.2_1.las", 1, 2, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);
    test_a_format("las/permutations/1.2_2.las", 1, 2, 2, 470692.440000, 4602888.900000, 16.000000, 0, 255, 12, 234);
    test_a_format("las/permutations/1.2_3.las", 1, 2, 3, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 255, 12, 234);
}


BOOST_AUTO_TEST_CASE(test_vlr)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/lots_of_vlr.las"));
    pdal::drivers::las::Reader reader;
    reader.setOptions(ops1);
    reader.prepare(ctx);

    BOOST_CHECK_EQUAL(reader.getLasHeader().getVLRs().getAll().size(), 390);
}


BOOST_AUTO_TEST_CASE(testInvalidFileSignature)
{
    PointContext ctx;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las.wkt"));
    pdal::drivers::las::Reader reader;
    reader.setOptions(ops1);
    try
    {
        reader.prepare(ctx);
    }
    catch (const std::invalid_argument& e)
    {
        std::string msg(e.what());
        BOOST_CHECK(msg.find("las/1.2-with-color.las.wkt") != std::string::npos);
        return;
    }
    BOOST_FAIL("reader.initialize() did not throw std::invalid_argument");
}

BOOST_AUTO_TEST_SUITE_END()
