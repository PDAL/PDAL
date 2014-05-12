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
#include <boost/cstdint.hpp>

#include <pdal/StageIterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(LasReaderTest)


BOOST_AUTO_TEST_CASE(test_base_options)
{
    const std::string file(Support::datapath("1.2-with-color.las"));

    const pdal::Option opt_filename("filename", file);
    const pdal::Option opt_verbose_string("verbose", "99");
    const pdal::Option opt_verbose_uint8("verbose", 99);
    const pdal::Option opt_debug_string("debug", "true");
    const pdal::Option opt_debug_bool("debug", true);

    {
        pdal::Options opts;
        opts.add(opt_filename);

        pdal::drivers::las::Reader reader(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 0);
        BOOST_CHECK(reader.isDebug() == false);
    }

    {
        pdal::Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_string);
        opts.add(opt_debug_string);
        pdal::drivers::las::Reader reader(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 99);
        BOOST_CHECK(reader.isDebug() == true);
    }

    {
        pdal::Options opts;
        opts.add(opt_filename);
        opts.add(opt_verbose_uint8);
        opts.add(opt_debug_bool);
        pdal::drivers::las::Reader reader(opts);
        BOOST_CHECK(reader.getVerboseLevel() == 99);
        BOOST_CHECK(reader.isDebug() == true);
    }
}


BOOST_AUTO_TEST_CASE(test_sequential)
{
    PointContext ctx;

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);
    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();

    {
        PointBuffer data(ctx);
        point_count_t numRead = iter->read(data, 3);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    // Can we seek it? Yes, we can!
    iter->skip(97);
    {
        PointBuffer data(ctx);
        BOOST_CHECK(iter->getIndex() == 100);
        point_count_t numRead = iter->read(data, 3);
        BOOST_CHECK(numRead == 3);

        Support::check_p100_p101_p102(data);
    }
    delete iter;
}


/***
BOOST_AUTO_TEST_CASE(test_random)
{
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer data(schema);
    pdal::StageRandomIterator* iter = reader.createRandomIterator(data);

    {
        point_count_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    // Can we seek it? Yes, we can!
    iter->seek(100);
    {
        BOOST_CHECK(iter->getIndex() == 100);
        point_count_t numRead = iter->read(data, 3);
        BOOST_CHECK(numRead == 3);

        Support::check_p100_p101_p102(data);
    }

    // Can we seek to beginning? Yes, we can!
    iter->seek(0);
    {
        PointBuffer data(schema);
        BOOST_CHECK(iter->getIndex() == 0);
        point_count_t numRead = iter->read(data, 3);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    delete iter;
}


#ifdef PDAL_HAVE_LASZIP
BOOST_AUTO_TEST_CASE(test_random_laz)
{
    PointContext ctx;
    pdal::drivers::las::Reader reader(Support::datapath("laszip/laszip-generated.laz"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.initialize();
    reader.buildSchema(ctx.getSchema());
    Schema& schema = *(ctx.getSchema());

    const Schema& schema = reader.getSchema();

    PointBuffer data(schema, 3);

    pdal::StageRandomIterator* iter = reader.createRandomIterator(data);

    {
        point_count_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    // Can we seek it? Yes, we can!
    iter->seek(100);
    {
        BOOST_CHECK(iter->getIndex() == 100);
        point_count_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p100_p101_p102(data);
    }

    // Can we seek to beginning? Yes, we can!
    iter->seek(0);
    {
        BOOST_CHECK(iter->getIndex() == 0);
        point_count_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        Support::check_p0_p1_p2(data);
    }

    delete iter;
}
#endif
**/

BOOST_AUTO_TEST_CASE(test_two_iters)
{
    PointContext ctx;
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);

    BOOST_CHECK(reader.getNumPoints() == 1065);
    {
        pdal::StageSequentialIterator* iter = reader.createSequentialIterator();
        BOOST_CHECK(iter->getIndex() == 0);

        PointBuffer data(ctx);
        point_count_t numRead = iter->read(data, 1065);
        BOOST_CHECK(numRead == 1065);
        BOOST_CHECK(iter->getIndex() == 1065);

        Support::check_p0_p1_p2(data);

        delete iter;
    }

/**
    {
        pdal::StageRandomIterator* iter = reader.createRandomIterator(data);
        BOOST_CHECK(iter->getIndex() == 0);

        PointBuffer data(schema);
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1065);
        BOOST_CHECK(iter->getIndex() == 1065);

        Support::check_p0_p1_p2(data);

        delete iter;
    }
**/
}


BOOST_AUTO_TEST_CASE(test_simultaneous_iters)
{
    PointContext ctx;

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK_EQUAL(reader.getDescription(), "Las Reader");
    reader.prepare(ctx);

    BOOST_CHECK_EQUAL(reader.getNumPoints(), 1065);
    BOOST_CHECK_EQUAL(355 * 3, 1065);

    point_count_t numRead;

    pdal::StageSequentialIterator* iterS1 = reader.createSequentialIterator();
    BOOST_CHECK_EQUAL(iterS1->getIndex(), 0);

    pdal::StageSequentialIterator* iterS2 = reader.createSequentialIterator();
    BOOST_CHECK_EQUAL(iterS2->getIndex(), 0);

/**
    pdal::StageRandomIterator* iterR1 = reader.createRandomIterator(data);
    BOOST_CHECK_EQUAL(iterR1->getIndex(), 0);

    pdal::StageRandomIterator* iterR2 = reader.createRandomIterator(data);
    BOOST_CHECK_EQUAL(iterR2->getIndex(), 0);
**/

    {
        PointBuffer data(ctx);
        numRead = iterS1->read(data, 355);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterS1->getIndex(), 355);

        Support::check_p0_p1_p2(data);
    }

    {
        iterS2->skip(355);

        PointBuffer data(ctx);
        numRead = iterS2->read(data, 355);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterS2->getIndex(), 710);

        Support::check_p355_p356_p357(data);
    }

/**
    {
        iterR1->seek(355);
        numRead = iterR1->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterR1->getIndex(), 710);

        Support::check_p355_p356_p357(data);
    }

    {
        iterR2->seek(0);
        numRead = iterR2->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterR2->getIndex(), 355);

        Support::check_p0_p1_p2(data);
    }
**/

    {
        PointBuffer data(ctx);
        iterS1->skip(355);
        numRead = iterS1->read(data, 355);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterS1->getIndex(), 1065);

        Support::check_p710_p711_p712(data);
    }

    {
        PointBuffer data(ctx);
        iterS2->skip(0);
        numRead = iterS2->read(data, 355);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterS2->getIndex(), 1065);

        Support::check_p710_p711_p712(data);
    }

/**
    {
        iterR1->seek(355);
        numRead = iterR1->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterR1->getIndex(), 710);

        Support::check_p355_p356_p357(data);
    }

    {
        iterR2->seek(710);
        numRead = iterR2->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterR2->getIndex(), 1065);

        Support::check_p710_p711_p712(data);
    }

    {
        iterR1->seek(0);
        numRead = iterR1->read(data);
        BOOST_CHECK_EQUAL(numRead, 355);
        BOOST_CHECK_EQUAL(iterR1->getIndex(), 355);

        Support::check_p0_p1_p2(data);
    }
**/

    delete iterS1;
    delete iterS2;
/**
    delete iterR1;
    delete iterR2;
**/
}

static void test_a_format(const std::string& file, boost::uint8_t majorVersion, boost::uint8_t minorVersion, int pointFormat,
                          double xref, double yref, double zref, double tref, boost::uint16_t rref,  boost::uint16_t gref,  boost::uint16_t bref)
{
    PointContext ctx;

    pdal::drivers::las::Reader reader(Support::datapath(file));
    reader.prepare(ctx);

    BOOST_CHECK_EQUAL(reader.getLasHeader().getPointFormat(), pointFormat);
    BOOST_CHECK_EQUAL(reader.getLasHeader().GetVersionMajor(), majorVersion);
    BOOST_CHECK_EQUAL(reader.getLasHeader().GetVersionMinor(), minorVersion);

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();

    {
        PointBuffer data(ctx);
        point_count_t numRead = iter->read(data, 1);
        BOOST_CHECK_EQUAL(numRead, 1);

        Support::check_pN(data, 0, xref, yref, zref, tref, rref, gref, bref);
    }

    delete iter;
}

BOOST_AUTO_TEST_CASE(test_different_formats)
{
    test_a_format("1.0_0.las", 1, 0, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("1.0_1.las", 1, 0, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("1.1_0.las", 1, 1, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("1.1_1.las", 1, 1, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);

    test_a_format("1.2_0.las", 1, 2, 0, 470692.440000, 4602888.900000, 16.000000, 0, 0, 0, 0);
    test_a_format("1.2_1.las", 1, 2, 1, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 0, 0, 0);
    test_a_format("1.2_2.las", 1, 2, 2, 470692.440000, 4602888.900000, 16.000000, 0, 255, 12, 234);
    test_a_format("1.2_3.las", 1, 2, 3, 470692.440000, 4602888.900000, 16.000000, 1205902800.000000, 255, 12, 234);
}


BOOST_AUTO_TEST_CASE(test_vlr)
{
    PointContext ctx;

    pdal::drivers::las::Reader reader(Support::datapath("lots_of_vlr.las"));
    reader.prepare(ctx);

    BOOST_CHECK_EQUAL(reader.getLasHeader().getVLRs().getAll().size(), 390);
}


BOOST_AUTO_TEST_CASE(test_no_xyz)
{
    PointContext ctx;

    // Wipe off the XYZ dimensions and see if we can 
    // still read LAS data #123
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);

    Schema& schema = *(ctx.getSchema());
    
    Dimension x = schema.getDimension("X");
    boost::uint32_t flags = x.getFlags();
    x.setFlags(flags | dimension::IsIgnored);
    schema.setDimension(x);

    Dimension y = schema.getDimension("Y");
    flags = y.getFlags();
    y.setFlags(flags | dimension::IsIgnored);
    schema.setDimension(y);

    Dimension z = schema.getDimension("Z");
    flags = z.getFlags();
    z.setFlags(flags | dimension::IsIgnored);
    schema.setDimension(z);
        
//ABELL - No more packing.
//    schema = schema.pack(); // wipe out ignored dims.

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();

    {
        PointBuffer data(ctx);
        point_count_t numRead = iter->read(data, 3);
        BOOST_CHECK(numRead == 3);
    }
    delete iter;
}


BOOST_AUTO_TEST_CASE(testFilenameConstructorSetOption)
{
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK_EQUAL(
        reader.getOptions().getValueOrDefault<std::string>("filename", ""),
        Support::datapath("1.2-with-color.las"));
}


BOOST_AUTO_TEST_CASE(testInvalidFileSignature)
{
    PointContext ctx;
    pdal::drivers::las::Reader reader(
        Support::datapath("1.2-with-color.las.wkt"));
    try
    {
        reader.prepare(ctx);
    }
    catch (const std::invalid_argument& e)
    {
        std::string msg(e.what());
        BOOST_CHECK(msg.find("1.2-with-color.las.wkt") != std::string::npos);
        return;
    }
    BOOST_FAIL("reader.initialize() did not throw std::invalid_argument");
}


BOOST_AUTO_TEST_SUITE_END()
