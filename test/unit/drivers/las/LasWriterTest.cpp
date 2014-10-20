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
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/concept_check.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include <pdal/PointBuffer.hpp>

#include "StageTester.hpp"
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(LasWriterTest)

BOOST_AUTO_TEST_CASE(auto_offset)
{
    using namespace Dimension;

    const std::string FILENAME(Support::temppath("offset_test.las"));
    PointContext ctx;

    ctx.registerDim(Id::X);

    PointBufferPtr buf(new PointBuffer(ctx));
    buf->setField(Id::X, 0, 125000.00);
    buf->setField(Id::X, 1, 74529.00);
    buf->setField(Id::X, 2, 523523.02);

    Options writerOps;
    writerOps.add("filename", FILENAME);
    writerOps.add("offset_x", "auto");

    drivers::las::Writer writer;
    writer.setOptions(writerOps);

    writer.prepare(ctx);

    WriterTester::ready(&writer, ctx);
    WriterTester::write(&writer, *buf);
    WriterTester::done(&writer, ctx);

    Options readerOps;
    readerOps.add("filename", FILENAME);

    PointContext readCtx;

    drivers::las::Reader reader;
    reader.setOptions(readerOps);

    reader.prepare(readCtx);
    BOOST_CHECK_CLOSE(reader.header().offsetX(), 74529.00, .0001);
    PointBufferSet pbSet = reader.execute(readCtx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 3);
    BOOST_CHECK_CLOSE(buf->getFieldAs<double>(Id::X, 0), 125000.00, .0001);
    BOOST_CHECK_CLOSE(buf->getFieldAs<double>(Id::X, 1), 74529.00, .0001);
    BOOST_CHECK_CLOSE(buf->getFieldAs<double>(Id::X, 2), 523523.02, .0001);
    FileUtils::deleteFile(FILENAME);
}


//ABELL
/**
BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_las)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_simple_las.las");
    FileUtils::deleteFile(temp_filename);

    Options readerOpts;
    
    readerOpts.add("filename", Support::datapath("las/1.2-with-color.las"));

    Options writerOpts;
    writerOpts.add("creation_year", 0);
    writerOpts.add("creation_doy", 0);
    writerOpts.add("software_id", "TerraScan");

    drivers::las::Reader reader;
    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    drivers::las::Writer writer(ofs);
    writer.setOptions(writerOpts);
    writer.setInput(&reader);
    BOOST_CHECK_EQUAL(writer.getDescription(), "Las Writer");
    writer.prepare(ctx);
    writer.execute(ctx);

    FileUtils::closeFile(ofs);

    bool filesSame = Support::compare_files(Support::temppath(temp_filename),
        Support::datapath("simple.las"));
    BOOST_CHECK_EQUAL(filesSame, true);

    if (filesSame)
        FileUtils::deleteFile(Support::temppath(temp_filename));
}
**/

//ABELL
/**
BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_laz)
{
    PointContext ctx;

    WriterOpts writerOpts;
    writerOpts.add("compressed", true);
    writerOpts.add("creation_year", 0);
    writerOpts.add("creation_doy", 0);
    writerOpts.add("system_id", "");
    writerOpts.add("software_id", "TerraScan");


    // remove file from earlier run, if needed
    FileUtils::deleteFile("laszip/LasWriterTest_test_simple_laz.laz");

    drivers::las::Reader reader(Support::datapath("laszip/basefile.las"));

    std::ostream* ofs = FileUtils::createFile(
        Support::temppath("LasWriterTest_test_simple_laz.laz"));

    // need to scope the writer, so that's it dtor can use the stream
    drivers::las::Writer writer(ofs);
    writer.setOptions(writer);
    writer.setInput(&reader);

    writer.prepare(ctx);
    writer.execute(ctx);

    FileUtils::closeFile(ofs);

    {
        pdal::drivers::las::Reader reader(
            Support::temppath("LasWriterTest_test_simple_laz.laz"));
    }

    // these two files only differ by the description string in the VLR.
    // This now skips the entire LASzip VLR for comparison.
    const boost::uint32_t numdiffs =Support::diff_files(
        Support::temppath("LasWriterTest_test_simple_laz.laz"),
        Support::datapath("laszip/laszip-generated.laz"),
        227, 106);
    BOOST_CHECK_EQUAL(numdiffs, 0);

    if (numdiffs == 0)
        FileUtils::deleteFile(
            Support::temppath("LasWriterTest_test_simple_laz.laz"));
}
**/

//ABELL
/**
static void test_a_format(const std::string& refFile, uint8_t majorVersion,
    uint8_t minorVersion, int pointFormat)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    FileUtils::deleteFile("temp.las");

    std::string directory = "las/permutations/";
    Options readerOpts;
    readerOpts.add("filename", Support::datapath(directory + "1.2_3.las"));

    drivers::las::Reader reader;
    reader.setOptions(readerOpts);

    Options writerOpts;
    writerOpts.add("compression", false);
    writerOpts.add("creation_doy", 78);
    writerOpts.add("creation_year", 2008);
    writerOpts.add("format", pointFormat);
    writerOpts.add("minor_version", (unsigned)minorVersion);
    writerOpts.add("system_id", "libLAS");
    writerOpts.add("software_id", "libLAS 1.2");
    writerOpts.add("project_id", boost::lexical_cast<boost::uuids::uuid>(
        "8388f1b8-aa1b-4108-bca3-6bc68e7b062e"));

    std::ostream* ofs = FileUtils::createFile(Support::temppath("temp.las"));

    // need to scope the writer, so that's it dtor can use the stream
    drivers::las::Writer writer(ofs);
    writer.setOptions(writerOpts);
    writer.setInput(&reader);
    BOOST_CHECK_EQUAL(writer.getDescription(), "Las Writer");

    writer.prepare(ctx);
    writer.execute(ctx);

    bool filesSame = Support::compare_files("temp.las",
        Support::datapath(directory + refFile));
    BOOST_CHECK(filesSame);
    if (filesSame)
        FileUtils::deleteFile(Support::temppath("temp.las"));
    else
        exit(0);
}
**/


//ABELL
/**
BOOST_AUTO_TEST_CASE(version1_0)
{
    test_a_format("1.0_0.las", 1, 0, 0);
    test_a_format("1.0_1.las", 1, 0, 1);
}


BOOST_AUTO_TEST_CASE(version1_1)
{
    test_a_format("1.1_0.las", 1, 1, 0);
    test_a_format("1.1_1.las", 1, 1, 1);
}


BOOST_AUTO_TEST_CASE(version1_2)
{
    test_a_format("1.2_0.las", 1, 2, 0);
    test_a_format("1.2_1.las", 1, 2, 1);
    test_a_format("1.2_2.las", 1, 2, 2);
    test_a_format("1.2_3.las", 1, 2, 3);
}
**/


//ABELL
/**
BOOST_AUTO_TEST_CASE(test_summary_data_add_point)
{
    drivers::las::SummaryData summaryData;

    summaryData.addPoint(-95.329381929535259, 29.71948951835612,
        -17.515486778166398, 0);
    BOX3D b = summaryData.getBounds();
    BOOST_CHECK_EQUAL(b.minx, b.maxx);
    BOOST_CHECK_EQUAL(b.minz, b.maxz);
}
**/


//ABELL
/**
BOOST_AUTO_TEST_CASE(LasWriterTest_test_drop_extra_returns)
{
    using namespace pdal;

    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_drop_extra_returns.las");
    FileUtils::deleteFile(Support::temppath(temp_filename));

    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("num_points", 100);
    ops.add("mode", "constant");
    ops.add("number_of_returns", 10);
    drivers::faux::Reader reader;
    reader.setOptions(ops);

    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    Options writerOptions;
    writerOptions.add("discard_high_return_numbers", true);
    writerOptions.add("compression", false);
    writerOptions.add("creation_year", 0);
    writerOptions.add("creation_doy", 0);
    writerOptions.add("system_id", "");
    writerOptions.add("software_id", "TerraScan");

    drivers::las::Writer writer(ofs);
    writer.setOptions(writerOptions);
    writer.setInput(&reader);
    writer.prepare(ctx);
    writer.execute(ctx);

    Options readerOptions;
    readerOptions.add("filename", Support::temppath(temp_filename));
    readerOptions.add("count", 6);

    drivers::las::Reader reader2;
    reader2.setOptions(readerOptions);

    PointContext ctx2;

    reader2.prepare(ctx2);
    PointBufferSet pbSet = reader2.execute(ctx2);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    uint8_t r1 = buf->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 0);
    BOOST_CHECK_EQUAL(r1, 1);
    uint8_t r2 = buf->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 5);
    BOOST_CHECK_EQUAL(r2, 1);
    uint8_t n1 = buf->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 0);
    BOOST_CHECK_EQUAL(n1, 5);
    uint8_t n2 = buf->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 5);
    BOOST_CHECK_EQUAL(n1, 5);

    FileUtils::closeFile(ofs);
    FileUtils::deleteFile(Support::temppath(temp_filename));
}
**/

BOOST_AUTO_TEST_SUITE_END()
