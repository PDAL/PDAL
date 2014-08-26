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
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/concept_check.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(LasWriterTest)

BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_las)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_simple_las.las");
    FileUtils::deleteFile(temp_filename);

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(ofs);
        writer.setInput(&reader);
        BOOST_CHECK_EQUAL(writer.getDescription(), "Las Writer");
        writer.prepare(ctx);

        const boost::uint64_t numPoints = reader.getNumPoints();

        writer.setCompressed(false);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        writer.execute(ctx);
    }

    FileUtils::closeFile(ofs);

    bool filesSame = Support::compare_files(Support::temppath(temp_filename),
        Support::datapath("simple.las"));
    BOOST_CHECK_EQUAL(filesSame, true);

    if (filesSame)
        FileUtils::deleteFile(Support::temppath(temp_filename));
}

#ifdef PDAL_HAVE_LASZIP
BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_laz)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    FileUtils::deleteFile("laszip/LasWriterTest_test_simple_laz.laz");

    pdal::drivers::las::Reader reader(Support::datapath("laszip/basefile.las"));

    std::ostream* ofs = FileUtils::createFile(
        Support::temppath("LasWriterTest_test_simple_laz.laz"));
    {
        const boost::uint64_t numPoints = reader.getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(ofs);
        writer.setInput(&reader);

        writer.setCompressed(true);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");
        writer.setHeaderPadding(2);

        writer.prepare(ctx);
        writer.execute(ctx);
    }

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
#endif

static void test_a_format(const std::string& refFile,
    boost::uint8_t majorVersion, boost::uint8_t minorVersion, int pointFormat)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    FileUtils::deleteFile("temp.las");

    pdal::drivers::las::Reader reader(Support::datapath("1.2_3.las"));

    std::ostream* ofs = FileUtils::createFile(Support::temppath("temp.las"));

    {
        const boost::uint64_t numPoints = reader.getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(ofs);
        writer.setInput(&reader);
        BOOST_CHECK_EQUAL(writer.getDescription(), "Las Writer");

        writer.setCompressed(false);
        writer.setDate(78, 2008);
        writer.setPointFormat((::pdal::drivers::las::PointFormat)pointFormat);
        writer.setFormatVersion(majorVersion, minorVersion);
        writer.setSystemIdentifier("libLAS");
        writer.setGeneratingSoftware("libLAS 1.2");

        boost::uuids::uuid u =
            boost::lexical_cast<boost::uuids::uuid>(
                "8388f1b8-aa1b-4108-bca3-6bc68e7b062e");
        writer.setProjectId(u);
        writer.prepare(ctx);
        writer.execute(ctx);
    }

    FileUtils::closeFile(ofs);

    // BUG: the following test commented out as per ticket #35
    boost::ignore_unused_variable_warning(refFile);
    //const bool filesSame = Support::compare_files("temp.las", Support::datapath(refFile));
    //BOOST_CHECK(filesSame);
    //
    //if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("temp.las"));
    }
}

//ABELL
/**
BOOST_AUTO_TEST_CASE(LasWriterTest_test_metadata)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename(Support::temppath("temp-LasWriterTest_test_metadata.las"));
    FileUtils::deleteFile(temp_filename);

    pdal::Options options;
    pdal::Option filename("filename", temp_filename);
    pdal::Option metadata_option("metadata", "");
    pdal::Option software_id("software_id", "forward");
    pdal::Option system_id("system_id", "forward");
    pdal::Option creation_doy("creation_doy", "forward");
    pdal::Option creation_year("creation_year", "forward");

    pdal::Option polygon("vlr", "forward");
    pdal::Option record_id("record_id", 1234);
    pdal::Option user_id("user_id", "hobu");
    pdal::Option global_encoding("global_encoding", "AQA=");
    
    pdal::Options vlr_opts;
    vlr_opts.add(record_id);
    vlr_opts.add(user_id);
    polygon.setOptions(vlr_opts);
    

    pdal::Option debug("debug", true);
    pdal::Option verbosity("verbose", 7);

    pdal::Options moptions;
    moptions.add(software_id);
    moptions.add(system_id);
    moptions.add(creation_doy);
    moptions.add(creation_year);
    moptions.add(polygon);
    moptions.add(global_encoding);
    metadata_option.setOptions(moptions);

    options.add(metadata_option);
    options.add(filename);
    // options.add(debug);
    // options.add(verbosity);
    
    pdal::drivers::las::Reader reader(Support::datapath("interesting.las"));

    {
        pdal::drivers::las::Writer writer(options);
        writer.setInput(&reader);
        writer.prepare(ctx);
        writer.execute(ctx);
    }


    PointContext ctx2;
    pdal::drivers::las::Reader reader2(options);
    reader2.prepare(ctx2);
    
    pdal::drivers::las::LasHeader const& h = reader2.getLasHeader();
    
    BOOST_CHECK_EQUAL(h.GetSoftwareId(), "HOBU-GENERATING");
    BOOST_CHECK_EQUAL(h.GetSystemId(), "HOBU-SYSTEMID");
    BOOST_CHECK_EQUAL(h.GetCreationDOY(), 145u);
    BOOST_CHECK_EQUAL(h.GetCreationYear(), 2012u);
    
    pdal::drivers::las::VLRList const& vlrs = h.getVLRs();
#ifdef PDAL_SRS_ENABLED
    BOOST_CHECK_EQUAL(vlrs.count(), 5u);
#else
    BOOST_CHECK_EQUAL(vlrs.count(), 2u);
#endif
    pdal::drivers::las::VariableLengthRecord const& r = vlrs.get(0);
    
    BOOST_CHECK_EQUAL(r.getRecordId(), 1234u);
    BOOST_CHECK_EQUAL(r.getUserId(), "hobu");
    BOOST_CHECK_EQUAL(r.getLength(), 70);    
    
    FileUtils::deleteFile(temp_filename);
}
**/

BOOST_AUTO_TEST_CASE(LasWriterTest_test_ignored_dimensions)
{
    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename(
        Support::temppath("temp-LasWriterTest_test_ignored_dimensions.las"));
    FileUtils::deleteFile(temp_filename);

    pdal::Options options;
    pdal::Option filename("filename", temp_filename);
    pdal::Option debug("debug", true);
    pdal::Option verbosity("verbose", 7);

    pdal::Option ignored;
    pdal::Options mignored;
    pdal::Option keep;
    pdal::Options mkeep;
    
    mkeep.add(pdal::Option("dimension", "X"));
    mkeep.add(pdal::Option("dimension", "Y"));
    
    mignored.add(pdal::Option("dimension", "Red"));
    mignored.add(pdal::Option("dimension", "Green"));
    mignored.add(pdal::Option("dimension", "Blue"));
    
    keep.setOptions(mkeep);
    ignored.setOptions(mignored);

    options.add(ignored);
    options.add(filename);
    options.add(debug);
    
    pdal::drivers::las::Reader reader(Support::datapath("interesting.las"));

    {
        pdal::drivers::las::Writer writer(options);
        writer.setInput(&reader);
        writer.prepare(ctx);
        writer.execute(ctx);
    }

    {
        PointContext ctx2;

        pdal::drivers::las::Reader reader2(options);
        reader2.prepare(ctx2);
        PointBuffer altered(ctx2);

        pdal::StageSequentialIterator* iter =
            reader2.createSequentialIterator();
        iter->read(altered, 1);
    
        uint16_t r = altered.getFieldAs<uint16_t>(Dimension::Id::Red, 0);
        BOOST_CHECK_EQUAL(r, 0u);
        int32_t x = altered.getFieldAs<int32_t>(Dimension::Id::X, 0);
        BOOST_CHECK_EQUAL(x, 63701224);

        delete iter;
    }
    
    {
        PointContext ctx3;

        drivers::las::Reader reader3(Support::datapath("interesting.las"));
        reader3.prepare(ctx3);
        PointBuffer original(ctx3);
        StageSequentialIterator* iter2 = reader.createSequentialIterator();
    
        iter2->read(original, 1);
        
        uint16_t r2 = original.getFieldAs<uint16_t>(Dimension::Id::Red, 0);
        BOOST_CHECK_EQUAL(r2, 68u);
        int32_t x2 = original.getFieldAs<int32_t>(Dimension::Id::X, 0);
        BOOST_CHECK_EQUAL(x2, 63701224);

        delete iter2;
    }

    FileUtils::deleteFile(temp_filename);
}

BOOST_AUTO_TEST_CASE(test_different_formats)
{
    test_a_format("1.0_0.las", 1, 0, 0);
    test_a_format("1.0_1.las", 1, 0, 1);

    test_a_format("1.1_0.las", 1, 1, 0);
    test_a_format("1.1_1.las", 1, 1, 1);

    test_a_format("1.2_0.las", 1, 2, 0);
    test_a_format("1.2_1.las", 1, 2, 1);
    test_a_format("1.2_2.las", 1, 2, 2);
    test_a_format("1.2_3.las", 1, 2, 3);
}


BOOST_AUTO_TEST_CASE(test_summary_data_add_point)
{
    drivers::las::SummaryData summaryData;

    summaryData.addPoint(-95.329381929535259, 29.71948951835612,
        -17.515486778166398, 0);
    Bounds<double> b = summaryData.getBounds();
    BOOST_CHECK_EQUAL(b.getMinimum(0), b.getMaximum(0));
    BOOST_CHECK_EQUAL(b.getMinimum(2), b.getMaximum(2));
}


BOOST_AUTO_TEST_CASE(LasWriterTest_test_drop_extra_returns)
{
    using namespace pdal;

    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_drop_extra_returns.las");
    FileUtils::deleteFile(Support::temppath(temp_filename));

    Options ops;

    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("num_points", 100);
    ops.add("mode", "constant");
    ops.add("number_of_returns", 10);
    drivers::faux::Reader reader(ops);

    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    {
        Options writerOptions;
        writerOptions.add("discard_high_return_numbers", true);
        pdal::drivers::las::Writer writer(writerOptions, ofs);
        writer.setInput(&reader);
        writer.prepare(ctx);

        writer.setCompressed(false);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        writer.execute(ctx);
    }

    {
        Options readerOptions;
        readerOptions.add("filename", Support::temppath(temp_filename));

        pdal::drivers::las::Reader reader2(readerOptions);

        PointContext ctx2;
        reader2.prepare(ctx2);
        PointBuffer data(ctx2);

        pdal::StageSequentialIterator* iter = 
            reader2.createSequentialIterator();
        iter->read(data, 6);
    
        uint8_t r1 = data.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 0);
        BOOST_CHECK_EQUAL(r1, 1);
        uint8_t r2 = data.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 5);
        BOOST_CHECK_EQUAL(r2, 1);
        uint8_t n1 = data.getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 0);
        BOOST_CHECK_EQUAL(n1, 5);
        uint8_t n2 = data.getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 5);
        BOOST_CHECK_EQUAL(n1, 5);

        delete iter;
    }

    FileUtils::closeFile(ofs);
    FileUtils::deleteFile(Support::temppath(temp_filename));
}


BOOST_AUTO_TEST_SUITE_END()
