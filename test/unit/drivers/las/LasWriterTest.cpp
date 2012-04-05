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

#include "Support.hpp"

using namespace pdal;


BOOST_AUTO_TEST_SUITE(LasWriterTest)

BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_las)
{
    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_simple_las.las");
    FileUtils::deleteFile(temp_filename);

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(reader, ofs);
        BOOST_CHECK(writer.getDescription() == "Las Writer");
        writer.initialize();

        const boost::uint64_t numPoints = reader.getNumPoints();

        writer.setCompressed(false);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        writer.write(numPoints);
    }

    FileUtils::closeFile(ofs);

    bool filesSame = Support::compare_files(Support::temppath(temp_filename), Support::datapath("simple.las"));
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath(temp_filename));
    }

    return;
}

#ifdef PDAL_HAVE_LASZIP
BOOST_AUTO_TEST_CASE(LasWriterTest_test_simple_laz)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile("laszip/LasWriterTest_test_simple_laz.laz");

    pdal::drivers::las::Reader reader(Support::datapath("laszip/basefile.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath("LasWriterTest_test_simple_laz.laz"));

    {
        const boost::uint64_t numPoints = reader.getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(reader, ofs);
        writer.initialize();

        writer.setCompressed(true);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");
        writer.setHeaderPadding(2);

        writer.write(numPoints);
    }

    FileUtils::closeFile(ofs);

    {
        pdal::drivers::las::Reader reader(Support::temppath("LasWriterTest_test_simple_laz.laz"));
    }

    // these two files only differ by the description string in the VLR.
    // This now skips the entire LASzip VLR for comparison.
    const boost::uint32_t numdiffs = Support::diff_files(Support::temppath("LasWriterTest_test_simple_laz.laz"),
                                                         Support::datapath("laszip/laszip-generated.laz"),
                                                         227, 106);
    BOOST_CHECK(numdiffs==0);

    if (numdiffs==0)
    {
        FileUtils::deleteFile(Support::temppath("LasWriterTest_test_simple_laz.laz"));
    }

    return;
}
#endif

static void test_a_format(const std::string& refFile, boost::uint8_t majorVersion, boost::uint8_t minorVersion, int pointFormat)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile("temp.las");

    pdal::drivers::las::Reader reader(Support::datapath("1.2_3.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath("temp.las"));

    {
        const boost::uint64_t numPoints = reader.getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::las::Writer writer(reader, ofs);
        BOOST_CHECK(writer.getDescription() == "Las Writer");
        writer.initialize();

        writer.setCompressed(false);
        writer.setDate(78, 2008);
        writer.setPointFormat((::pdal::drivers::las::PointFormat)pointFormat);
        writer.setFormatVersion(majorVersion, minorVersion);
        writer.setSystemIdentifier("libLAS");
        writer.setGeneratingSoftware("libLAS 1.2");
        
        boost::uuids::uuid u = boost::lexical_cast<boost::uuids::uuid>("8388f1b8-aa1b-4108-bca3-6bc68e7b062e");
        writer.setProjectId(u);

        writer.write(numPoints);
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

    return;
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

    return;
}


BOOST_AUTO_TEST_SUITE_END()
