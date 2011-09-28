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
#include <pdal/external/boost/uuid/uuid_io.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/liblas/Writer.hpp>
#include <pdal/drivers/liblas/Reader.hpp>

#include "Support.hpp"

using namespace pdal;


BOOST_AUTO_TEST_SUITE(LiblasWriterTest)

BOOST_AUTO_TEST_CASE(LiblasWriterTest_test_simple_las)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile("LiblasWriterTest_test_simple_las.las");

    pdal::drivers::liblas::Reader reader(Support::datapath("1.2-with-color.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath("LiblasWriterTest_test_simple_las.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::liblas::Writer writer(reader, ofs);
        BOOST_CHECK(writer.getDescription() == "Liblas Writer");
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

    bool filesSame = Support::compare_files(Support::temppath("LiblasWriterTest_test_simple_las.las"), Support::datapath("simple.las"));
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("LiblasWriterTest_test_simple_las.las"));
    }

    return;
}

BOOST_AUTO_TEST_CASE(LiblasWriterTest_test_options)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile(Support::datapath("LiblasWriterTest_test_options.las"));

    Option opt("filename", Support::temppath("LiblasWriterTest_test_options.las"));
    Options opts(opt);

    pdal::drivers::liblas::Reader reader(Support::datapath("1.2-with-color.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::liblas::Writer writer(reader, opts);
        BOOST_CHECK(writer.getDescription() == "Liblas Writer");

        writer.initialize();

        writer.setCompressed(false);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        const boost::uint64_t numPoints = reader.getNumPoints();
        writer.write(numPoints);
    }

    bool filesSame = Support::compare_files(Support::temppath("LiblasWriterTest_test_options.las"), 
                                            Support::datapath("simple.las"));
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("LiblasWriterTest_test_options.las"));
    }

    return;
}


BOOST_AUTO_TEST_CASE(LiblasWriterTest_test_simple_laz)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile(Support::temppath("LiblasWriterTest_test_simple_laz.laz"));

    pdal::drivers::liblas::Reader reader(Support::datapath("laszip/basefile.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath("LiblasWriterTest_test_simple_laz.laz"));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::liblas::Writer writer(reader, ofs);
        writer.initialize();

        const boost::uint64_t numPoints = reader.getNumPoints();

        writer.setCompressed(true);
        writer.setDate(0, 0);
        writer.setPointFormat(::pdal::drivers::las::PointFormat3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");
        writer.setHeaderPadding(2);

        writer.write(numPoints);
    }

    FileUtils::closeFile(ofs);

    bool filesSame = Support::compare_files(Support::temppath("LiblasWriterTest_test_simple_laz.laz"), 
                                            Support::datapath("laszip/laszip-generated_with2bytespadding.laz"));
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("LiblasWriterTest_test_simple_laz.laz"));
    }

    return;
}


static void test_a_format(const std::string& refFile, boost::uint8_t majorVersion, boost::uint8_t minorVersion, int pointFormat)
{
    // remove file from earlier run, if needed
    FileUtils::deleteFile("temp.las");

    pdal::drivers::liblas::Reader reader(Support::datapath("1.2_3.las"));
    
    std::ostream* ofs = FileUtils::createFile(Support::temppath("temp.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream
        pdal::drivers::liblas::Writer writer(reader, ofs);
        BOOST_CHECK(writer.getDescription() == "Liblas Writer");
        writer.initialize();

        const boost::uint64_t numPoints = reader.getNumPoints();

        writer.setCompressed(false);
        writer.setDate(78, 2008);
        writer.setPointFormat((::pdal::drivers::las::PointFormat)pointFormat);
        writer.setFormatVersion(majorVersion, minorVersion);
        writer.setSystemIdentifier("libLAS");
        writer.setGeneratingSoftware("libLAS 1.2");
        
        pdal::external::boost::uuids::uuid u = boost::lexical_cast<pdal::external::boost::uuids::uuid>("8388f1b8-aa1b-4108-bca3-6bc68e7b062e");
        writer.setProjectId(u);

        writer.write(numPoints);
    }

    FileUtils::closeFile(ofs);

    const bool filesSame = Support::compare_files(Support::temppath("temp.las"), Support::datapath(refFile));
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath("temp.las"));
    }

    return;
}

BOOST_AUTO_TEST_CASE(test_different_formats)
{
    test_a_format("1.0_0_nosrs.las", 1, 0, 0);
    test_a_format("1.0_1_nosrs.las", 1, 0, 1);
    
    test_a_format("1.1_0_nosrs.las", 1, 1, 0);
    test_a_format("1.1_1_nosrs.las", 1, 1, 1);

    test_a_format("1.2_0_nosrs.las", 1, 2, 0);
    test_a_format("1.2_1_nosrs.las", 1, 2, 1);
    test_a_format("1.2_2_nosrs.las", 1, 2, 2);
    test_a_format("1.2_3_nosrs.las", 1, 2, 3);

    return;
}
BOOST_AUTO_TEST_SUITE_END()
