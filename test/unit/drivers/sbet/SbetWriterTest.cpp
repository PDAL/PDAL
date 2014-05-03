/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <pdal/drivers/sbet/Reader.hpp>
#include <pdal/drivers/sbet/Writer.hpp>

#include "Support.hpp"


pdal::Options makeReaderOptions()
{
    pdal::Options options;
    pdal::Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    options.add(filename);
    return options;
}


pdal::Options makeWriterOptions()
{
    pdal::Options options;
    pdal::Option filename("filename",
                          Support::temppath("SbetWriterTest.sbet"), "");
    options.add(filename);
    return options;
}


BOOST_AUTO_TEST_SUITE(SbetWriterTest)


BOOST_AUTO_TEST_CASE(testConstructor)
{
    pdal::drivers::sbet::Reader reader(makeReaderOptions());
    pdal::drivers::sbet::Writer writer(makeWriterOptions());
    writer.setInput(&reader);

    BOOST_CHECK(writer.getDescription() == "SBET Writer");
    BOOST_CHECK_EQUAL(writer.getName(), "drivers.sbet.writer");

    writer.initialize();
}


BOOST_AUTO_TEST_CASE(testWrite)
{
    {
        pdal::drivers::sbet::Reader reader(makeReaderOptions());
        pdal::drivers::sbet::Writer writer(makeWriterOptions());
        writer.setInput(&reader);
        writer.initialize();

        writer.write(2);
    }

    BOOST_CHECK(Support::compare_files(
                    Support::temppath("SbetWriterTest.sbet"),
                    Support::datapath("sbet/2-points.sbet")));

    pdal::FileUtils::deleteFile(
        Support::temppath("SbetWriterTest.sbet"));
}


BOOST_AUTO_TEST_SUITE_END()
