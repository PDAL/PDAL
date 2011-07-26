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

#include <sstream>
#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include <pdal/StreamOwner.hpp>
#include <pdal/Utils.hpp>
#include <pdal/exceptions.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StreamOwnerTest)

BOOST_AUTO_TEST_CASE(StreamOwnerTest_test1)
{
    const std::string rfile = Support::datapath("1.2-with-color.las");
    const std::string wfile = "temp.txt";

    // remove file from earlier run, if needed
    Utils::deleteFile(wfile);
    BOOST_CHECK(!Utils::fileExists(wfile));

    {
        // filename, reading
        pdal::IStreamOwner file1(rfile);
        BOOST_CHECK(file1.istream() != NULL);
        BOOST_CHECK(file1.getFileName() == rfile);
    }

    {
        // filename, reading -- should throw
        bool ok = false;
        try
        {
            pdal::IStreamOwner file1x("sillyfilenamethatdoesnotexist.xml");
            ok = false;
        }
        catch (pdal::pdal_error ex)
        {
            ok = true;
        }
        BOOST_CHECK(ok);
    }

    {
        // filename, writing
        pdal::OStreamOwner file2(wfile);
        BOOST_CHECK(file2.ostream() != NULL);
        BOOST_CHECK(file2.getFileName() == wfile);

        BOOST_CHECK(Utils::fileExists(wfile));
    }

    {
        // stream, reading
        pdal::IStreamOwner file3(std::cin);
        BOOST_CHECK(file3.istream() == std::cin);
        BOOST_CHECK(file3.getFileName() == "");
    }

    {
        // stream, writing
        pdal::OStreamOwner file4(std::cout);
        BOOST_CHECK(file4.ostream() == std::cout);
        BOOST_CHECK(file4.getFileName() == "");
    }

    // cleanup
    Utils::deleteFile(wfile);
    BOOST_CHECK(!Utils::fileExists(wfile));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
