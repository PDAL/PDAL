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
#include <iostream>

#include <pdal/StreamFactory.hpp>
#include <pdal/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StreamFactoryTest)

BOOST_AUTO_TEST_CASE(test1)
{
    const std::string wfilename = "temp.txt";

    // filename, writing
    {
        FileUtils::deleteFile(wfilename);
        BOOST_CHECK(!FileUtils::fileExists(wfilename));

        {
            pdal::OutputStreamManager wfile(wfilename);
            wfile.open();

            BOOST_CHECK(FileUtils::fileExists(wfilename));
            wfile.close();
        }
     
        // cleanup
        FileUtils::deleteFile(wfilename);
        BOOST_CHECK(!FileUtils::fileExists(wfilename));
    }

    // stream, writing
    {
        std::ostream* ostreamname = FileUtils::createFile(wfilename);
        
        {
            pdal::OutputStreamManager ostream(ostreamname);
            ostream.open();
            BOOST_CHECK(&(ostream.ostream()) == ostreamname);
            ostream.close();
        }

        FileUtils::closeFile(ostreamname);
        FileUtils::deleteFile(wfilename);
    }

    return;
}


static void check_contents_sub(std::istream& s)
{
    int c;
    
    c = s.get();
    BOOST_CHECK(c == 'd');
    
    c = s.get();
    BOOST_CHECK(c == 'i');
    
    c = s.get();
    BOOST_CHECK(c == 's');
    
    c = s.get();
    BOOST_CHECK(c == 't');
    
    c = s.get();
    BOOST_CHECK(c == -1);
    
    BOOST_CHECK(s.eof());
}


static void check_contents(std::istream& s)
{
    int c;
    
    c = s.get();
    BOOST_CHECK(c == 'R');
    
    c = s.get();
    BOOST_CHECK(c == 'e');
    
    s.seekg(-4, std::ios_base::end);

    std::streamoff e = s.tellg();
    BOOST_CHECK(e == 78);

    c = s.get();
    BOOST_CHECK(c == 'n');
    
    BOOST_CHECK(!s.eof());

    BOOST_CHECK(s.get() == '.');
    BOOST_CHECK(s.get() == '.');
    BOOST_CHECK(s.get() == '.');
    BOOST_CHECK(s.get() == -1);

    BOOST_CHECK(s.eof());

    return;
}


BOOST_AUTO_TEST_CASE(test2)
{
    {
        const std::string nam = Support::datapath("text.txt");
        FilenameStreamFactory f(nam);

        std::istream& s1 = f.allocate();
        std::istream& s2 = f.allocate();
        std::istream& s3 = f.allocate();

        check_contents(s1);
        check_contents(s2);
        check_contents(s3);

        f.deallocate(s3);
        f.deallocate(s1);
        // f.deallocate(s2);   // let the dtor do it for us
    }

    {
        const std::string nam = Support::datapath("text.txt");
        std::istream* s = FileUtils::openFile(nam);
        PassthruStreamFactory f(*s);

        std::istream& s1 = f.allocate();

        BOOST_REQUIRE_THROW(f.allocate(), pdal_error);

        check_contents(s1);

        f.deallocate(s1);

        FileUtils::closeFile(s);
    }

    {
        const std::string nam = Support::datapath("text.txt");
        FilenameSubsetStreamFactory f(nam,2,4);

        std::istream& s1 = f.allocate();
        std::istream& s2 = f.allocate();
        std::istream& s3 = f.allocate();

        check_contents_sub(s1);
        check_contents_sub(s2);
        check_contents_sub(s3);

        f.deallocate(s1);
        f.deallocate(s3);
        // f.deallocate(s2);   // let the dtor do it for us
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
