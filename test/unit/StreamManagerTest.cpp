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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include <pdal/StreamManager.hpp>
#include <pdal/Utils.hpp>
#include <pdal/exceptions.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StreamManagerTest)

BOOST_AUTO_TEST_CASE(StreamManagerTest_test1)
{
    const std::string rfilename = Support::datapath("1.2-with-color.las");
    const std::string wfilename = "temp.txt";

    // filename, reading
    {
        BOOST_CHECK(Utils::fileExists(rfilename));
        {
            pdal::IStreamManager rfile(rfilename);
            rfile.open();
            BOOST_CHECK(rfile.getFileName() == rfilename);
            rfile.close();
        }
    }

    // filename, reading -- should throw
    {
        const std::string silly = "sillyfilenamethatdoesnotexist.xml";
        BOOST_CHECK(!Utils::fileExists(silly));

        {
            bool ok = false;
            try
            {
                pdal::IStreamManager file(silly);
                file.open();
                ok = false;
            }
            catch (pdal::pdal_error ex)
            {
                ok = true;
            }
            BOOST_CHECK(ok);
        }
    }
    
    // filename, writing
    {
        Utils::deleteFile(wfilename);
        BOOST_CHECK(!Utils::fileExists(wfilename));

        {
            pdal::OStreamManager wfile(wfilename);
            wfile.open();
            BOOST_CHECK(wfile.getFileName() == wfilename);

            BOOST_CHECK(Utils::fileExists(wfilename));
            wfile.close();
        }
     
        // cleanup
        Utils::deleteFile(wfilename);
        BOOST_CHECK(!Utils::fileExists(wfilename));
    }

    // stream, reading
    {
        std::istream* istreamname = Utils::openFile(rfilename);
        
        {
            pdal::IStreamManager istream(istreamname);
            istream.open();
            BOOST_CHECK(istream.istream() == *istreamname);
            BOOST_CHECK(istream.getFileName() == "");
            istream.close();
        }

        Utils::closeFile(istreamname);
    }

    // stream, writing
    {
        std::ostream* ostreamname = Utils::createFile(wfilename);
        
        {
            pdal::OStreamManager ostream(ostreamname);
            ostream.open();
            BOOST_CHECK(ostream.ostream() == *ostreamname);
            BOOST_CHECK(ostream.getFileName() == "");
            ostream.close();
        }

        Utils::closeFile(ostreamname);
        Utils::deleteFile(wfilename);
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
