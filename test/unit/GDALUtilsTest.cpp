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

#include <pdal/GDALUtils.hpp>
#include <pdal/FileUtils.hpp>
#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(GDALUtilsTest)


BOOST_AUTO_TEST_CASE(GDALUtilsTest_1)
{
    const int FILESIZE = 10000;  // size of file to use
    const int NUMREPS = 10000; // number of points to check

    const std::string tempfile = Support::datapath("vsilfile_test.dat");

    pdal::FileUtils::deleteFile(tempfile);

    srand(17);

    // compute the "true" values, and store into test file
    boost::uint8_t truth[FILESIZE];
    {
        FILE* fp = fopen(tempfile.c_str(), "wb");
        for (int i=0; i<FILESIZE; i++)
        {
            truth[i] = rand() % 256;
            fwrite(&truth[i], 1, 1, fp);\
        }
        fclose(fp);
    }

    // open the test file
    VSILFILE* fp = VSIFOpenL/*fopen*/(tempfile.c_str(), "rb");
    boost::iostreams::stream<pdal::gdal::VSILFileBuffer> i(fp);

    for (int reps=0; reps<NUMREPS; reps++)
    {
        // seek to a random spot and read a point
        int pos = rand()%FILESIZE;
        i.seekg(pos, std::iostream::beg);

        boost::uint8_t c;
        i.read((char*)&c, 1);

        // did we get the correct value?
        BOOST_CHECK(c == truth[pos]);
    }

    i.close();
    VSIFCloseL/*fclose*/(fp);

    pdal::FileUtils::deleteFile(tempfile);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
