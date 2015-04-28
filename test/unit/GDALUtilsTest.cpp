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

//#include "UnitTest.hpp"
#include <pdal/pdal_test_main.hpp>

#include <pdal/GDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/iostreams/restrict.hpp>
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

using namespace pdal;

TEST(GDALUtilsTest, test_wrapped_vsifile_read)
{
    const int FILESIZE = 10000;  // size of file to use
    const int NUMREPS = 10000; // number of points to check

    const std::string tempfile = Support::datapath("vsilfile_test.dat");

    FileUtils::deleteFile(tempfile);

    srand(17);

    // compute the "true" values, and store into test file
    uint8_t truth[FILESIZE];
    {
        FILE* fp = fopen(tempfile.c_str(), "wb");
        for (int i=0; i<FILESIZE; i++)
        {
            truth[i] = rand() % 256;
            fwrite(&truth[i], 1, 1, fp);
        }
        fclose(fp);
    }

    // open the test file
    VSILFILE* fp = VSIFOpenL/*fopen*/(tempfile.c_str(), "rb");
    boost::iostreams::stream<gdal::VSILFileBuffer> i(fp);

    for (int reps=0; reps<NUMREPS; reps++)
    {
        // seek to a random spot and read a point
        int pos = rand()%FILESIZE;
        i.seekg(pos, std::iostream::beg);

        uint8_t c;
        i.read((char*)&c, 1);

        // did we get the correct value?
        EXPECT_TRUE(c == truth[pos]);
    }

    i.close();
    VSIFCloseL/*fclose*/(fp);

    FileUtils::deleteFile(tempfile);
}


TEST(GDALUtilsTest, GDALUtilsTest_test_vsifile_write)
{
    const int FILESIZE = 10000;  // size of file to use

    const std::string tempfile_a = Support::datapath("vsilfile_test_a.dat");
    const std::string tempfile_b = Support::datapath("vsilfile_test_b.dat");

    FileUtils::deleteFile(tempfile_a);
    FileUtils::deleteFile(tempfile_b);

    // compute the "true" values, and store into test file
    uint8_t truth[FILESIZE];
    {
        FILE* fp_a = fopen(tempfile_a.c_str(), "wb");
        VSILFILE* fp_b = VSIFOpenL(tempfile_b.c_str(), "wb");
        for (int i=0; i<FILESIZE; i++)
        {
            truth[i] = rand() % 256;
            fwrite(&truth[i], 1, 1, fp_a);
            VSIFWriteL(&truth[i], 1, 1, fp_b);
        }

        fseek(fp_a, 40, SEEK_SET);
        VSIFSeekL(fp_b, 40, SEEK_SET);

        fwrite(truth, 1, 34, fp_a);
        VSIFWriteL(truth, 1, 34, fp_b);

        fseek(fp_a, 342, SEEK_END);
        VSIFSeekL(fp_b, 342, SEEK_END);

        fwrite(truth, 1, 12, fp_a);
        VSIFWriteL(truth, 1, 12, fp_b);

        long pos_a = (long)ftell(fp_a);
        long pos_b = (long)VSIFTellL(fp_b);
        EXPECT_EQ(pos_a, pos_b);

        fclose(fp_a);
        VSIFCloseL(fp_b);
    }

    const bool ok = Support::compare_files(tempfile_a, tempfile_b);
    EXPECT_TRUE(ok);

    FileUtils::deleteFile(tempfile_a);
    FileUtils::deleteFile(tempfile_b);
}


TEST(GDALUtilsTest, test_wrapped_vsifile_subsequence)
{
    const std::string tempfile = Support::datapath("vsilfile_test.dat");

    FileUtils::deleteFile(tempfile);

    // create a test file
    {
        FILE* fp = fopen(tempfile.c_str(), "wb");
        for (int i=0; i<100; i++)
        {
            char c = (char)i;
            fwrite(&c, 1, 1, fp);
        }
        fclose(fp);
    }

    namespace io = boost::iostreams;

    // open the test file with VSID
    VSILFILE* vsi_file = VSIFOpenL/*fopen*/(tempfile.c_str(), "rb");

    // wrap it with an iostream
    io::stream<gdal::VSILFileBuffer> full_stream(vsi_file);

    // and make a subset view
    io::restriction<io::stream<gdal::VSILFileBuffer> > restricted_dev(full_stream, 10, 20);
    io::stream<io::restriction<io::stream<gdal::VSILFileBuffer> > > restricted_str(restricted_dev);

    std::istream& str(restricted_str);

    // seek to a random spot and read a point
    str.seekg(5, std::iostream::beg);

    uint8_t c;

    str.read((char*)&c, 1);
    EXPECT_TRUE(c == 15);

    str.read((char*)&c, 1);
    EXPECT_TRUE(c == 16);

    str.seekg(4, std::iostream::cur);
    str.read((char*)&c, 1);
    EXPECT_TRUE(c == 21);

    str.seekg(-5, std::iostream::end);
    str.read((char*)&c, 1);
    EXPECT_TRUE(c == 25);

    restricted_str.close();
    restricted_dev.close();
    full_stream.close();

    VSIFCloseL/*fclose*/(vsi_file);

    FileUtils::deleteFile(tempfile);
}
