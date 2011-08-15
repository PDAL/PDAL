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

#include "Support.hpp"

#include <iostream>
#include <string>

#include <boost/test/unit_test.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Stage.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/exceptions.hpp>
#include "TestConfig.hpp"

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
//#pragma GCC diagnostic ignored "-Wsign-compare"
#endif


std::string Support::datapath()
{
    const std::string s = TestConfig::g_data_path;
    return s;
}

bool Support::compare_stage_data(pdal::Stage const& a, pdal::Stage const& b)
{
    pdal::Schema const& a_schema = a.getSchema();
    pdal::Schema const& b_schema = b.getSchema();
    if (a_schema != b_schema) return false;
    
    boost::uint64_t a_num_points = a.getNumPoints();
    boost::uint64_t b_num_points = b.getNumPoints();
    if (a_num_points != b_num_points) return false;
    
    pdal::StageSequentialIterator* a_itr = a.createSequentialIterator();
    pdal::StageSequentialIterator* b_itr = b.createSequentialIterator();
    
    if (!a_itr) throw pdal::pdal_error("unable to create sequential iterator for compare_stage_data for stage a");
    if (!b_itr) throw pdal::pdal_error("unable to create sequential iterator for compare_stage_data for stage b");

    
    // if we don't have any sizes here, we'll just use a small default
    if (a_num_points == 0) a_num_points = 1024;
    if (b_num_points == 0) b_num_points = 1024;
    
    pdal::PointBuffer a_data(a_schema, a_num_points);
    pdal::PointBuffer b_data(b_schema, b_num_points);

    {
        boost::uint32_t a_numRead = a_itr->read(a_data);
        boost::uint32_t b_numRead = b_itr->read(b_data);
        if (a_numRead != b_numRead) return false;
        
        boost::uint8_t const* a_bytes = a_data.getData(0);
        boost::uint8_t const* b_bytes = b_data.getData(0);
        
        boost::uint64_t a_length = a_data.getBufferByteLength();
        // boost::uint64_t b_length = b_data.getBufferByteLength();
    
        for (boost::uintmax_t i=0; i<a_length; i++)
        {
            if (*a_bytes != *b_bytes) 
            {
                return false;
            }
            ++a_bytes;
            ++b_bytes;
        }
        
    }
    
    return true;
    
}

std::string Support::datapath(const std::string& file)
{
    const std::string s = datapath() + file;
    return s;
}


std::string Support::temppath()
{
    const std::string s = TestConfig::g_data_path + "../temp/";
    return s;
}


std::string Support::temppath(const std::string& file)
{
    const std::string s = temppath() + file;
    return s;
}


std::string Support::binpath()
{
    const std::string argv0 = boost::unit_test::framework::master_test_suite().argv[0];
    const std::string s = pdal::FileUtils::getDirectory(argv0);
    return s;
}


std::string Support::binpath(const std::string& file)
{
    const std::string s = binpath() + file;
    return s;
}


std::string Support::exename(const std::string& name)
{
#ifdef PDAL_COMPILER_MSVC
    return name + ".exe";
#else
    return name;
#endif
}


// do a comparison by line of two (text) files, ignoring CRLF differences
boost::uint32_t Support::diff_text_files(const std::string& file1, const std::string& file2)
{
    if (!pdal::FileUtils::fileExists(file1) ||
        !pdal::FileUtils::fileExists(file2))
        return std::numeric_limits<boost::uint32_t>::max();

    std::istream* str1 = pdal::FileUtils::openFile(file1, false);
    std::istream* str2 = pdal::FileUtils::openFile(file2, false);
    BOOST_CHECK(str1);
    BOOST_CHECK(str2);

    boost::uint32_t numdiffs = 0;
    while (!str1->eof() && !str2->eof())
    {
        std::string buf1;
        std::string buf2;
        std::getline(*str1, buf1);
        std::getline(*str2, buf2);

        if (str1->eof() && str2->eof())
        {
            // hit end on both together
            break;
        }
        else if (str1->eof() && !str2->eof())
        {
            // str1 ended, but str2 still going
            while (!str2->eof())
            {
                std::getline(*str2, buf2);
                ++numdiffs;
            }
            break;
        }
        else if (!str1->eof() && str2->eof())
        {
            // str2 ended, but str1 still going
            while (!str1->eof())
            {
                std::getline(*str1, buf1);
                ++numdiffs;
            }
            break;
        }

        if (buf1 != buf2)
        {
            ++numdiffs;
        }
    }

    assert(str1->eof());
    assert(str2->eof());

    pdal::FileUtils::closeFile(str1);
    pdal::FileUtils::closeFile(str2);

    return numdiffs;
}


boost::uint32_t Support::diff_files(const std::string& file1, const std::string& file2,
                                    boost::uint32_t ignorable_start, boost::uint32_t ignorable_length)
{
    boost::uint32_t start[] = { ignorable_start };
    boost::uint32_t len[] = { ignorable_length };
    return diff_files(file1, file2, start, len, 1);
}


// do a byte-wise comparison of two (binary) files
boost::uint32_t Support::diff_files(const std::string& file1, const std::string& file2,
                                    boost::uint32_t* ignorable_start, boost::uint32_t* ignorable_length, boost::uint32_t num_ignorables)
{
    if (!pdal::FileUtils::fileExists(file1) ||
        !pdal::FileUtils::fileExists(file2))
        return std::numeric_limits<boost::uint32_t>::max();

    boost::uintmax_t len1x = pdal::FileUtils::fileSize(file1);
    boost::uintmax_t len2x = pdal::FileUtils::fileSize(file2);
    const size_t len1 = (size_t)len1x; // BUG
    const size_t len2 = (size_t)len2x;

    std::istream* str1 = pdal::FileUtils::openFile(file1);
    std::istream* str2 = pdal::FileUtils::openFile(file2);
    BOOST_CHECK(str1);
    BOOST_CHECK(str2);

    char* buf1 = new char[len1];
    char* buf2 = new char[len2];

    str1->read(buf1,len1);
    str2->read(buf2,len2);

    pdal::FileUtils::closeFile(str1);
    pdal::FileUtils::closeFile(str2);

    char* p = buf1;
    char* q = buf2;

    boost::uint32_t numdiffs = 0;
    const size_t minlen = (len1 < len2) ? len1 : len2;
    const size_t maxlen = (len1 > len2) ? len1 : len2;
    for (size_t i=0; i<minlen; i++)
    {
        if (*p != *q) 
        {
            if (num_ignorables == 0)
            {
                ++numdiffs;
            }
            else
            {
                // only count the difference if we are NOT in an ignorable region
                bool is_ignorable = false;
                for (boost::uint32_t region=0; region<num_ignorables; region++)
                {
                    const boost::uint32_t start = ignorable_start[region];
                    const boost::uint32_t end = start + ignorable_length[region];
                    if (i >= start && i < end)
                    {
                        // we are in an ignorable region!
                        is_ignorable = true;
                        break;
                    }
                }
                if (is_ignorable == false)
                {
                    ++numdiffs;
                }
            }
        }

        ++p;
        ++q;
    }

    if (minlen != maxlen)
    {
        numdiffs += (maxlen - minlen);
    }

    delete[] buf1;
    delete[] buf2;

    return numdiffs;
}


boost::uint32_t Support::diff_files(const std::string& file1, const std::string& file2)
{
    return diff_files(file1, file2, NULL, NULL, 0);
}


bool Support::compare_files(const std::string& file1, const std::string& file2)
{
    const boost::uint32_t numdiffs = diff_files(file1, file2);
    return (numdiffs == 0);
}


bool Support::compare_text_files(const std::string& file1, const std::string& file2)
{
    boost::uint32_t numdiffs = diff_text_files(file1, file2);
    return (numdiffs == 0);
}



#define Compare(x,y)    BOOST_CHECK(pdal::Utils::compare_approx((x),(y),0.001));

void Support::check_pN(const pdal::PointBuffer& data, const pdal::Schema& schema, 
                       std::size_t index, 
                       double xref, double yref, double zref)
{
    int offsetX = schema.getDimensionIndex(pdal::Dimension::Field_X, pdal::Dimension::Int32);
    int offsetY = schema.getDimensionIndex(pdal::Dimension::Field_Y, pdal::Dimension::Int32);
    int offsetZ = schema.getDimensionIndex(pdal::Dimension::Field_Z, pdal::Dimension::Int32);

    boost::int32_t x0raw = data.getField<boost::int32_t>(index, offsetX);
    boost::int32_t y0raw = data.getField<boost::int32_t>(index, offsetY);
    boost::int32_t z0raw = data.getField<boost::int32_t>(index, offsetZ);
    double x0 = schema.getDimension(offsetX).applyScaling<boost::int32_t>(x0raw);
    double y0 = schema.getDimension(offsetY).applyScaling<boost::int32_t>(y0raw);
    double z0 = schema.getDimension(offsetZ).applyScaling<boost::int32_t>(z0raw);
    
    Compare(x0, xref);
    Compare(y0, yref);
    Compare(z0, zref);
}


void Support::check_pN(const pdal::PointBuffer& data, const ::pdal::Schema& schema, 
                       std::size_t index, 
                       double xref, double yref, double zref,
                       double tref,
                       boost::uint16_t rref, boost::uint16_t gref, boost::uint16_t bref)
{
    check_pN(data, schema, index, xref, yref, zref);

    int offsetT = schema.getDimensionIndex(pdal::Dimension::Field_Time, pdal::Dimension::Double);
    double t0 = data.getField<double>(index, offsetT);
    Compare(t0, tref);

    int offsetR = schema.getDimensionIndex(pdal::Dimension::Field_Red, pdal::Dimension::Uint16);
    int offsetG = schema.getDimensionIndex(pdal::Dimension::Field_Green, pdal::Dimension::Uint16);
    int offsetB = schema.getDimensionIndex(pdal::Dimension::Field_Blue, pdal::Dimension::Uint16);
    boost::uint16_t r0 = data.getField<boost::uint16_t>(index, offsetR);
    boost::uint16_t g0 = data.getField<boost::uint16_t>(index, offsetG);
    boost::uint16_t b0 = data.getField<boost::uint16_t>(index, offsetB);
    BOOST_CHECK_EQUAL(r0, rref);
    BOOST_CHECK_EQUAL(g0, gref);
    BOOST_CHECK_EQUAL(b0, bref);
}


void Support::check_p0_p1_p2(const pdal::PointBuffer& data, const pdal::Schema& schema)
{
    Support::check_pN(data, schema, 0, 637012.240000, 849028.310000, 431.660000);
    Support::check_pN(data, schema, 1, 636896.330000, 849087.700000, 446.390000);
    Support::check_pN(data, schema, 2, 636784.740000, 849106.660000, 426.710000);
}


void Support::check_p100_p101_p102(const pdal::PointBuffer& data, const pdal::Schema& schema)
{
    Support::check_pN(data, schema, 0, 636661.060000, 849854.130000, 424.900000);
    Support::check_pN(data, schema, 1, 636568.180000, 850179.490000, 441.800000);
    Support::check_pN(data, schema, 2, 636554.630000, 850040.030000, 499.110000);
}


void Support::check_p355_p356_p357(const pdal::PointBuffer& data, const pdal::Schema& schema)
{
    Support::check_pN(data, schema, 0, 636462.600000, 850566.110000, 432.610000);
    Support::check_pN(data, schema, 1, 636356.140000, 850530.480000, 432.680000);
    Support::check_pN(data, schema, 2, 636227.530000, 850592.060000, 428.670000);
}


void Support::check_p710_p711_p712(const pdal::PointBuffer& data, const pdal::Schema& schema)
{
    Support::check_pN(data, schema, 0, 638720.670000, 850926.640000, 417.320000);
    Support::check_pN(data, schema, 1, 638672.380000, 851081.660000, 420.670000);
    Support::check_pN(data, schema, 2, 638598.880000, 851445.370000, 422.150000);
}


void Support::compareBounds(const pdal::Bounds<double>& p, const pdal::Bounds<double>& q)
{
    BOOST_CHECK_CLOSE(p.getMinimum(0), q.getMinimum(0), 1);
    BOOST_CHECK_CLOSE(p.getMinimum(1), q.getMinimum(1), 1);
    BOOST_CHECK_CLOSE(p.getMinimum(2), q.getMinimum(2), 1);
    BOOST_CHECK_CLOSE(p.getMaximum(0), q.getMaximum(0), 1);
    BOOST_CHECK_CLOSE(p.getMaximum(1), q.getMaximum(1), 1);
    BOOST_CHECK_CLOSE(p.getMaximum(2), q.getMaximum(2), 1);
}


#ifdef PDAL_COMPILER_MSVC
// http://www.codepedia.com/1/CppStringReplace
static std::string replaceAll(std::string result, 
                              const std::string& replaceWhat, 
                              const std::string& replaceWithWhat)
{
    while(1)
    {
        const int pos = result.find(replaceWhat);
        if (pos==-1) break;
        result.replace(pos,replaceWhat.size(),replaceWithWhat);
    }
    return result;
}
#endif


static FILE* portable_popen(const std::string& command, const std::string& mode)
{
    FILE* fp = 0;
    
#ifdef PDAL_COMPILER_MSVC
    const std::string dos_command = replaceAll(command, "/", "\\");
    fp = _popen(dos_command.c_str(), "r");
#else
    fp = popen(command.c_str(), "r");
#endif

    return fp;
}


static int portable_pclose(FILE* fp)
{
    int status = 0;

#ifdef PDAL_COMPILER_MSVC
    status = _pclose(fp);
#else
    status = pclose(fp);
    if (status == -1)
    {
        throw std::runtime_error("error executing command");
    }
    if (WIFEXITED(status) != 0)
    {
        status = WEXITSTATUS(status);
    }
    else
    {
        status = 0;
    }
    #endif

    return status;
}


int Support::run_command(const std::string& cmd, std::string& output)
{
    const int maxbuf = 4096;
    char buf[maxbuf];
        
    output = "";
    
    FILE* fp = portable_popen(cmd.c_str(), "r");
    
    while (!feof(fp))
    {
        if (fgets(buf, maxbuf, fp) == NULL)
        {
            if (feof(fp)) break;
            if (ferror(fp)) break;                
        }

        output += buf;
    }

    int stat = portable_pclose(fp);

    return stat;
}
