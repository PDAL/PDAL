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

#include <boost/test/unit_test.hpp>

#include <pdal/Utils.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/PointBuffer.hpp>

#include "TestConfig.hpp"


std::string Support::datapath(const std::string& file)
{
    std::string s = TestConfig::g_data_path + file;
    return s;
}


bool Support::compare_files(const std::string& file1, const std::string& file2)
{
    if (!pdal::Utils::fileExists(file1) ||
        !pdal::Utils::fileExists(file2))
        return false;

    uintmax_t len1x = pdal::Utils::fileSize(file1);
    uintmax_t len2x = pdal::Utils::fileSize(file2);
    size_t len1 = (size_t)len1x; // BUG
    size_t len2 = (size_t)len2x;

    if (len1 != len2)
    {
        return false;
    }

    std::istream* str1 = pdal::Utils::openFile(file1);
    std::istream* str2 = pdal::Utils::openFile(file2);
    BOOST_CHECK(str1);
    BOOST_CHECK(str2);

    char* buf1 = new char[len1];
    char* buf2 = new char[len2];

    str1->read(buf1,len1);
    str2->read(buf2,len2);

    pdal::Utils::closeFile(str1);
    pdal::Utils::closeFile(str2);

    char* p = buf1;
    char* q = buf2;

    int numdiffs = 0;
    for (uintmax_t i=0; i<len1; i++)
    {
        if (*p != *q) 
        {
            ++numdiffs;
        }
        ++p;
        ++q;
    }

    delete[] buf1;
    delete[] buf2;

    return (numdiffs==0);
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
    BOOST_CHECK(r0 == rref);
    BOOST_CHECK(g0 == gref);
    BOOST_CHECK(b0 == bref);
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

