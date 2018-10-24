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

#include "gtest/gtest.h"

#include "Support.hpp"

#include <iostream>
#include <string>

#include <stdio.h>

#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>
#include "TestConfig.hpp"

using namespace pdal;
using namespace std;

string Support::datapath()
{
    return TestConfig::dataPath();
}

std::string Support::datapath(const std::string& file)
{
    return datapath() + file;
}

string Support::configuredpath()
{
    return TestConfig::configuredPath();
}

std::string Support::configuredpath(const std::string& file)
{
    return configuredpath() + file;
}

std::string Support::temppath()
{
    return TestConfig::dataPath() + "../temp/";
}

std::string Support::temppath(const std::string& file)
{
    return temppath() + file;
}

std::string Support::binpath()
{
    std::string binpath = TestConfig::binaryPath();

#ifdef PDAL_APP_BUNDLE
    return binpath + "/pdal.app/Contents/MacOS/";
#else
    return binpath;
#endif
}

std::string Support::binpath(const std::string& file)
{
    return binpath() + file;
}

std::string Support::exename(const std::string& name)
{
#ifdef _WIN32
    return name + ".exe";
#else
    return name;
#endif
}


// do a comparison by line of two (text) files, ignoring CRLF differences
uint32_t Support::diff_text_files(const std::string& file1,
    const std::string& file2, int32_t ignoreLine1)
{
    if (!pdal::Utils::fileExists(file1) || !pdal::Utils::fileExists(file2))
        return (std::numeric_limits<uint32_t>::max)();

    std::istream* str1 = pdal::Utils::openFile(file1, false);
    std::istream* str2 = pdal::Utils::openFile(file2, false);

    int32_t diffs = diff_text_files(*str1, *str2, ignoreLine1);

    pdal::Utils::closeFile(str1);
    pdal::Utils::closeFile(str2);
    return diffs;
}

uint32_t Support::diff_text_files(std::istream& str1, std::istream& str2,
    int32_t ignoreLine1)
{
    uint32_t numdiffs = 0;
    int32_t currLine = 1;

    while (!str1.eof() && !str2.eof())
    {
        std::string buf1;
        std::string buf2;
        std::getline(str1, buf1);
        std::getline(str2, buf2);

        if (currLine == ignoreLine1)
        {
            ++currLine;
            continue;
        }

        if (str1.eof() && str2.eof())
        {
            // hit end on both together
            break;
        }
        else if (str1.eof() && !str2.eof())
        {
            // str1 ended, but str2 still going
            while (!str2.eof())
            {
                std::getline(str2, buf2);
                ++numdiffs;
            }
            break;
        }
        else if (!str1.eof() && str2.eof())
        {
            // str2 ended, but str1 still going
            while (!str1.eof())
            {
                std::getline(str1, buf1);
                ++numdiffs;
            }
            break;
        }

        if (buf1 != buf2)
            ++numdiffs;

        ++currLine;
    }

    assert(str1.eof());
    assert(str2.eof());

    return numdiffs;
}


uint32_t Support::diff_files(const std::string& file1,
    const std::string& file2, uint32_t ignorable_start,
    uint32_t ignorable_length)
{
    uint32_t start[] = { ignorable_start };
    uint32_t len[] = { ignorable_length };
    return diff_files(file1, file2, start, len, 1);
}


// do a byte-wise comparison of two (binary) files
uint32_t Support::diff_files(const std::string& file1,
    const std::string& file2, uint32_t* ignorable_start,
    uint32_t* ignorable_length, uint32_t num_ignorables)
{
    if (!pdal::Utils::fileExists(file1) || !pdal::Utils::fileExists(file2))
        return (std::numeric_limits<uint32_t>::max)();

    std::istream* str1 = pdal::Utils::openFile(file1);
    std::istream* str2 = pdal::Utils::openFile(file2);

    uint32_t ret = diff_files(*str1, *str2, ignorable_start, ignorable_length,
        num_ignorables);

    pdal::Utils::closeFile(str1);
    pdal::Utils::closeFile(str2);
    return ret;
}


uint32_t Support::diff_files(std::istream& str1, std::istream& str2,
    uint32_t* ignorable_start, uint32_t* ignorable_length,
    uint32_t num_ignorables)
{
    uint32_t numdiffs = 0;
    char p, q;

    for (uint32_t i = 0; ; ++i)
    {
        str1.get(p);
        str2.get(q);
        if (!str1 || !str2)
        {
            if (!str1 != !str2)
                numdiffs++;
            break;
        }
        if (p == q)
            continue;

        if (num_ignorables == 0)
            ++numdiffs;
        else
        {
            // only count the difference if we are NOT in an ignorable
            // region
            bool is_ignorable = false;
            for (uint32_t region = 0; region < num_ignorables; region++)
            {
                uint32_t start = ignorable_start[region];
                uint32_t end = start + ignorable_length[region];
                if (i >= start && i < end)
                {
                    // we are in an ignorable region!
                    is_ignorable = true;
                    break;
                }
            }
            if (!is_ignorable)
                ++numdiffs;
        }
    }
    return numdiffs;
}


uint32_t Support::diff_files(const std::string& file1,
    const std::string& file2)
{
    return diff_files(file1, file2, NULL, NULL, 0);
}

uint32_t Support::diff_files(std::istream& str1, std::istream& str2)
{
    return diff_files(str1, str2, NULL, NULL, 0);
}

bool Support::compare_files(const std::string& file1, const std::string& file2)
{
    return diff_files(file1, file2) == 0;
}

bool Support::compare_text_files(const std::string& file1,
    const std::string& file2)
{
    return diff_text_files(file1, file2) == 0;
}

bool Support::compare_text_files(std::istream& str1, std::istream& str2)
{
    return diff_text_files(str1, str2) == 0;
}


void Support::checkXYZ(const std::string& file1, const std::string& file2)
{
    StageFactory f;

    const std::string driver1 = f.inferReaderDriver(file1);
    EXPECT_NE(driver1, "") << "Can't find driver";

    const std::string driver2 = f.inferReaderDriver(file2);
    EXPECT_NE(driver2, "") << "Can't find driver";

    Stage *reader1 = f.createStage(driver1);
    EXPECT_NE(reader1, (Stage *)NULL) << "Couldn't create stage";

    Stage *reader2 = f.createStage(driver2);
    EXPECT_NE(reader1, (Stage *)NULL) << "Couldn't create stage";

    Options o1;
    o1.add("filename", file1);
    reader1->setOptions(o1);
    PointTable t1;
    reader1->prepare(t1);
    PointViewSet s1 = reader1->execute(t1);
    EXPECT_EQ(s1.size(), 1u);
    PointViewPtr v1 = *s1.begin();

    Options o2;
    o2.add("filename", file2);
    reader2->setOptions(o2);
    PointTable t2;
    reader2->prepare(t2);
    PointViewSet s2 = reader2->execute(t2);
    EXPECT_EQ(s2.size(), 1u);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v1->size(), v2->size()) << "Files " << file1 << " and " <<
        file2 << " have different point counts.";
    for (PointId idx = 0; idx < v1->size(); ++idx)
    {
        using namespace Dimension;
        ASSERT_DOUBLE_EQ(v1->getFieldAs<double>(Id::X, idx),
            v2->getFieldAs<double>(Id::X, idx)) << "Index = " << idx;
        ASSERT_DOUBLE_EQ(v1->getFieldAs<double>(Id::Y, idx),
            v2->getFieldAs<double>(Id::Y, idx)) << "Index = " << idx;
        ASSERT_DOUBLE_EQ(v1->getFieldAs<double>(Id::Z, idx),
            v2->getFieldAs<double>(Id::Z, idx)) << "Index = " << idx;
    }
}


void Support::check_pN(const pdal::PointView& data, PointId index,
    double xref, double yref, double zref)
{
    float x0 = data.getFieldAs<float>(Dimension::Id::X, index);
    float y0 = data.getFieldAs<float>(Dimension::Id::Y, index);
    float z0 = data.getFieldAs<float>(Dimension::Id::Z, index);

    EXPECT_FLOAT_EQ(x0, static_cast<float>(xref));
    EXPECT_FLOAT_EQ(y0, static_cast<float>(yref));
    EXPECT_FLOAT_EQ(z0, static_cast<float>(zref));
}


void Support::check_pN(const PointView& data, PointId index,
    double xref, double yref, double zref, double tref,
    uint16_t rref, uint16_t gref, uint16_t bref)
{
    check_pN(data, index, xref, yref, zref);

    if (data.hasDim(Dimension::Id::GpsTime))
    {
        double t0 = data.getFieldAs<double>(Dimension::Id::GpsTime, index);
        EXPECT_DOUBLE_EQ(t0, tref);
    }

    if (data.hasDim(Dimension::Id::Red))
    {
        uint16_t r0 = data.getFieldAs<uint16_t>(Dimension::Id::Red, index);
        uint16_t g0 = data.getFieldAs<uint16_t>(Dimension::Id::Green, index);
        uint16_t b0 = data.getFieldAs<uint16_t>(Dimension::Id::Blue, index);
        EXPECT_EQ(r0, rref);
        EXPECT_EQ(g0, gref);
        EXPECT_EQ(b0, bref);
    }
}


void Support::check_p0_p1_p2(const pdal::PointView& data)
{
    Support::check_pN(data, 0, 637012.240000, 849028.310000, 431.660000);
    Support::check_pN(data, 1, 636896.330000, 849087.700000, 446.390000);
    Support::check_pN(data, 2, 636784.740000, 849106.660000, 426.710000);
}


void Support::check_p100_p101_p102(const pdal::PointView& data)
{
    Support::check_pN(data, 0, 636661.060000, 849854.130000, 424.900000);
    Support::check_pN(data, 1, 636568.180000, 850179.490000, 441.800000);
    Support::check_pN(data, 2, 636554.630000, 850040.030000, 499.110000);
}


void Support::check_p355_p356_p357(const pdal::PointView& data)
{
    Support::check_pN(data, 0, 636462.600000, 850566.110000, 432.610000);
    Support::check_pN(data, 1, 636356.140000, 850530.480000, 432.680000);
    Support::check_pN(data, 2, 636227.530000, 850592.060000, 428.670000);
}


void Support::check_p710_p711_p712(const pdal::PointView& data)
{
    Support::check_pN(data, 0, 638720.670000, 850926.640000, 417.320000);
    Support::check_pN(data, 1, 638672.380000, 851081.660000, 420.670000);
    Support::check_pN(data, 2, 638598.880000, 851445.370000, 422.150000);
}


void Support::compareBounds(const BOX3D& p, const BOX3D& q)
{
    EXPECT_DOUBLE_EQ(p.minx, q.minx);
    EXPECT_DOUBLE_EQ(p.miny, q.miny);
    EXPECT_DOUBLE_EQ(p.minz, q.minz);
    EXPECT_DOUBLE_EQ(p.maxx, q.maxx);
    EXPECT_DOUBLE_EQ(p.maxy, q.maxy);
    EXPECT_DOUBLE_EQ(p.maxz, q.maxz);
}

