/******************************************************************************
 * Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <algorithm>

#include <nlohmann/json.hpp>

#include <pdal/pdal_test_main.hpp>

#include <io/CopcReader.hpp>
#include <io/LasReader.hpp>
#include <filters/CropFilter.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <filters/SortFilter.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/private/OGRSpec.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "gdal.h"
#include "cpl_vsi_virtual.h"

#include "Support.hpp"

namespace pdal
{

void testURLs(const std::string& url, const BOX2D& bounds)
{
    CopcReader reader;
    {
        Options options;
        options.add("bounds", bounds);
        options.add("filename", url);
        options.add("resolution", 1000);

        reader.setOptions(options);
    }

    pdal::QuickInfo qi(reader.preview());
    EXPECT_EQ(61201u, qi.m_pointCount);

    pdal::BOX3D bounds3d = qi.m_bounds;
    EXPECT_EQ(pdal::Bounds(bounds).to2d(), pdal::Bounds(bounds3d).to2d());
}


TEST(CopcRemoteReaderTest, plain)
{
    BOX2D bounds(635700, 848900, 637000, 853300);
    std::string url( "https://github.com/PDAL/data/raw/refs/heads/main/autzen/autzen-classified.copc.laz");
    testURLs(url, bounds);
}

void testVsi()
{
    BOX2D bounds(635700, 848900, 637000, 853300);
    std::string url( "https://github.com/PDAL/data/raw/refs/heads/main/autzen/autzen-classified.copc.laz");
    std::string vsi ("/vsicurl/" + url);
    testURLs(vsi, bounds);
}

void testVsiSimple()
{
    GDALAllRegister();

    std::string url("/vsicurl/https://github.com/PDAL/data/raw/refs/heads/main/"
                    "autzen/autzen-classified.copc.laz");
    uint32_t size = 54;

    std::vector<char> buf(size);

    std::cerr << "Enter!\n";
    VSILFILE *file = VSIFOpenL(url.c_str(), "rb");
    if (!file)
    {
        std::cerr << "Couldn't open file - perhaps no CURL support?\n";
    }
    else
    {
        std::cerr << "Opened!\n";
        VSIFSeekL(file, 375, SEEK_SET);
        std::cerr << "Seeked!\n";
        VSIFReadL(buf.data(), 1, size, file);
        std::cerr << "Read!\n";
        VSIFCloseL(file);
        std::cerr << "Closed!\n";
        int sum = 0;
        for (char c : buf)
            sum += (uint8_t)c;
        std::cerr << "Sum = " << sum << "!\n";
    }
}

TEST(CopcRemoteReaderTest, simplevsi)
{
    Support::wrap_timeout(testVsiSimple, 5000, "COPC VSI SIMPLE");
}

TEST(CopcRemoteReaderTest, vsi)
{
    Support::wrap_timeout(testVsi, 5000, "COPC VSI");

    /**
    BOX2D bounds(635700, 848900, 637000, 853300);
    std::string url( "https://github.com/PDAL/data/raw/refs/heads/main/autzen/autzen-classified.copc.laz");
    std::string vsi ("/vsicurl/" + url);
    testURLs(vsi, bounds);
    **/
}

} // namespace pdal
