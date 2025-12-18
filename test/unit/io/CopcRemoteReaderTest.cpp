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

#include "Support.hpp"

namespace pdal
{

namespace
{
    const std::string copcPath(Support::datapath("copc/lone-star.copc.laz"));
    const std::string copcAutzenPath(Support::datapath("copc/1.2-with-color.copc.laz"));
    const BOX3D pointBounds(515368.60225, 4918340.364, 2322.89625,
        515401.043, 4918381.12375, 2338.5755);
    const point_count_t numPoints(518862);
}


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
    EXPECT_EQ(61201, qi.m_pointCount);

    pdal::BOX3D bounds3d = qi.m_bounds;
    EXPECT_EQ(pdal::Bounds(bounds).to2d(), pdal::Bounds(bounds3d).to2d());

}


TEST(CopcRemoteReaderTest, vsi)
{

    /*
          "maxx": 639003.73,
          "maxy": 853534.37,
          "maxz": 615.26,
          "minx": 635579.2,
          "miny": 848887.49,
          "minz": 406.56
    */

    BOX2D bounds(635700,848900, 637000, 853300);
    std::string url( "https://github.com/PDAL/data/raw/refs/heads/main/autzen/autzen-classified.copc.laz");
    std::string vsi ("/vsicurl/"+url);

    testURLs(url, bounds);
    testURLs(vsi, bounds);


}

} // namespace pdal
