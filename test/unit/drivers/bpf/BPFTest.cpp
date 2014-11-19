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

#include "UnitTest.hpp"

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Utils.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/bpf/BpfReader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(BPFTest)

using namespace pdal;

namespace
{

void test_file_type(const std::string& filename)
{
    PointContext context;

    Options ops;

    ops.add("filename", Support::datapath(filename));
    ops.add("count", 506);
    BpfReader reader;
    reader.setOptions(ops);

    reader.prepare(context);
    PointBufferSet pbSet = reader.execute(context);

    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 506);

    struct PtData
    {
        float x;
        float y;
        float z;
    };

    PtData pts2[3] = { {494057.312, 4877433.5, 130.630005},
                       {494133.812, 4877440, 130.440002},
                       {494021.094, 4877440, 130.460007} };

    for (int i = 0; i < 3; ++i)
    {
        float x = buf->getFieldAs<float>(Dimension::Id::X, i);
        float y = buf->getFieldAs<float>(Dimension::Id::Y, i);
        float z = buf->getFieldAs<float>(Dimension::Id::Z, i);

        BOOST_CHECK_CLOSE(x, pts2[i].x, 0.001);
        BOOST_CHECK_CLOSE(y, pts2[i].y, 0.001);
        BOOST_CHECK_CLOSE(z, pts2[i].z, 0.001);
    }

    PtData pts[3] = { {494915.25, 4878096.5, 128.220001},
                      {494917.062, 4878124.5, 128.539993},
                      {494920.781, 4877914.5, 127.43} };

    for (int i = 503; i < 3; ++i)
    {
        float x = buf->getFieldAs<float>(Dimension::Id::X, i);
        float y = buf->getFieldAs<float>(Dimension::Id::Y, i);
        float z = buf->getFieldAs<float>(Dimension::Id::Z, i);

        BOOST_CHECK_CLOSE(x, pts[i].x, 0.001);
        BOOST_CHECK_CLOSE(y, pts[i].y, 0.001);
        BOOST_CHECK_CLOSE(z, pts[i].z, 0.001);
    }
}

} //namespace

BOOST_AUTO_TEST_CASE(test_point_major)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3-interleaved.bpf");
}

BOOST_AUTO_TEST_CASE(test_dim_major)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3.bpf");
}

BOOST_AUTO_TEST_CASE(test_byte_major)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3-segregated.bpf");
}

BOOST_AUTO_TEST_CASE(test_point_major_zlib)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3-deflate-interleaved.bpf");
}

BOOST_AUTO_TEST_CASE(test_dim_major_zlib)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3-deflate.bpf");
}

BOOST_AUTO_TEST_CASE(test_byte_major_zlib)
{
    test_file_type("bpf/autzen-utm-chipped-25-v3-deflate-segregated.bpf");
}

BOOST_AUTO_TEST_CASE(inspect)
{
    Options ops;
    ops.add("filename", Support::datapath("bpf/autzen-dd.bpf"));

    BpfReader reader;
    reader.setOptions(ops);

    QuickInfo qi = reader.preview();

    std::string testWkt = "PROJCS[\"WGS 84 / SCAR IMW ST05-08\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"standard_parallel_1\",-76.66666666666667],PARAMETER[\"standard_parallel_2\",-79.33333333333333],PARAMETER[\"latitude_of_origin\",-90],PARAMETER[\"central_meridian\",-144],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH],AUTHORITY[\"EPSG\",\"3261\"]]";
    BOOST_CHECK_EQUAL(qi.m_srs.getWKT(), testWkt);

    BOOST_CHECK_EQUAL(qi.m_pointCount, 1065);

    BOX3D bounds(
        -13676090.610841721296, 4894836.9556098170578, 123.93000030517578125,
        -13674705.011110275984, 4896224.6888861842453, 178.7299957275390625);
    BOOST_CHECK_EQUAL(qi.m_bounds, bounds);

    const char *dims[] = 
    {
        "Blue",
        "Classification",
        "GPSTime",
        "Green",
        "Intensity",
        "Number of Returns",
        "Red",
        "Return Information",
        "Return Number",
        "X",
        "Y",
        "Z"
    };

    std::sort(qi.m_dimNames.begin(), qi.m_dimNames.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(qi.m_dimNames.begin(), qi.m_dimNames.end(),
      std::begin(dims), std::end(dims));
}

BOOST_AUTO_TEST_SUITE_END()
