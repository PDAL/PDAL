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

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Utils.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/bpf/BpfReader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(BPFTest)

namespace
{

void test_file_type(const std::string& filename)
{
    using namespace pdal;

    PointContext context;

    Options ops;

    ops.add("filename", Support::datapath(filename));
    ops.add("count", 506);
    BpfReader reader(ops);

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

#ifdef PDAL_HAVE_ZLIB
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
#endif


BOOST_AUTO_TEST_SUITE_END()
