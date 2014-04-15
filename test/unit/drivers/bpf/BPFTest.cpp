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
#include <pdal/StageIterator.hpp>
#include <pdal/drivers/bpf/BpfReader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(BPFTest)

namespace
{
}

BOOST_AUTO_TEST_CASE(test_point_major)
{
    using namespace pdal;

    pdal::BpfReader reader(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));

    reader.initialize();
    const Schema& schema = reader.getSchema();
    
    PointBuffer data(schema, 3);
    StageSequentialIterator *it = reader.createSequentialIterator(data);
    boost::uint32_t numRead = it->read(data);
    BOOST_CHECK(numRead = 3);
    Dimension dimX = schema.getDimension("X");
    Dimension dimY = schema.getDimension("Y");
    Dimension dimZ = schema.getDimension("Z");
    try
    {
        Dimension dimQ = schema.getDimension("Q");
        BOOST_ERROR("Found unexpected dimension");
    }
    catch (std::runtime_error err)
    {
    }

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
        float x = data.getFieldAs<float>(dimX, i);
        float y = data.getFieldAs<float>(dimY, i);
        float z = data.getFieldAs<float>(dimZ, i);
        
        BOOST_CHECK_CLOSE(x, pts2[i].x, 0.001);
        BOOST_CHECK_CLOSE(y, pts2[i].y, 0.001);
        BOOST_CHECK_CLOSE(z, pts2[i].z, 0.001);
    }

    // Should be positioned at point 3.
    // Skip by 500, which should put us at 503.

    it->skip(500);
    numRead = it->read(data);
    BOOST_CHECK(numRead = 3);
    PtData pts[3] = { {494915.25, 4878096.5, 128.220001},
                      {494917.062, 4878124.5, 128.539993},
                      {494920.781, 4877914.5, 127.43} };

    for (int i = 0; i < 3; ++i)
    {
        float x = data.getFieldAs<float>(dimX, i);
        float y = data.getFieldAs<float>(dimY, i);
        float z = data.getFieldAs<float>(dimZ, i);
        
        BOOST_CHECK_CLOSE(x, pts[i].x, 0.001);
        BOOST_CHECK_CLOSE(y, pts[i].y, 0.001);
        BOOST_CHECK_CLOSE(z, pts[i].z, 0.001);
    }

    // We should now be at 506.  Skip too far.  Check that we've skipped
    // to the end.
    uint32_t numSkipped = it->skip(2000);
    BOOST_CHECK(numSkipped == 559);

    delete it;
}


BOOST_AUTO_TEST_SUITE_END()
