/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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
#include <boost/cstdint.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Reprojection.hpp>
#include <pdal/filters/Ferry.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include "Support.hpp"
#include "../StageTester.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(FerryFilterTest)


BOOST_AUTO_TEST_CASE(test_ferry_copy)
{
#ifdef PDAL_HAVE_GDAL
    using namespace pdal;
    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(Support::datapath("filters/ferry.xml"));

    Stage *stage = mgr.getStage();
    mgr.execute();
    PointContext ctx = mgr.context();

    PointBufferSet pbSet = mgr.buffers();

    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 1065u);

    Dimension::Id::Enum state_plane_x = ctx.findDim("StatePlaneX");
    Dimension::Id::Enum state_plane_y = ctx.findDim("StatePlaneY");

    double lon = buf->getFieldAs<double>(Dimension::Id::X, 0);
    double lat = buf->getFieldAs<double>(Dimension::Id::Y, 0);

    double x = buf->getFieldAs<double>(state_plane_x, 0);
    double y = buf->getFieldAs<double>(state_plane_y, 0);

    BOOST_CHECK_CLOSE(-117.2501328350574, lon, 0.0001);
    BOOST_CHECK_CLOSE(49.341077824192915, lat, 0.0001);
    BOOST_CHECK_CLOSE(637012.24, x, 0.0001);
    BOOST_CHECK_CLOSE(849028.31, y, 0.0001);
#endif
}

BOOST_AUTO_TEST_CASE(test_ferry_invalid)
{
    using namespace pdal;

    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    drivers::las::Reader reader;
    reader.setOptions(ops1);

    Options options;

    Option x("dimension", "X", "");
    Option toX("to","X", "");
    Options xO;
    xO.add(toX);
    x.setOptions(xO);
    options.add(x);

    filters::Ferry ferry;
    ferry.setInput(&reader);
    ferry.setOptions(options);

    PointContext ctx;

    BOOST_CHECK_THROW(ferry.prepare(ctx), pdal::pdal_error );
}

BOOST_AUTO_TEST_SUITE_END()
