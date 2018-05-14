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

#include <pdal/pdal_test_main.hpp>

#include <pdal/Filter.hpp>
#include <pdal/PointTable.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/MergeFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>
#include "Support.hpp"

using namespace pdal;

// This test depends on stages being executed in the order that they were
// added to each parent.  If you change order, things will break.
TEST(Streaming, filter)
{
    Options ro1;
    ro1.add("bounds", BOX3D(0, 0, 0, 99, 99, 99));
    ro1.add("mode", "ramp");
    ro1.add("count", 100);
    FauxReader r1;
    r1.setOptions(ro1);

    Options ro2;
    ro2.add("bounds", BOX3D(100, 100, 100, 199, 199, 199));
    ro2.add("mode", "ramp");
    ro2.add("count", 100);
    FauxReader r2;
    r2.setOptions(ro2);

    Options ro3;
    ro3.add("bounds", BOX3D(200, 200, 200, 299, 299, 299));
    ro3.add("mode", "ramp");
    ro3.add("count", 100);
    FauxReader r3;
    r3.setOptions(ro3);

    Options ro4;
    ro4.add("bounds", BOX3D(300, 300, 300, 399, 399, 399));
    ro4.add("mode", "ramp");
    ro4.add("count", 100);
    FauxReader r4;
    r4.setOptions(ro4);

    MergeFilter m1;
    m1.setInput(r1);
    m1.setInput(r2);

    MergeFilter m2;
    m2.setInput(r3);
    m2.setInput(r4);

    MergeFilter m3;
    m3.setInput(m1);
    m3.setInput(m2);

    StreamCallbackFilter f;
    int cnt = 0;
    auto cb = [&cnt](PointRef& point)
    {
        static int x(0), y(0), z(0);
        EXPECT_EQ(point.getFieldAs<int>(Dimension::Id::X), x++);
        EXPECT_EQ(point.getFieldAs<int>(Dimension::Id::Y), y++);
        EXPECT_EQ(point.getFieldAs<int>(Dimension::Id::Z), z++);
        cnt++;
        return true;
    };
    f.setCallback(cb);
    f.setInput(m3);

    FixedPointTable t(20);
    f.prepare(t);
    f.execute(t);
    EXPECT_EQ(cnt, 400);
}

// Test that SRS changes aren't repeated when new input is processed.
TEST(Streaming, issue_2009)
{
    StageFactory f;

    Stage& r1 = *(f.createStage("readers.las"));
    Options r1Opts;
    r1Opts.add("filename", Support::datapath("las/test_epsg_4326.las"));
    r1.setOptions(r1Opts);

    Stage& r2 = *(f.createStage("readers.las"));
    Options r2Opts;
    r2Opts.add("filename", Support::datapath("las/test_epsg_4326.las"));
    r2.setOptions(r2Opts);

    class TestFilter : public Filter, public Streamable
    {
    public:
        TestFilter() : m_srsCnt(0)
        {}

        std::string getName() const { return "filters.test"; }

        int m_srsCnt;

    private:
        virtual void spatialReferenceChanged(const SpatialReference&)
        {
            m_srsCnt++;
        }

        virtual bool processOne(PointRef&)
        {
            return true;
        }
    };

    TestFilter t;
    t.setInput(r1);
    t.setInput(r2);

    FixedPointTable table(100);
    t.prepare(table);
    t.execute(table);

    EXPECT_EQ(t.m_srsCnt, 1);
}
