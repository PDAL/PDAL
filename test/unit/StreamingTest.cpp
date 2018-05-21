/******************************************************************************
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

namespace
{

class R : public Reader, public Streamable
{

public:
    R() : m_entered(false)
    {}

    std::string getName() const
    { return "readers.r"; }

private:
    virtual bool processOne(PointRef& point)
    {
        if (!m_entered)
        {
            std::cout << tag();
            m_entered = true;
            return true;
        }
        return false;
    }

    virtual point_count_t read(PointViewPtr, point_count_t)
    {
        std::cout << tag();
        return 0;
    }
private:
    bool m_entered;
};

class F : public Filter, public Streamable
{

public:
    std::string getName() const
    { return "filters.f"; };

private:
    virtual bool processOne(PointRef& point)
    {
        std::cout << tag();
        return true;
    }

    virtual void filter(PointView& view)
    {
        std::cout << tag();
    }
};

} // unnamed pdal

TEST(Streaming, order)
{

StaticPluginInfo const f_info
{
    "filters.f",
    "F Filter",
    "",
    {}
};

CREATE_STATIC_STAGE(F, f_info)

StaticPluginInfo const r_info
{
    "readers.r",
    "R Reader",
    "",
    {}
};

CREATE_STATIC_STAGE(R, r_info)

std::string pipeline =
R"(
{
    "pipeline" : [
        {
            "type": "readers.r",
            "tag": "G"
        },
        {
            "type": "readers.r",
            "tag": "H"
        },
        {
            "type": "readers.r",
            "tag": "D"
        },
        {
            "type": "filters.f",
            "tag": "E",
            "inputs": [ "G", "H" ]
        },
        {
            "type": "readers.r",
            "tag": "F"
        },
        {
            "type": "filters.f",
            "tag": "B",
            "inputs": [ "D", "E", "F" ]
        },
        {
            "type": "readers.r",
            "tag": "C"
        },
        {
            "type": "filters.f",
            "tag": "A",
            "inputs": [ "B", "C" ]
        }
    ]
}
)";

/**
  Tree representation of the pipeline above:

       G   H
        \ /
     D   E   F
      \  |  /
       \ | /
         B  C
         | /
         A
**/

    std::ostringstream oss;

    // Order of traversal based on one point from each source.
    auto ctx = Utils::redirect(std::cout, oss);
    std::istringstream iss(pipeline);
    PipelineManager mgr;
    mgr.readPipeline(iss);
    FixedPointTable t(10000);
    mgr.executeStream(t);
    Utils::restore(std::cout, ctx);
    std::string output(oss.str());
    EXPECT_NE(output.find("DBAGEBAHEBAFBACA"), std::string::npos);

    // In non-stream mode we get a letter for each point view.
    oss.clear();
    oss.str("");
    std::cerr << oss.str() << "!\n";
    iss.seekg(0);
    ctx = Utils::redirect(std::cout, oss);
    PipelineManager mgr2;
    mgr2.readPipeline(iss);
    mgr2.execute();
    Utils::restore(std::cout, ctx);
    EXPECT_NE(output.find("DGHEEFBBBBCAAAAA"), std::string::npos);
}

