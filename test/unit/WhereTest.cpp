/******************************************************************************
* Copyright (c) 2020, Hobu Inc. (info@hobu.co)
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
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

namespace
{
    size_t g_count;
}

TEST(WhereTest, t1)
{
    StageFactory factory;

    Stage *r = factory.createStage("readers.faux");
    Options ro;
    ro.add("count", 100);
    ro.add("bounds", BOX3D(0, 0, 100, 99, 9.9, 199));
    ro.add("mode", "ramp");
    r->setOptions(ro);


    class TestFilter : public Filter
    {
        std::string getName() const
        { return "filters.test"; }

        void filter(PointView& v)
        {
            g_count = v.size();
        }
    };

    TestFilter f;
    Options fo;
    fo.add("where", "X<50");
    f.setOptions(fo);
    f.setInput(*r);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    for (auto vp : s)
        std::cerr << "View size = " << vp->size() << "!\n";
    PointViewPtr v = *s.begin();
    std::cerr << "Filter/Post size = " << g_count << "/" << v->size() << "!\n";
}

} // namespace pdal
