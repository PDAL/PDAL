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
#include <SbetReader.hpp>
#include <FauxReader.hpp>
#include <RangeFilter.hpp>
#include "Support.hpp"

using namespace pdal;

/**
TEST(Streaming, simple)
{
    class SimpleFilter : public Filter
    {
    public:
        std::string getName() const
            { return "filters.simple"; }
        SimpleFilter() : m_cnt(0)
            {}
    private:
        int m_cnt;
        bool processOne(PointRef point)
        {
            double d = point.getFieldAs<double>(Dimension::Id::X);
            std::cerr << "Value[" << m_cnt++ << "] = " << d << "!\n";
            return true;
        }
    };

    Options ops;
    ops.add("filename", Support::datapath("sbet/autzen_trim.sbet"));
    SbetReader r;
    r.setOptions(ops);

    SimpleFilter f;
    f.setInput(r);

    FixedPointTable t(100);
    f.prepare(t);
    f.execute(t);
}
**/


TEST(Streaming, filter)
{
    Options ro;
    ro.add("bounds", BOX3D(1, 1, 1, 1000, 1000, 1000));
    ro.add("mode", "ramp");
    ro.add("count", 1000);
    FauxReader r;
    r.setOptions(ro);

    Options fo;
    fo.add("limits", "X(100:200], X(300:400], X(500:600], "
        "X(700:800], X(900:1000]");
    RangeFilter f;
    f.setOptions(fo);
    f.setInput(r);

    class Writer : public Filter
    {
    public:
        std::string getName() const
            { return "filterstester"; }
        Writer() : m_cnt(0), m_val(101)
            {}
    private:
        int m_cnt;
        int m_val;

        bool processOne(PointRef point)
        {
            int i = point.getFieldAs<int>(Dimension::Id::X);
            std::cerr << "Testing " << m_val << " against " << i << "!\n";
            EXPECT_EQ(m_val, i);
            if (m_val / 100 == 0)
                m_val += 100;
            m_val++;
            m_cnt++;
            return true;
        }

        void done(PointTableRef)
        {
            EXPECT_EQ(m_cnt, 500);
        }
    };

    Writer w;
    w.setInput(f);

    FixedPointTable t(50);
    f.prepare(t);
    f.execute(t);
}
