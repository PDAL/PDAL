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

#include <io/FauxReader.hpp>
#include <filters/ColorinterpFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

std::string makeColor()
{
    // Red is specified.  Green and blue are 0.
    static uint8_t rgb[12] = { 1, 2, 3, 4 };

    std::string ptr = Utils::toString(&rgb);
    if (ptr[0] != '0' || ptr[1] != 'x')
        ptr = "0x" + ptr;
    std::ostringstream ss;
    ss << "MEM:::PIXELS=4,LINES=1,BANDS=3,DATAPOINTER=" << ptr;
    return ss.str();
}

} // unnamed namespace


TEST(ColorinterpFilterTest, minmax)
{
    FauxReader f;
    Options options;

    options.add("count", 101);
    options.add("mode", "ramp");
    options.add("bounds", "([0,100],[0,100],[0,100])");

    f.setOptions(options);

    ColorinterpFilter c;
    Options coptions;

    coptions.add("minimum", 0);
    coptions.add("maximum", 99);
    coptions.add("ramp", makeColor());

    c.setOptions(coptions);

    StreamCallbackFilter s;

    c.setInput(f);
    s.setInput(c);

    auto cb  = [](PointRef& point)
    {
        int z = point.getFieldAs<int>(Dimension::Id::Z);
        int r = point.getFieldAs<int>(Dimension::Id::Red);
        int g = point.getFieldAs<int>(Dimension::Id::Green);
        int b = point.getFieldAs<int>(Dimension::Id::Blue);

        if (z != 100)
        {
            std::stringstream ss;
            ss << "Z = " << z << "; R = " << r;
            SCOPED_TRACE(ss.str());
            EXPECT_EQ((int)(z / 25) + 1, r);
        }
        if (z == 100)
            EXPECT_EQ(0, r);
        EXPECT_EQ(0, g);
        EXPECT_EQ(0, b);

        return true;
    };

    s.setCallback(cb);

    FixedPointTable t(10);
    s.prepare(t);
    s.execute(t);

    PointTable t2;

    s.prepare(t2);
    s.execute(t2);
}

TEST(ColorinterpFilterTest, missingz)
{
    ColorinterpFilter c;
    Options coptions;

    coptions.add("minimum", 0);
    coptions.add("maximum", 0);
    coptions.add("ramp", makeColor());
    c.setOptions(coptions);

    PointTable t;
    EXPECT_THROW(c.prepare(t), pdal_error);
}

TEST(ColorinterpFilterTest, badramp)
{
    FauxReader f;
    Options options;

    options.add("count", 100);
    options.add("mode", "ramp");
    options.add("bounds", "([0,99],[0,99],[0,99])");

    f.setOptions(options);

    ColorinterpFilter c;
    Options coptions;

    coptions.add("minimum", 0);
    coptions.add("maximum", 100);
    coptions.add("ramp", "ramp_that_doesnt_exist");
    c.setOptions(coptions);
    c.setInput(f);

    PointTable t;
    c.prepare(t);
    EXPECT_THROW(c.execute(t), pdal_error);
}

TEST(ColorinterpFilterTest, cantstream)
{
    FauxReader f;
    Options options;

    options.add("count", 100);
    options.add("mode", "ramp");
    options.add("bounds", "([0,99],[0,99],[0,99])");
    f.setOptions(options);

    ColorinterpFilter c;
    Options coptions;

    coptions.add("minimum", 0);
    coptions.add("ramp", makeColor());
    c.setOptions(coptions);
    c.setInput(f);

    FixedPointTable t(10);
    c.prepare(t);
    EXPECT_FALSE(c.pipelineStreamable());
}

namespace
{

void standardTest(Options& coptions, std::function<bool(int, int)> test)
{
    FauxReader f;
    Options options;

    options.add("count", 100);
    options.add("mode", "ramp");
    options.add("bounds", "([0,99],[0,99],[0,99])");

    f.setOptions(options);

    ColorinterpFilter c;

    c.setOptions(coptions);

    StreamCallbackFilter s;

    c.setInput(f);
    s.setInput(c);

    auto cb  = [test](PointRef& point)
    {
        int z = point.getFieldAs<int>(Dimension::Id::Z);
        int r = point.getFieldAs<int>(Dimension::Id::Red);
        int g = point.getFieldAs<int>(Dimension::Id::Green);
        int b = point.getFieldAs<int>(Dimension::Id::Blue);

        std::stringstream ss;
	ss << "Z = " << z << "; R = " << r;
        SCOPED_TRACE(ss.str());
        EXPECT_TRUE(test(z, r));
        EXPECT_EQ(0, g);
        EXPECT_EQ(0, b);
        return true;
    };

    s.setCallback(cb);

    PointTable t;

    s.prepare(t);
    s.execute(t);

}

} // unnamed namespace

TEST(ColorinterpFilterTest, autorange)
{
    Options coptions;

    coptions.add("ramp", makeColor());

    auto test = [](int z, int r)
    {
        return ((int)(z / 25) + 1 == r);
    };

    SCOPED_TRACE("autorange");
    standardTest(coptions, test);
}

TEST(ColorinterpFilterTest, k)
{
    Options coptions;

    coptions.add("ramp", makeColor());
    coptions.add("k", 1);

    auto test = [](int z, int r)
    {
        if (z < 22 && r == 0)
            return true;
        if (z < 36 && r == 1)
            return true;
        if (z < 51 && r == 2)
            return true;
        if (z < 65 && r == 3)
            return true;
        if (z < 80 && r == 4)
            return true;
        if (z >= 80 && r == 0)
            return true;
        return false;
    };

    SCOPED_TRACE("k");
    standardTest(coptions, test);
}

TEST(ColorinterpFilterTest, mad)
{
    Options coptions;

    coptions.add("ramp", makeColor());
    coptions.add("mad", "true");
    coptions.add("k", 1);

    auto test = [](int z, int r)
    {
        if (z < 13 && r == 0)
            return true;
        if (z < 32 && r == 1)
            return true;
        if (z < 50 && r == 2)
            return true;
        if (z < 69 && r == 3)
            return true;
        else if (z < 88 && r == 4)
            return true;
        else if (z >= 88 && r == 0)
            return true;
        return false;
    };

    SCOPED_TRACE("mad");
    standardTest(coptions, test);
}

