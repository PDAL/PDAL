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

#include <boost/property_tree/xml_parser.hpp>

#include <pdal/pdal_test_main.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/PDALUtils.hpp>

using namespace pdal;

TEST(BoundsTest, test_ctor)
{
    BOX3D b1;
    EXPECT_TRUE(b1.empty());

    b1.clear();
    BOX3D b2;
    EXPECT_EQ(b1, b2);
}

TEST(BoundsTest, test_equals)
{
    BOX2D b1(1,2,3,4);
    BOX2D b2(1,2,3,4);
    BOX2D b3(1,2,3,6);

    EXPECT_TRUE(b1 == b1);
    EXPECT_TRUE(b1 == b2);
    EXPECT_TRUE(b1 != b3);
    EXPECT_TRUE(b2 != b3);

    BOX3D b4(1,2,3,4,5,6);
    BOX3D b5(1,2,3,4,5,6);
    BOX3D b6(1,2,3,4,5,7);

    EXPECT_TRUE(b4 == b4);
    EXPECT_TRUE(b4 == b5);
    EXPECT_TRUE(b5 == b4);
    EXPECT_TRUE(b4 != b6);
    EXPECT_TRUE(b6 == b6);
}

TEST(BoundsTest, test_copy)
{
    BOX2D b1(1,2,3,4);
    BOX2D b2(b1);
    EXPECT_TRUE(b1==b2);

    BOX3D b3(1,2,3,4,5,6);
    BOX3D b4(b3);
    EXPECT_TRUE(b1==b2);
}

TEST(BoundsTest, test_accessor)
{
    BOX2D b1(1,2,3,4);
    EXPECT_FLOAT_EQ(b1.minx, 1);
    EXPECT_FLOAT_EQ(b1.miny, 2);
    EXPECT_FLOAT_EQ(b1.maxx, 3);
    EXPECT_FLOAT_EQ(b1.maxy, 4);

    BOX3D b2(1,2,3,4,5,6);
    EXPECT_FLOAT_EQ(b2.minx, 1);
    EXPECT_FLOAT_EQ(b2.miny, 2);
    EXPECT_FLOAT_EQ(b2.minz, 3);
    EXPECT_FLOAT_EQ(b2.maxx, 4);
    EXPECT_FLOAT_EQ(b2.maxy, 5);
    EXPECT_FLOAT_EQ(b2.maxz, 6);
}

TEST(BoundsTest, test_clip)
{
    BOX2D r1(0,0,10,10);
    BOX2D r2(1,1,11,11);
    r1.clip(r2);

    BOX2D r3(1,1,10,10);
    EXPECT_TRUE(r1==r3);

    BOX2D r4(2,4,6,8);
    r1.clip(r4);

    EXPECT_TRUE(r1==r4);

    BOX2D r5(20,40,60,80);
    r1.clip(r5);

    // BUG: seems wrong -- need to better define semantics of clip, etc
    // .clip() can make an invalid bounds, this should be fixed.
    BOX2D r6(20,6, 40,8);

    EXPECT_FLOAT_EQ(r1.minx, 20);
    EXPECT_FLOAT_EQ(r1.maxx, 6);
    EXPECT_FLOAT_EQ(r1.miny, 40);
    EXPECT_FLOAT_EQ(r1.maxy, 8);

//ABELL - Need BOX3D example.
}

TEST(BoundsTest, test_intersect)
{
    BOX2D r1(0,0,10,10);
    BOX2D r2(1,1,11,11);
    BOX2D r3(100,100,101,101);
    BOX2D r4(2,4,6,8);

    EXPECT_TRUE(r1.overlaps(r1));

    EXPECT_TRUE(r1.overlaps(r2));
    EXPECT_TRUE(r2.overlaps(r1));

    EXPECT_TRUE(!r1.overlaps(r3));
    EXPECT_TRUE(!r3.overlaps(r1));

    EXPECT_TRUE(r1.contains(r1));
    EXPECT_TRUE(!r1.contains(r2));
    EXPECT_TRUE(r1.contains(r4));

//ABELL - Need BOX3D example.
}

TEST(BoundsTest, test_grow)
{
    BOX2D r1(50,51,100,101);
    BOX2D r2(0,1,10,201);

    r1.grow(r2);

    BOX2D r3(0,1,100,201);
    EXPECT_TRUE(r1 == r3);
//ABELL - Need BOX3D example.
}

TEST(BoundsTest, test_static)
{
    BOX2D t(BOX2D::getDefaultSpatialExtent());
    double mind = (std::numeric_limits<double>::lowest)();
    double maxd = (std::numeric_limits<double>::max)();
    EXPECT_DOUBLE_EQ(t.minx, mind);
    EXPECT_DOUBLE_EQ(t.maxx, maxd);
    EXPECT_DOUBLE_EQ(t.miny, mind);
    EXPECT_DOUBLE_EQ(t.maxy, maxd);

    BOX3D u = BOX3D::getDefaultSpatialExtent();
    EXPECT_DOUBLE_EQ(u.minx, mind);
    EXPECT_DOUBLE_EQ(u.maxx, maxd);
    EXPECT_DOUBLE_EQ(u.miny, mind);
    EXPECT_DOUBLE_EQ(u.maxy, maxd);
    EXPECT_DOUBLE_EQ(u.minz, mind);
    EXPECT_DOUBLE_EQ(u.maxz, maxd);
}

TEST(BoundsTest, test_invalid)
{
    BOX2D t;
    double mind = (std::numeric_limits<double>::lowest)();
    double maxd = (std::numeric_limits<double>::max)();
    EXPECT_DOUBLE_EQ(t.minx, maxd);
    EXPECT_DOUBLE_EQ(t.maxx, mind);
    EXPECT_DOUBLE_EQ(t.miny, maxd);
    EXPECT_DOUBLE_EQ(t.maxy, mind);

    BOX3D u;
    EXPECT_DOUBLE_EQ(u.minx, maxd);
    EXPECT_DOUBLE_EQ(u.maxx, mind);
    EXPECT_DOUBLE_EQ(u.miny, maxd);
    EXPECT_DOUBLE_EQ(u.maxy, mind);
    EXPECT_DOUBLE_EQ(u.minz, maxd);
    EXPECT_DOUBLE_EQ(u.maxz, mind);
}

TEST(BoundsTest, test_output)
{
    const BOX2D b2(1,2,101,102);
    const BOX3D b3(1.1,2.2,3.3,101.1,102.2,103.3);

    std::stringstream ss2(std::stringstream::in | std::stringstream::out);
    std::stringstream ss3(std::stringstream::in | std::stringstream::out);

    ss2 << b2;
    ss3 << b3;

    const std::string out2 = ss2.str();
    const std::string out3 = ss3.str();

    EXPECT_EQ(out2, "([1, 101], [2, 102])");
    EXPECT_EQ(out3, "([1.1, 101.1], [2.2, 102.2], [3.3, 103.3])");
}

TEST(BoundsTest, BoundsTest_ptree)
{
    const BOX2D b2(1,2,101,102);

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);

    boost::property_tree::ptree tree = Utils::toPTree(b2);
    boost::property_tree::write_xml(ss1, tree);

    const std::string out1 = ss1.str();

    static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    const std::string ref = xml_header + "<0><minimum>1</minimum><maximum>101</maximum></0><1><minimum>2</minimum><maximum>102</maximum></1>";

    EXPECT_EQ(ref, out1);
}

TEST(BoundsTest, test_input)
{
    std::stringstream ss("([1.1, 101.1], [2.2, 102.2], [3.3, 103.3])",
        std::stringstream::in | std::stringstream::out);

    BOX3D rr;
    ss >> rr;
    BOX3D r(1.1,2.2,3.3,101.1,102.2,103.3);
    EXPECT_TRUE(r == rr);
}

TEST(BoundsTest, test_lexicalcast_whitespace)
{
    const BOX3D b1 = boost::lexical_cast<BOX3D>("([1,101],[2,102],[3,103])");
    const BOX3D b2 =
        boost::lexical_cast<BOX3D>("([1, 101], [2, 102], [3, 103])");

    EXPECT_EQ(b1, b2);
}

TEST(BoundsTest, test_wkt)
{
    BOX2D b(1.1,2.2,101.1,102.2);
    std::string out = "POLYGON ((1.1 2.2, 1.1 102.2, 101.1 102.2, 101.1 2.2, 1.1 2.2))";
    EXPECT_EQ(b.toWKT(1), out);

    BOX3D b2(1.1,2.2,3.3,101.1,102.2,103.3);
    std::string out2 = "POLYHEDRON Z ( ((1.1 2.2 3.3, 101.1 2.2 3.3, 101.1 102.2 3.3, 1.1 102.2 3.3, 1.1 2.2 3.3, )), ((1.1 2.2 3.3, 101.1 2.2 3.3, 101.1 2.2 103.3, 1.1 2.2 103.3, 1.1 2.2 3.3, )), ((101.1 2.2 3.3, 101.1 102.2 3.3, 101.1 102.2 103.3, 101.1 2.2 103.3, 101.1 2.2 3.3, )), ((101.1 102.2 3.3, 1.1 102.2 3.3, 1.1 102.2 103.3, 101.1 102.2 103.3, 101.1 102.2 3.3, )), ((1.1 102.2 3.3, 1.1 2.2 3.3, 1.1 2.2 103.3, 1.1 102.2 103.3, 1.1 102.2 3.3, )), ((1.1 2.2 103.3, 101.1 2.2 103.3, 101.1 102.2 103.3, 1.1 102.2 103.3, 1.1 2.2 103.3, )) )";
    EXPECT_EQ(b2.toWKT(1), out2);
}

TEST(BoundsTest, test_2d_input)
{
    std::stringstream ss("([1.1, 101.1], [2.2, 102.2])", std::stringstream::in | std::stringstream::out);
    BOX2D rr;
    ss >> rr;
    BOX2D r(1.1,2.2,101.1,102.2);
    EXPECT_EQ(r, rr);
}

TEST(BoundsTest, test_precisionloss)
{
    const BOX2D b1(0.123456789,0.0,0,0);
    EXPECT_DOUBLE_EQ(b1.minx, 0.123456789);

    // convert it to a string, which is what happens
    // when you do something like:
    //   options.getValueOrDefault<BOX3D>("bounds", BOX3D());
    std::ostringstream oss;
    oss << b1; 

    // convert it back
    std::istringstream iss(oss.str());
    BOX2D b2;
    iss >> b2;

    EXPECT_DOUBLE_EQ(b2.minx, 0.123456789);
}
