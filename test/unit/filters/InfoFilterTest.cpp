/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (hobu@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <filters/InfoFilter.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

namespace pdal
{

MetadataNode run(Options& fOpts)
{
    StageFactory factory;
    
    Stage *r = factory.createStage("readers.las");
    Options rOpts;
    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    r->setOptions(rOpts);

    Stage *f = factory.createStage("filters.info");
    f->setOptions(fOpts);
    f->setInput(*r);

    FixedPointTable t(1000);

    f->prepare(t);
    f->execute(t);
    return f->getMetadata();
}

TEST(InfoFilterTest, point)
{
    struct rgb
    {
        int r;
        int g;
        int b;

        bool operator == (const rgb& o) const
        {
            return o.r == r && o.g == g && o.b == b;
        }
    };

    std::vector<rgb> vtest { {84, 102, 93}, {82, 98, 90}, {80, 96, 90},
        {79, 96, 90}, {78, 94, 89}, {82, 98, 90}, {80, 98, 90},
        {89, 106, 99}, {80, 100, 90}, {77, 93, 86} };
    std::vector<rgb> v;

    Options fOpts;
    fOpts.add("point", "0-9");
    MetadataNode m = run(fOpts);
    MetadataNode p = m.findChild("points");
    MetadataNodeList l = p.children();
    EXPECT_EQ(l.size(), 10U);
    for (MetadataNode& n : l)
    {
        int r = n.findChild("Red").value<int>();
        int g = n.findChild("Green").value<int>();
        int b = n.findChild("Blue").value<int>();
        v.push_back({r, g, b});
    }
    for (size_t i = 0; i < vtest.size(); ++i)
        EXPECT_EQ(v[i], vtest[i]);
}

TEST(InfoFilterTest, query)
{
    std::vector<int> v;
    std::vector<int> vtest { 107596, 108135, 107595, 108136, 107565, 107566,
        108164, 108134, 107597, 108163 };

    Options fOpts;
    fOpts.add("query", "636133,849000/10");
    MetadataNode m = run(fOpts);
    MetadataNode p = m.findChild("points");
    MetadataNodeList l = p.children();
    EXPECT_EQ(l.size(), 10U);
    for (MetadataNode& n : l)
        v.push_back(n.findChild("PointId").value<int>());
    std::sort(v.begin(), v.end());
    std::sort(vtest.begin(), vtest.end());
    for (size_t i = 0; i < vtest.size(); ++i)
        EXPECT_EQ(v[i], vtest[i]);
}

TEST(InfoFilterTest, direct_bounds)
{
    StageFactory factory;
    
    Stage *r = factory.createStage("readers.las");
    Options rOpts;
    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    r->setOptions(rOpts);

    Stage *f = factory.createStage("filters.info");
    f->setInput(*r);

    FixedPointTable t(1000);

    f->prepare(t);
    f->execute(t);

    BOX3D box = dynamic_cast<InfoFilter *>(f)->bounds();
    EXPECT_EQ(box.minx, 636001.76);
    EXPECT_EQ(box.maxz, 520.51);
}

TEST(InfoFilterTest, misc)
{
    Options fOpts;
    MetadataNode m = run(fOpts);

    MetadataNodeList s = m.findChild("schema").children();
    EXPECT_EQ(s.size(), 16U);
    if (s.size() == 16U)
    {
        auto orderbyname = [](const MetadataNode& m1, const MetadataNode& m2)
        {
            MetadataNode m1c = m1.findChild("name");
            MetadataNode m2c = m2.findChild("name");
            return m1c.value() < m2c.value();
        };
        std::sort(s.begin(), s.end(), orderbyname);
        std::vector<std::string> dims { "Blue", "Classification",
            "EdgeOfFlightLine", "GpsTime", "Green", "Intensity",
            "NumberOfReturns", "PointSourceId", "Red", "ReturnNumber",
            "ScanAngleRank", "ScanDirectionFlag", "UserData", "X", "Y", "Z" };

        size_t i = 0;
        for (MetadataNode& m : s)
            EXPECT_EQ(m.findChild("name").value(), dims[i++]);
    }
    MetadataNode n = m.findChild("bbox");
    EXPECT_EQ(n.findChild("maxz").value(), "520.51");

    EXPECT_TRUE(m.findChild("dimensions").valid());

    EXPECT_TRUE(m.findChild("srs:compoundwkt").valid());
}

} // namespace pdal
