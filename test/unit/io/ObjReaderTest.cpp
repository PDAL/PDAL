/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#include <pdal/Filter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>

#include <io/ObjReader.hpp>
#include "Support.hpp"

namespace pdal
{


void checkPoint(const PointViewPtr& view, point_count_t idx,
        double x, double y, double z)
{
    EXPECT_DOUBLE_EQ(x, view->getFieldAs<double>(Dimension::Id::X, idx));
    EXPECT_DOUBLE_EQ(y, view->getFieldAs<double>(Dimension::Id::Y, idx));
    EXPECT_DOUBLE_EQ(z, view->getFieldAs<double>(Dimension::Id::Z, idx));
}


TEST(ObjReader, Constructor)
{
    ObjReader reader1;

    StageFactory f;
    Stage* reader2(f.createStage("readers.obj"));
    EXPECT_TRUE(reader2);
}



TEST(ObjReader, ReadBinary)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/simple_binary.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 3u);

    checkPoint(view, 0, -1, 0, 0);
    checkPoint(view, 1, 0, 1, 0);
    checkPoint(view, 2, 1, 0, 0);
}


TEST(ObjReader, ReadBinaryStream)
{
    class Checker : public Filter, public Streamable
    {
    public:
        std::string getName() const
            { return "checker"; }
    private:
        bool processOne(PointRef& point)
        {
            static int cnt = 0;
            if (cnt == 0)
            {
                EXPECT_DOUBLE_EQ(-1,
                    point.getFieldAs<double>(Dimension::Id::X));
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::Y));
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::Z));
            }
            if (cnt == 1)
            {
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::X));
                EXPECT_DOUBLE_EQ(1,
                    point.getFieldAs<double>(Dimension::Id::Y));
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::Z));
            }
            if (cnt == 2)
            {
                EXPECT_DOUBLE_EQ(1,
                    point.getFieldAs<double>(Dimension::Id::X));
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::Y));
                EXPECT_DOUBLE_EQ(0,
                    point.getFieldAs<double>(Dimension::Id::Z));
            }
            cnt++;
            return true;
        }
    };

    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/simple_binary.obj"));
    reader.setOptions(options);

    FixedPointTable table(10);

    Checker c;
    c.setInput(reader);

    c.prepare(table);
    c.execute(table);
}


TEST(ObjReader, NoVertex)
{
    ObjReader reader;
    Options options;
    options.add("filename", Support::datapath("obj/no_vertex.obj"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 0u);

}

}
