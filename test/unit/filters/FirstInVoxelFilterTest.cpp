/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re, https://github.com/pravinshinde825)
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

#include <pdal/StageFactory.hpp>
#include<pdal/Reader.hpp>
#include<pdal/Streamable.hpp>
#include"filters/FirstInVoxelFilter.hpp"

#include "Support.hpp"

namespace pdal
{

TEST(FirstInVoxelFilterTest, standard)
{
    StageFactory fac;

    Stage *reader = fac.createStage("readers.las");
    Options ro;
    ro.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(ro);

    Stage *filter = fac.createStage("filters.first-in-voxel");
    Options fo;
    fo.add("cell", 5);
    filter->setOptions(fo);
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet set = filter->execute(t);
    EXPECT_EQ(set.size(), 1U);
    PointViewPtr v = *set.begin();
    PointRef p=v->point(0);
    EXPECT_EQ(v->size(), 5768U);
}

TEST(FirstInVoxelFilterTest, stream)
{
    using namespace Dimension;

    FixedPointTable table(2);
    table.layout()->registerDim(Id::X);
    table.layout()->registerDim(Id::Y);
    table.layout()->registerDim(Id::Z);

    class StreamReader : public Reader, public Streamable
    {
    public:
        std::string getName() const
        {
            return "readers.stream";
        }
        bool processOne(PointRef& point)
        {
            static int i = 0;

            if (i == 0)
            {
                point.setField(Id::X, 2);
                point.setField(Id::Y, 2);
            }
            else if (i == 1)
            {
                point.setField(Id::X, 6);
                point.setField(Id::Y, 2);
            }
            else if (i == 2)
            {
                point.setField(Id::X, 8);
                point.setField(Id::Y, 2);
            }
            else if (i == 3)
            {
                point.setField(Id::X, 10);
                point.setField(Id::Y, 2);
            }
            else if (i == 4)
            {
                point.setField(Id::X, 12);
                point.setField(Id::Y, 2);
            }
            else if (i == 5)
            {
                point.setField(Id::X, 105);
                point.setField(Id::Y, 105);
            }
            else
                return false;
            i++;
            return true;
        }
    };

    StreamReader r;

    FirstInVoxelFilter voxelfilter;
    Options o;
    o.add("cell", 5);
    voxelfilter.setInput(r);
    voxelfilter.setOptions(o);

    class TestFilter : public Filter, public Streamable
    {
    public:
        std::string getName() const
        {
            return "filters.testfilter";
        }
        point_count_t m_count;

    private:
        virtual void ready(PointTableRef)
        {
            m_count = 0;
        }

        virtual bool processOne(PointRef& point)
        {
            if (m_count == 0)
            {
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 2);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            if (m_count == 1)
            {
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 8);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            if (m_count == 2)
            {
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 12);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            if (m_count == 3)
            {
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 105);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 105);
            }
            m_count++;
            return true;
        }
    };

    TestFilter f;
    f.setInput(voxelfilter);

    f.prepare(table);
    f.execute(table);
    EXPECT_EQ(f.m_count, (point_count_t)4);
}

} // namespace
