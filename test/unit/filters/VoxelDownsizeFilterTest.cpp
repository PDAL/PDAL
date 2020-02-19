/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re,
 *    https://github.com/pravinshinde825)
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
#include <pdal/Streamable.hpp>
#include "io/BufferReader.hpp"
#include "filters/VoxelDownsizeFilter.hpp"

#include "Support.hpp"

namespace pdal
{

using namespace Dimension;
void standard_test(std::string mode) {
    StageFactory fac;

    Stage* reader = fac.createStage("readers.las");
    Options ro;
    ro.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(ro);

    Stage* filter = fac.createStage("filters.voxeldownsize");
    Options fo;
    fo.add("cell", 10);
    fo.add("mode", mode);
    filter->setOptions(fo);
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet set = filter->execute(t);
    EXPECT_EQ(set.size(), 1U);
    PointViewPtr v = *set.begin();
    EXPECT_EQ(v->size(), 7788U);
}

void origin_test(std::string mode)
{
    using namespace Dimension;
    PointTable t;
    t.layout()->registerDims({Id::X, Id::Y, Id::Z});
    PointViewPtr v(new PointView(t));

    std::vector<std::array<double, 3>> plist 
        { { 5, 5, 5 },
          { 1, 1, -1 },
          { 1, -1, 1 },
          { 1, -1, -1 },
          { -1, 1, 1 },
          { -1, 1, -1 },
          { -1, -1, 1 },
          { -1, -1, -1 },
          { 1, 1, 1 } };

    PointId id = 0;
    for (std::array<double, 3>& p : plist)
    {
        v->setField(Id::X, id, p[0]);
        v->setField(Id::Y, id, p[1]);
        v->setField(Id::Z, id, p[2]);
        ++id;
    }

    BufferReader r;
    r.addView(v);

    VoxelDownsizeFilter f;
    Options o;
    o.add("cell", 10);
    o.add("mode", mode);
    f.setOptions(o);
    f.setInput(r);

    f.prepare(t);
    PointViewSet s = f.execute(t);
    EXPECT_EQ(s.size(), 1u);
    v = *s.begin();
    EXPECT_EQ(v->size(), 8u);
    if (mode == "center")
    {
        PointId id = 0;
        for (PointRef p : *v)
        {
            double x = p.getFieldAs<double>(Dimension::Id::X);
            double y = p.getFieldAs<double>(Dimension::Id::Y);
            double z = p.getFieldAs<double>(Dimension::Id::Z);
            std::cerr << "x/y/z = " << x << "/" << y << "/" << z << "!\n";
            /**
            EXPECT_EQ(p.getFieldAs<double>(Dimension::Id::X),
                plist[id][0] * 
            **/
            id++;
        }
    }
}

void stream_test(std::string mode) {
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
				point.setField(Id::X, 8);
				point.setField(Id::Y, 2);
			}
			else if (i == 2)
			{
				point.setField(Id::X, 10);
				point.setField(Id::Y, 2);
			}
			else
				return false;
			i++;
			return true;
		}
	};

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
                std::cerr << "0 Point ID = " << point.pointId() << "!\n";
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 2);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            if (m_count == 1)
            {
                std::cerr << "1 Point ID = " << point.pointId() << "!\n";
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 8);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            if (m_count == 2)
            {
                std::cerr << "2 Point ID = " << point.pointId() << "!\n";
                EXPECT_EQ(point.getFieldAs<int>(Id::X), 10);
                EXPECT_EQ(point.getFieldAs<int>(Id::Y), 2);
            }
            m_count++;
            return true;
        }
    };

    StreamReader r;
    VoxelDownsizeFilter voxelfilter;
    Options o;
    o.add("cell", 5);
    o.add("mode", mode);
    voxelfilter.setInput(r);
    voxelfilter.setOptions(o);

    FixedPointTable table(2);
    table.layout()->registerDim(Id::X);
    table.layout()->registerDim(Id::Y);
    table.layout()->registerDim(Id::Z);

    TestFilter f;
    f.setInput(voxelfilter);
    f.prepare(table);
    f.execute(table);
}

TEST(VoxelDownsizeFilter, firstinvoxel_origin)
{
    origin_test("first");
}

TEST(VoxelDownsizeFilter, voxelcenter_origin)
{
    origin_test("center");
}

/**
TEST(VoxelDownsizeFilter, firstinvoxel_stream)
{
    stream_test("firstinvoxel");
}

TEST(VoxelDownsizeFilter, firstinvoxel_standard)
{
    standard_test("firstinvoxel");
}

TEST(VoxelDownsizeFilter, voxelcenter_standard)
{
    standard_test("voxelcenter");
}

TEST(VoxelDownsizeFilter, voxelcenter_stream)
{
    stream_test("voxelcenter");
}
**/

} // namespace
