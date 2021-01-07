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

#include <pdal/PointTable.hpp>
#include <io/LasReader.hpp>
#include "Support.hpp"

namespace pdal
{

TEST(PointTable, resolveType)
{
    using namespace Dimension;

    PointTable table;
    PointLayoutPtr layout(table.layout());

    // Start with a default-defined dimension.
    layout->registerDim(Id::X);
    EXPECT_EQ(layout->dimSize(Id::X), 8u);
    EXPECT_EQ(layout->dimType(Id::X), Type::Double);

    layout->registerDim(Id::X, Type::Signed32);
    EXPECT_EQ(layout->dimSize(Id::X), 8u);
    EXPECT_EQ(layout->dimType(Id::X), Type::Double);

    layout->registerDim(Dimension::Id::X, Type::Unsigned8);
    EXPECT_EQ(layout->dimSize(Id::X), 8u);
    EXPECT_EQ(layout->dimType(Id::X), Type::Double);

    /// Build as we go.
    layout->registerDim(Id::Intensity, Type::Unsigned8);
    EXPECT_EQ(layout->dimSize(Id::Intensity), 1u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Unsigned8);

    layout->registerDim(Id::Intensity, Type::Unsigned8);
    EXPECT_EQ(layout->dimSize(Id::Intensity), 1u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Unsigned8);

    layout->registerDim(Id::Intensity, Type::Signed8);
    // Signed 8 and Unsigned 8 should yield signed 16.
    EXPECT_EQ(layout->dimSize(Id::Intensity), 2u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Signed16);

    layout->registerDim(Id::Intensity, Type::Signed16);
    EXPECT_EQ(layout->dimSize(Id::Intensity), 2u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Signed16);

    layout->registerDim(Id::Intensity, Type::Float);
    EXPECT_EQ(layout->dimSize(Id::Intensity), 4u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Float);

    layout->registerDim(Id::Intensity, Type::Double);
    EXPECT_EQ(layout->dimSize(Id::Intensity), 8u);
    EXPECT_EQ(layout->dimType(Id::Intensity), Type::Double);

    ///
    layout->registerDim(Id::Red, Type::Unsigned16);
    EXPECT_EQ(layout->dimSize(Id::Red), 2u);
    EXPECT_EQ(layout->dimType(Id::Red), Type::Unsigned16);

    layout->registerDim(Id::Red, Type::Signed8);
    EXPECT_EQ(layout->dimSize(Id::Red), 4u);
    EXPECT_EQ(layout->dimType(Id::Red), Type::Signed32);

    layout->registerDim(Id::Red, Type::Signed16);
    EXPECT_EQ(layout->dimSize(Id::Red), 4u);
    EXPECT_EQ(layout->dimType(Id::Red), Type::Signed32);

    layout->registerDim(Id::Red, Type::Double);
    EXPECT_EQ(layout->dimSize(Id::Red), 8u);
    EXPECT_EQ(layout->dimType(Id::Red), Type::Double);
}

TEST(PointTable, userView)
{
    class UserTable : public PointTable
    {
    private:
        double m_x;
        double m_y;
        double m_z;

    public:
        PointId addPoint()
            { return 0; }
        char *getPoint(PointId idx)
            { return NULL; }
        void setFieldInternal(Dimension::Id id, PointId idx,
            const void *value)
        {
            if (id == Dimension::Id::X)
               m_x = *(const double *)value;
            else if (id == Dimension::Id::Y)
               m_y = *(const double *)value;
            else if (id == Dimension::Id::Z)
               m_z = *(const double *)value;
        }
        void getFieldInternal(Dimension::Id id, PointId idx,
            void *value) const
        {
            if (id == Dimension::Id::X)
               *(double *)value = m_x;
            else if (id == Dimension::Id::Y)
               *(double *)value = m_y;
            else if (id == Dimension::Id::Z)
               *(double *)value = m_z;
        }
    };

    LasReader reader;

    Options opts;
    opts.add("filename", Support::datapath("las/simple.las"));
    opts.add("count", 100);

    reader.setOptions(opts);

    PointTable defTable;
    reader.prepare(defTable);
    PointViewSet viewSet = reader.execute(defTable);
    PointViewPtr defView = *viewSet.begin();

    bool called(false);
    auto readCb = [defView, &called](PointView& customView, PointId id)
    {
        called = true;
        double xDef = defView->getFieldAs<double>(Dimension::Id::X, id);
        double yDef = defView->getFieldAs<double>(Dimension::Id::Y, id);
        double zDef = defView->getFieldAs<double>(Dimension::Id::Z, id);

        double x = customView.getFieldAs<double>(Dimension::Id::X, id);
        double y = customView.getFieldAs<double>(Dimension::Id::Y, id);
        double z = customView.getFieldAs<double>(Dimension::Id::Z, id);

        EXPECT_DOUBLE_EQ(xDef, x);
        EXPECT_DOUBLE_EQ(yDef, y);
        EXPECT_DOUBLE_EQ(zDef, z);
    };

    reader.setReadCb(readCb);
    UserTable table;

    reader.prepare(table);
    reader.execute(table);
    EXPECT_TRUE(called);
}

TEST(PointTable, srs)
{
   SpatialReference srs1("GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]");

   SpatialReference srs2("PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-81],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"32617\"]]");

    PointTable table;

    table.addSpatialReference(srs1);
    table.addSpatialReference(srs1);
    EXPECT_TRUE(table.spatialReferenceUnique());
    EXPECT_EQ(table.anySpatialReference(), srs1);

    table.addSpatialReference(srs2);
    EXPECT_FALSE(table.spatialReferenceUnique());
    EXPECT_EQ(table.anySpatialReference(), srs2);
    EXPECT_EQ(table.m_spatialRefs.size(), 2u);

    table.addSpatialReference(srs1);
    EXPECT_FALSE(table.spatialReferenceUnique());
    EXPECT_EQ(table.anySpatialReference(), srs1);
    EXPECT_EQ(table.m_spatialRefs.size(), 2u);
}

void simpleTest(PointTableRef table)
{
    PointLayoutPtr layout = table.layout();

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::Intensity);
    layout->registerDim(Dimension::Id::Blue);

    PointView v(table);
    for (PointId id = 0; id < 10000; id++)
    {
        if (id % 200 < 100)
        {
            v.setField(Dimension::Id::X, id, id);
            v.setField(Dimension::Id::Y, id, id + 1);
            v.setField(Dimension::Id::Z, id, id + 2);
            v.setField(Dimension::Id::Intensity, id, (id * 100) % 6523);
        }
        else
            v.setField(Dimension::Id::Blue, id, 0);
    }

    for (PointId id = 0; id < 10000; id++)
    {
        if (id % 200 < 100)
        {
            EXPECT_EQ(id, v.getFieldAs<PointId>(Dimension::Id::X, id));
            EXPECT_EQ(id + 1, v.getFieldAs<PointId>(Dimension::Id::Y, id));
            EXPECT_EQ(id + 2, v.getFieldAs<PointId>(Dimension::Id::Z, id));
            EXPECT_EQ((id * 100) % 6523,
                v.getFieldAs<PointId>(Dimension::Id::Intensity, id));
        }
        else
        {
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::X, id));
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Y, id));
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Z, id));
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Intensity, id));
        }
    }
}


TEST(PointTable, simple)
{
    PointTable t;
    simpleTest(t);
}

TEST(PointTable, layoutLimit)
{
    PointTable t;
    PointLayoutPtr layout = t.layout();
    layout->setAllowedDims({ "X", "Z"});

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::Intensity);
    layout->registerDim(Dimension::Id::Blue);
    t.finalize();

    PointView v(t);
    for (PointId id = 0; id < 1000; id++)
    {
        if (id % 200 < 100)
        {
            v.setField(Dimension::Id::X, id, id);
            v.setField(Dimension::Id::Y, id, id + 1);
            v.setField(Dimension::Id::Z, id, id + 2);
            v.setField(Dimension::Id::Intensity, id, (id * 100) % 6523);
        }
        else
        {
            v.setField(Dimension::Id::X, id, 0);
            v.setField(Dimension::Id::Blue, id, id);
        }
    }

    for (PointId id = 0; id < 1000; id++)
    {
        if (id % 200 < 100)
        {
            EXPECT_EQ(id, v.getFieldAs<PointId>(Dimension::Id::X, id));
            EXPECT_EQ(id + 1, v.getFieldAs<PointId>(Dimension::Id::Y, id));
            EXPECT_EQ(id + 2, v.getFieldAs<PointId>(Dimension::Id::Z, id));
        }
        else
        {
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::X, id));
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Y, id));
            EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Z, id));
        }
        EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Intensity, id));
        EXPECT_EQ(0U, v.getFieldAs<PointId>(Dimension::Id::Blue, id));
    }
}

} // namespace
