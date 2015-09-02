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
#include <las/LasReader.hpp>
#include "Support.hpp"

using namespace pdal;

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
    layout->registerDim(Id::Y, Type::Unsigned8);
    EXPECT_EQ(layout->dimSize(Id::Y), 1u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Unsigned8);

    layout->registerDim(Id::Y, Type::Unsigned8);
    EXPECT_EQ(layout->dimSize(Id::Y), 1u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Unsigned8);

    layout->registerDim(Id::Y, Type::Signed8);
    // Signed 8 and Unsigned 8 should yeild signed 16.
    EXPECT_EQ(layout->dimSize(Id::Y), 2u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Signed16);

    layout->registerDim(Id::Y, Type::Signed16);
    EXPECT_EQ(layout->dimSize(Id::Y), 2u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Signed16);

    layout->registerDim(Id::Y, Type::Float);
    EXPECT_EQ(layout->dimSize(Id::Y), 4u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Float);

    layout->registerDim(Id::Y, Type::Double);
    EXPECT_EQ(layout->dimSize(Id::Y), 8u);
    EXPECT_EQ(layout->dimType(Id::Y), Type::Double);

    ///
    layout->registerDim(Id::Z, Type::Unsigned16);
    EXPECT_EQ(layout->dimSize(Id::Z), 2u);
    EXPECT_EQ(layout->dimType(Id::Z), Type::Unsigned16);

    layout->registerDim(Id::Z, Type::Signed8);
    EXPECT_EQ(layout->dimSize(Id::Z), 4u);
    EXPECT_EQ(layout->dimType(Id::Z), Type::Signed32);

    layout->registerDim(Id::Z, Type::Signed16);
    EXPECT_EQ(layout->dimSize(Id::Z), 4u);
    EXPECT_EQ(layout->dimType(Id::Z), Type::Signed32);

    layout->registerDim(Id::Z, Type::Double);
    EXPECT_EQ(layout->dimSize(Id::Z), 8u);
    EXPECT_EQ(layout->dimType(Id::Z), Type::Double);
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
        void setField(const Dimension::Detail *d, PointId idx, const void *value)
        {
            if (d->id() == Dimension::Id::X)
               m_x = *(const double *)value;
            else if (d->id() == Dimension::Id::Y)
               m_y = *(const double *)value;
            else if (d->id() == Dimension::Id::Z)
               m_z = *(const double *)value;
        }
        void getField(const Dimension::Detail *d, PointId idx, void *value)
        {
            if (d->id() == Dimension::Id::X)
               *(double *)value = m_x;
            else if (d->id() == Dimension::Id::Y)
               *(double *)value = m_y;
            else if (d->id() == Dimension::Id::Z)
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

        EXPECT_FLOAT_EQ(xDef, x);
        EXPECT_FLOAT_EQ(yDef, y);
        EXPECT_FLOAT_EQ(zDef, z);
    };

    reader.setReadCb(readCb);
    UserTable table;

    reader.prepare(table);
    reader.execute(table);
    EXPECT_TRUE(called);
}

