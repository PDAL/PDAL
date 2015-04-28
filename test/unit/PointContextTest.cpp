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

#include <pdal/PointContext.hpp>
#include <las/LasReader.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(PointContext, resolveType)
{
    using namespace Dimension;

    PointContext ctx;

    // Start with a default-defined dimension.
    ctx.registerDim(Id::X);
    EXPECT_EQ(ctx.dimSize(Id::X), 8u);
    EXPECT_EQ(ctx.dimType(Id::X), Type::Double);

    ctx.registerDim(Id::X, Type::Signed32);
    EXPECT_EQ(ctx.dimSize(Id::X), 8u);
    EXPECT_EQ(ctx.dimType(Id::X), Type::Double);

    ctx.registerDim(Dimension::Id::X, Type::Unsigned8);
    EXPECT_EQ(ctx.dimSize(Id::X), 8u);
    EXPECT_EQ(ctx.dimType(Id::X), Type::Double);

    /// Build as we go.
    ctx.registerDim(Id::Y, Type::Unsigned8);
    EXPECT_EQ(ctx.dimSize(Id::Y), 1u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Unsigned8);

    ctx.registerDim(Id::Y, Type::Unsigned8);
    EXPECT_EQ(ctx.dimSize(Id::Y), 1u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Unsigned8);

    ctx.registerDim(Id::Y, Type::Signed8);
    // Signed 8 and Unsigned 8 should yeild signed 16.
    EXPECT_EQ(ctx.dimSize(Id::Y), 2u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Signed16);

    ctx.registerDim(Id::Y, Type::Signed16);
    EXPECT_EQ(ctx.dimSize(Id::Y), 2u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Signed16);

    ctx.registerDim(Id::Y, Type::Float);
    EXPECT_EQ(ctx.dimSize(Id::Y), 4u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Float);

    ctx.registerDim(Id::Y, Type::Double);
    EXPECT_EQ(ctx.dimSize(Id::Y), 8u);
    EXPECT_EQ(ctx.dimType(Id::Y), Type::Double);

    ///
    ctx.registerDim(Id::Z, Type::Unsigned16);
    EXPECT_EQ(ctx.dimSize(Id::Z), 2u);
    EXPECT_EQ(ctx.dimType(Id::Z), Type::Unsigned16);

    ctx.registerDim(Id::Z, Type::Signed8);
    EXPECT_EQ(ctx.dimSize(Id::Z), 4u);
    EXPECT_EQ(ctx.dimType(Id::Z), Type::Signed32);

    ctx.registerDim(Id::Z, Type::Signed16);
    EXPECT_EQ(ctx.dimSize(Id::Z), 4u);
    EXPECT_EQ(ctx.dimType(Id::Z), Type::Signed32);

    ctx.registerDim(Id::Z, Type::Double);
    EXPECT_EQ(ctx.dimSize(Id::Z), 8u);
    EXPECT_EQ(ctx.dimType(Id::Z), Type::Double);
}

TEST(PointContext, userBuffer)
{
    class UserBuf : public RawPtBuf
    {
    private:
        double m_x;
        double m_y;
        double m_z;

    public:
        PointId addPoint()
            { return 0; }
        char *getPoint(PointId /*idx*/)
            { return NULL; }
        void setField(Dimension::Detail *d, PointId /*idx*/, const void *value)
        {
            if (d->id() == Dimension::Id::X)
               m_x = *(double *)value; 
            else if (d->id() == Dimension::Id::Y)
               m_y = *(double *)value; 
            else if (d->id() == Dimension::Id::Z)
               m_z = *(double *)value; 
        }
        void getField(Dimension::Detail *d, PointId /*idx*/, void *value)
        {
            if (d->id() == Dimension::Id::X)
               *(double *)value = m_x; 
            else if (d->id() == Dimension::Id::Y)
               *(double *)value = m_y; 
            else if (d->id() == Dimension::Id::Z)
               *(double *)value = m_z; 
        }

        bool update(Dimension::DetailList& /*detail*/, Dimension::Detail *cur,
            const std::string& /*name*/)
        {
            Dimension::Id::Enum id = cur->id();

            if (id != Dimension::Id::X && id != Dimension::Id::Y &&
                id != Dimension::Id::Z)
                return false;
            cur->setType(Dimension::Type::Double);
            return true;    
        }
    };

    LasReader reader;

    Options opts;
    opts.add("filename", Support::datapath("las/simple.las"));
    opts.add("count", 100);

    reader.setOptions(opts);

    PointContext defCtx;
    reader.prepare(defCtx);
    PointBufferSet pbSet = reader.execute(defCtx);
    PointBufferPtr pb = *pbSet.begin();

    auto readCb = [pb](PointBuffer& buf, PointId id)
    {
        double xDef = pb->getFieldAs<double>(Dimension::Id::X, id);
        double yDef = pb->getFieldAs<double>(Dimension::Id::Y, id);
        double zDef = pb->getFieldAs<double>(Dimension::Id::Z, id);

        double x = buf.getFieldAs<double>(Dimension::Id::X, id);
        double y = buf.getFieldAs<double>(Dimension::Id::Y, id);
        double z = buf.getFieldAs<double>(Dimension::Id::Z, id);

        EXPECT_FLOAT_EQ(xDef, x);
        EXPECT_FLOAT_EQ(yDef, y);
        EXPECT_FLOAT_EQ(zDef, z);
    };

    reader.setReadCb(readCb);
    PointContext ctx(RawPtBufPtr(new UserBuf()));

    reader.prepare(ctx);
    reader.execute(ctx);
}

