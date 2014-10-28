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

#include "UnitTest.hpp"

#include <pdal/PointContext.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PointContextTest)

BOOST_AUTO_TEST_CASE(resolveType)
{
    using namespace Dimension;

    PointContext ctx;

    // Start with a default-defined dimension.
    ctx.registerDim(Id::X);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::X), 8);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::X), Type::Double);

    ctx.registerDim(Id::X, Type::Signed32);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::X), 8);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::X), Type::Double);

    ctx.registerDim(Dimension::Id::X, Type::Unsigned8);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::X), 8);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::X), Type::Double);

    /// Build as we go.
    ctx.registerDim(Id::Y, Type::Unsigned8);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 1);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Unsigned8);

    ctx.registerDim(Id::Y, Type::Unsigned8);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 1);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Unsigned8);

    ctx.registerDim(Id::Y, Type::Signed8);
    // Signed 8 and Unsigned 8 should yeild signed 16.
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 2);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Signed16);

    ctx.registerDim(Id::Y, Type::Signed16);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 2);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Signed16);

    ctx.registerDim(Id::Y, Type::Float);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 4);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Float);

    ctx.registerDim(Id::Y, Type::Double);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Y), 8);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Y), Type::Double);

    ///
    ctx.registerDim(Id::Z, Type::Unsigned16);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Z), 2);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Z), Type::Unsigned16);

    ctx.registerDim(Id::Z, Type::Signed8);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Z), 4);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Z), Type::Signed32);

    ctx.registerDim(Id::Z, Type::Signed16);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Z), 4);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Z), Type::Signed32);

    ctx.registerDim(Id::Z, Type::Double);
    BOOST_CHECK_EQUAL(ctx.dimSize(Id::Z), 8);
    BOOST_CHECK_EQUAL(ctx.dimType(Id::Z), Type::Double);
}


BOOST_AUTO_TEST_SUITE_END()
