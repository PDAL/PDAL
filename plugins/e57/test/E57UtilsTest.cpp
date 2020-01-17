/******************************************************************************
* Copyright (c) 2019, Helix Re Inc.
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
*     * Neither the name of Helix Re Inc. nor the
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

#include <gtest/gtest.h>

#include "plugins/e57/io/Utils.hpp"

using namespace pdal::e57plugin;

TEST(E57Utils, e57ToPdalTranslation)
{
    ASSERT_EQ(e57ToPdal("cartesianX"),pdal::Dimension::Id::X);
    ASSERT_EQ(e57ToPdal("fake"),pdal::Dimension::Id::Unknown);
    ASSERT_EQ(e57ToPdal("cartesianY"),pdal::Dimension::Id::Y);
    ASSERT_EQ(e57ToPdal("cartesianZ"),pdal::Dimension::Id::Z);
    ASSERT_EQ(e57ToPdal("colorRed"),pdal::Dimension::Id::Red);
    ASSERT_EQ(e57ToPdal("colorGreen"),pdal::Dimension::Id::Green);
    ASSERT_EQ(e57ToPdal("colorBlue"),pdal::Dimension::Id::Blue);
    ASSERT_EQ(e57ToPdal("intensity"),pdal::Dimension::Id::Intensity);
}

TEST(E57Utils, pdalToE57Translation)
{
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::X),"cartesianX");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Y),"cartesianY");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Z),"cartesianZ");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Red),"colorRed");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Green),"colorGreen");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Blue),"colorBlue");
    ASSERT_EQ(pdalToE57(pdal::Dimension::Id::Intensity),"intensity");
}

TEST(E57Utils, getPdalBounds)
{
    using pdal::Dimension::Id;
    auto pdalTypes = {Id::Red,Id::Green,Id::Blue,Id::Intensity, Id::Classification};
    for (auto type: pdalTypes)
    {
        ASSERT_NO_THROW(getPdalBounds(type));
    }
    ASSERT_ANY_THROW(getPdalBounds(pdal::Dimension::Id::X));
    ASSERT_ANY_THROW(getPdalBounds(pdal::Dimension::Id::Unknown));
}
