/******************************************************************************
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

#include <io/BufferReader.hpp>
#include <io/TextReader.hpp>
#include <filters/ELMFilter.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ELMFilterTest, test1)
{
    Options readerOps;
    readerOps.add("filename",
        Support::datapath("filters/elm1.txt"));

    TextReader reader;
    reader.setOptions(readerOps);

    ELMFilter filter;
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 10u);
    int noise(0);
    for (size_t i = 0; i < view->size(); ++i)
    {
        uint8_t c = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        if (c == ClassLabel::LowPoint)
            noise++;
    }
    EXPECT_EQ(noise, 2);
}

TEST(ELMFilterTest, test2)
{
    Options readerOps;
    readerOps.add("filename",
        Support::datapath("filters/elm2.txt"));

    TextReader reader;
    reader.setOptions(readerOps);

    ELMFilter filter;
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 17u);
    int noise(0);
    for (size_t i = 0; i < view->size(); ++i)
    {
        uint8_t c = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);
        if (c == ClassLabel::LowPoint)
            noise++;
    }
    EXPECT_EQ(noise, 7);
}

TEST(ELMFilterTest, emptyView)
{
    PointTable table;
    table.layout()->registerDims(
        {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z});

    PointViewPtr view(new PointView(table));
    BufferReader reader;
    reader.addView(view);

    StageFactory factory;
    Stage* filter(factory.createStage("filters.elm"));
    filter->setInput(reader);
    filter->prepare(table);

    PointViewSet s = filter->execute(table);
    EXPECT_EQ(s.size(), 1u);

    PointViewPtr v = *s.begin();
    EXPECT_EQ(v->size(), 0u);
}
