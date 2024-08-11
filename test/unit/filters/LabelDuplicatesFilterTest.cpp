/******************************************************************************
* Copyright (c) 2024, Hobu Inc. (hobu@hobu.co)
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

#include <random>

#include <io/TextReader.hpp>
#include <filters/SortFilter.hpp>
#include <filters/LabelDuplicatesFilter.hpp>
#include "Support.hpp"

namespace pdal
{


void testDimensions(std::string const& data, std::string const& dimensions)
{

    TextReader t;
    Options textOptions;
    textOptions.add("filename", Support::datapath(data));
    t.setOptions(textOptions);


    PointTable table;

    StringList splitDimension = Utils::split2(dimensions, ',');

    Stage* prev = &t;
    for (auto dimension: splitDimension)
    {
        SortFilter* filter = new SortFilter();
        Options opts;
        opts.add("dimension", dimension);
        filter->setOptions(opts);
        prev->setInput(*filter);
    }

    Options opts;
    opts.add("dimensions", dimensions);
    LabelDuplicatesFilter filter;
    filter.setOptions(opts);
    prev->setInput(filter);


    prev->prepare(table);
    PointViewSet views = filter.execute(table);
    PointViewPtr view = *views.begin();

    for (PointId i = dimensions.size(); i < view->size(); ++i) {
        EXPECT_EQ(1, view->getFieldAs<uint8_t>(Dimension::Id::Duplicate, i)); }

}

TEST(LabelDuplicatesFilterTest, sorted)
{
    std::string data("text/duplicates-sorted.txt");
    testDimensions(data, "X");
    testDimensions(data, "X,Y");
    testDimensions(data, "X,Y,Z");
    testDimensions(data, "X,Y,Z,GpsTime");
    testDimensions(data, "X,Y,Z,GpsTime,PointSourceId,UserData");
}

TEST(LabelDuplicatesFilterTest, unsorted)
{
    std::string data("text/duplicates-unsorted.txt");
    testDimensions(data, "X");
    testDimensions(data, "X,Y");
    testDimensions(data, "X,Y,Z");
    testDimensions(data, "X,Y,Z,GpsTime");
    testDimensions(data, "X,Y,Z,GpsTime,PointSourceId,UserData");
}

}
