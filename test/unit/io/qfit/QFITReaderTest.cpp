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

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <QfitReader.hpp>
#include "Support.hpp"

#include <iostream>


using namespace pdal;

void Check_Point(const PointView& data,
                 PointId index,
                 double xref, double yref, double zref,
                 int32_t tref)
{
    double x = data.getFieldAs<double>(Dimension::Id::X, index);
    double y = data.getFieldAs<double>(Dimension::Id::Y, index);
    double z = data.getFieldAs<double>(Dimension::Id::Z, index);
    int32_t t = data.getFieldAs<int32_t>(Dimension::Id::OffsetTime, index);

    EXPECT_FLOAT_EQ(x, xref);
    EXPECT_FLOAT_EQ(y, yref);
    EXPECT_FLOAT_EQ(z, zref);
    EXPECT_EQ(t, tref);
}

TEST(QFITReaderTest, test_10_word)
{
    Options options;

    options.add("filename", Support::datapath("qfit/10-word.qi"),
        "Input filename for reader to use");
    options.add("flip_coordinates", false,
        "Flip coordinates from 0-360 to -180-180");
    options.add("scale_z", 0.001f, "Z scale from mm to m");
    options.add("count", 3);

    std::shared_ptr<QfitReader> reader(new QfitReader);
    reader->setOptions(options);
    EXPECT_EQ(reader->getName(), "readers.qfit");

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 3u);

    Check_Point(*view, 0, 221.826822, 59.205160, 32.0900, 0);
    Check_Point(*view, 1, 221.826740, 59.205161, 32.0190, 0);
    Check_Point(*view, 2, 221.826658, 59.205164, 32.0000, 0);
}

TEST(QFITReaderTest, test_14_word)
{
    Options options;

    options.add("filename", Support::datapath("qfit/14-word.qi"),
        "Input filename for reader to use");
    options.add("flip_coordinates", false,
        "Flip coordinates from 0-360 to -180-180");
    options.add("scale_z", 0.001f, "Z scale from mm to m");
    options.add("count", 3);

    PointTable table;
    std::shared_ptr<QfitReader> reader(new QfitReader);
    reader->setOptions(options);
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 3u);

    Check_Point(*view, 0, 244.306337, 35.623317, 1056.830000000, 903);
    Check_Point(*view, 1, 244.306260, 35.623280, 1056.409000000, 903);
    Check_Point(*view, 2, 244.306204, 35.623257, 1056.483000000, 903);
}
