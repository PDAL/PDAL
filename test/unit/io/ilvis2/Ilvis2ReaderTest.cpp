/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>

#include <Ilvis2Reader.hpp>

#include "Support.hpp"

using namespace pdal;

void checkPoint(const PointView& data, PointId index, double time,
    double latitude, double longitude, double altitude)
{
    auto checkDimension = [&data,index](Dimension::Id::Enum dim,
        double expected)
    {
        double actual = data.getFieldAs<double>(dim, index);
        EXPECT_FLOAT_EQ(expected, actual);
    };

    checkDimension(Dimension::Id::Y, latitude);
    checkDimension(Dimension::Id::X, longitude);
    checkDimension(Dimension::Id::Z, altitude);
    checkDimension(Dimension::Id::GpsTime, time);
}

TEST(Ilvis2ReaderTest, testRead)
{
    Option filename("filename", Support::datapath("ilvis2/ILVIS2_GL2009_0414_R1401_042504.TXT"), "");
    Options options(filename);
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;

    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 998u);

    checkPoint(*view.get(), 0, 42504.48313,
             78.307673,121.21479,1956.583
            );

    checkPoint(*view.get(), 996, 42520.90035,
             78.320149, 121.32024, 1959.206
            );
}

