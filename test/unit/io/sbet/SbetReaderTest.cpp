/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <SbetReader.hpp>

#include "Support.hpp"

using namespace pdal;

void checkPoint(const PointView& data, PointId index, double time,
    double latitude, double longitude, double altitude, double xvelocity,
    double yvelocity, double zvelocity, double roll, double pitch,
    double heading, double wander, double xaccel, double yaccel,
    double zaccel, double xangrate, double yangrate, double zangrate)
{
    auto checkDimension = [&data,index](Dimension::Id::Enum dim,
        double expected)
    {
        double actual = data.getFieldAs<double>(dim, index);
        EXPECT_FLOAT_EQ(expected, actual);
    };

    checkDimension(Dimension::Id::GpsTime, time);
    checkDimension(Dimension::Id::Y, latitude);
    checkDimension(Dimension::Id::X, longitude);
    checkDimension(Dimension::Id::Z, altitude);
    checkDimension(Dimension::Id::XVelocity, xvelocity);
    checkDimension(Dimension::Id::YVelocity, yvelocity);
    checkDimension(Dimension::Id::ZVelocity, zvelocity);
    checkDimension(Dimension::Id::Roll, roll);
    checkDimension(Dimension::Id::Pitch, pitch);
    checkDimension(Dimension::Id::PlatformHeading, heading);
    checkDimension(Dimension::Id::WanderAngle, wander);
    checkDimension(Dimension::Id::XBodyAccel, xaccel);
    checkDimension(Dimension::Id::YBodyAccel, yaccel);
    checkDimension(Dimension::Id::ZBodyAccel, zaccel);
    checkDimension(Dimension::Id::XBodyAngRate, xangrate);
    checkDimension(Dimension::Id::YBodyAngRate, yangrate);
    checkDimension(Dimension::Id::ZBodyAngRate, zangrate);
}

TEST(SbetReaderTest, testRead)
{
    Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    Options options(filename);
    std::shared_ptr<SbetReader> reader(new SbetReader);
    reader->setOptions(options);

    PointTable table;

    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 2u);

    checkPoint(*view.get(), 0,
               1.516310028360710e+05, 5.680211852972264e-01,
               -2.041654392303940e+00, 1.077152953296560e+02,
               -2.332420866600025e+00, -3.335067504871401e-01,
               -3.093961631767838e-02, -2.813407149321339e-02,
               -2.429905393889139e-02, 3.046773230278662e+00,
               -2.198414736922658e-02, 7.859639737752390e-01,
               7.849084719295495e-01, -2.978807916450262e-01,
               6.226807982589819e-05, 9.312162756440178e-03,
               7.217812320996525e-02);
    checkPoint(*view.get(), 1,
               1.516310078318641e+05, 5.680211834722869e-01,
               -2.041654392034053e+00, 1.077151424357507e+02,
               -2.336228229691271e+00, -3.324663118952635e-01,
               -3.022948961008987e-02, -2.813856631423094e-02,
               -2.425215669392169e-02, 3.047131105236811e+00,
               -2.198416007932108e-02, 8.397590491636475e-01,
               3.252165276637165e-01, -1.558883225990844e-01,
               8.379685112283802e-04, 7.372886784718076e-03,
               7.179027672314571e-02);
}

TEST(SbetReaderTest, testBadFile)
{
    Option filename("filename", Support::datapath("sbet/badfile.sbet"), "");
    Options options(filename);
    std::shared_ptr<SbetReader> reader(new SbetReader);
    reader->setOptions(options);
    PointTable table;
    reader->prepare(table);
    EXPECT_THROW(reader->execute(table), pdal_error);
}

TEST(SbetReaderTest, testPipeline)
{
    PipelineManager manager;
    PipelineReader reader(manager);
    reader.readPipeline(Support::configuredpath("sbet/pipeline.xml"));

    point_count_t numPoints = manager.execute();
    EXPECT_EQ(numPoints, 2u);
    FileUtils::deleteFile(Support::datapath("sbet/outfile.txt"));
}
