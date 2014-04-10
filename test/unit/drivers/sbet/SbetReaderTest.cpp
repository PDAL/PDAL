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

#include <boost/test/unit_test.hpp>

#include <pdal/Options.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointBuffer.hpp>

#include <pdal/drivers/sbet/Reader.hpp>

#include "Support.hpp"


void checkDimension(const pdal::PointBuffer& data,
                    std::size_t index, const std::string& name, double expected)
{
    pdal::Dimension const& dimension = data.getSchema().getDimension(name);
    double actual = data.getField<double>(dimension, index);
    BOOST_CHECK_CLOSE(expected, actual, 0.000001);
}


void checkPoint(const pdal::PointBuffer& data, std::size_t index,
                double time, double latitude, double longitude,
                double altitude,
                double xvelocity, double yvelocity, double zvelocity,
                double roll, double pitch, double heading,
                double wander,
                double xaccel, double yaccel, double zaccel,
                double xangrate, double yangrate, double zangrate)
{
    checkDimension(data, index, "Time", time);
    checkDimension(data, index, "Y", latitude);
    checkDimension(data, index, "X", longitude);
    checkDimension(data, index, "Z", altitude);
    checkDimension(data, index, "XVelocity", xvelocity);
    checkDimension(data, index, "YVelocity", yvelocity);
    checkDimension(data, index, "ZVelocity", zvelocity);
    checkDimension(data, index, "Roll", roll);
    checkDimension(data, index, "Pitch", pitch);
    checkDimension(data, index, "PlatformHeading", heading);
    checkDimension(data, index, "WanderAngle", wander);
    checkDimension(data, index, "XBodyAccel", xaccel);
    checkDimension(data, index, "YBodyAccel", yaccel);
    checkDimension(data, index, "ZBodyAccel", zaccel);
    checkDimension(data, index, "XBodyAngRate", xangrate);
    checkDimension(data, index, "YBodyAngRate", yangrate);
    checkDimension(data, index, "ZBodyAngRate", zangrate);
}


BOOST_AUTO_TEST_SUITE(SbetReaderTest)


BOOST_AUTO_TEST_CASE(testConstructor)
{
    pdal::Options options;
    pdal::Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    options.add(filename);

    pdal::drivers::sbet::Reader reader(options);
    BOOST_CHECK(reader.getDescription() == "SBET Reader");
    BOOST_CHECK_EQUAL(reader.getName(), "drivers.sbet.reader");

    reader.initialize();

    BOOST_CHECK_EQUAL(reader.getPointCountType(), pdal::PointCount_Fixed);
    BOOST_CHECK_EQUAL(reader.getNumPoints(), 2);

}


BOOST_AUTO_TEST_CASE(testRead)
{
    pdal::Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    pdal::Options options(filename);
    pdal::drivers::sbet::Reader reader(options);
    reader.initialize();

    const pdal::Schema& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 2);

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator(data);
    BOOST_CHECK(!iter->atEnd());

    boost::uint32_t numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 2);
    BOOST_CHECK(iter->atEnd());

    BOOST_CHECK_THROW(iter->read(data), pdal::pdal_error);

    delete iter;

    checkPoint(data, 0, 1.516310028360710e+05, 5.680211852972264e-01,
               -2.041654392303940e+00, 1.077152953296560e+02,
               -2.332420866600025e+00, -3.335067504871401e-01,
               -3.093961631767838e-02, -2.813407149321339e-02,
               -2.429905393889139e-02, 3.046773230278662e+00,
               -2.198414736922658e-02, 7.859639737752390e-01,
               7.849084719295495e-01, -2.978807916450262e-01,
               6.226807982589819e-05, 9.312162756440178e-03,
               7.217812320996525e-02);
    checkPoint(data, 1, 1.516310078318641e+05, 5.680211834722869e-01,
               -2.041654392034053e+00, 1.077151424357507e+02,
               -2.336228229691271e+00, -3.324663118952635e-01,
               -3.022948961008987e-02, -2.813856631423094e-02,
               -2.425215669392169e-02, 3.047131105236811e+00,
               -2.198416007932108e-02, 8.397590491636475e-01,
               3.252165276637165e-01, -1.558883225990844e-01,
               8.379685112283802e-04, 7.372886784718076e-03,
               7.179027672314571e-02);

    return;
}


BOOST_AUTO_TEST_CASE(testSkip)
{
    pdal::Option filename("filename", Support::datapath("sbet/2-points.sbet"), "");
    pdal::Options options(filename);
    pdal::drivers::sbet::Reader reader(options);
    reader.initialize();

    const pdal::Schema& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 1);

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator(data);
    iter->skip(1);
    iter->read(data);

    delete iter;

    checkPoint(data, 0, 1.516310078318641e+05, 5.680211834722869e-01,
               -2.041654392034053e+00, 1.077151424357507e+02,
               -2.336228229691271e+00, -3.324663118952635e-01,
               -3.022948961008987e-02, -2.813856631423094e-02,
               -2.425215669392169e-02, 3.047131105236811e+00,
               -2.198416007932108e-02, 8.397590491636475e-01,
               3.252165276637165e-01, -1.558883225990844e-01,
               8.379685112283802e-04, 7.372886784718076e-03,
               7.179027672314571e-02);

    return;
}


BOOST_AUTO_TEST_CASE(testBadFile)
{
    pdal::Option filename("filename", Support::datapath("sbet/badfile.sbet"), "");
    pdal::Options options(filename);
    pdal::drivers::sbet::Reader reader(options);
    BOOST_CHECK_THROW(reader.initialize(), pdal::pdal_error);
}


BOOST_AUTO_TEST_CASE(testPipeline)
{
    pdal::PipelineManager manager;
    pdal::PipelineReader reader(manager);

    bool isWriter = reader.readPipeline(Support::datapath("sbet/pipeline.xml"));
    BOOST_CHECK(isWriter);

    const boost::uint64_t numPoints = manager.execute();
    BOOST_CHECK_EQUAL(numPoints, 2);
    pdal::FileUtils::deleteFile(Support::datapath("sbet/outfile.txt"));

    return;
}


BOOST_AUTO_TEST_SUITE_END()
