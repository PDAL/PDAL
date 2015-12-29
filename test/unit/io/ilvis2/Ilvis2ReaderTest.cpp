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

TEST(Ilvis2ReaderTest, testReadDefault)
{
    Option filename("filename",
        Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT"));
    Options options(filename);
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;

    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 4u);

    checkPoint(*view.get(), 0, 42504.48313,
             78.307672,-58.785213,1956.777
            );

    checkPoint(*view.get(), 1, 42504.48512,
             78.307592, 101.215097, 1956.588
            );

    checkPoint(*view.get(), 2, 42504.48712,
             78.307512, -58.78459, 1956.667
            );

    checkPoint(*view.get(), 3, 42504.48712,
             78.307512, -58.78459, 2956.667
            );
}


TEST(Ilvis2ReaderTest, testReadHigh)
{
    Option filename("filename",
        Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT"));
    Options options(filename);
    options.add("mapping","high");
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;

    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 3u);

    checkPoint(*view.get(), 0, 42504.48313,
             78.307672,-58.785213,1956.777
            );

    checkPoint(*view.get(), 1, 42504.48512,
             78.307592, 101.215097, 1956.588
            );

    checkPoint(*view.get(), 2, 42504.48712,
             78.307512, -58.78459, 2956.667
            );
}


TEST(Ilvis2ReaderTest, testInvalidMetadataFile)
{
    Option filename("filename",
        Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT"));
    Options options(filename);
    options.add("metadata", "invalidfile");
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;
    try
    {
        reader->prepare(table);
        reader->execute(table);
        FAIL() << "Expected an exception for an invalid file";
    }
    catch (pdal_error const & err)
    {
        EXPECT_EQ("Invalid metadata file: 'invalidfile'", std::string(err.what()));
    }
}


TEST(Ilvis2ReaderTest, testValidMetadataFile)
{
    Option filename("filename",
        Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT"));
    Options options(filename);
    options.add("metadata", Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT.xml"));
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    reader->execute(table);

    MetadataNode m, n;
    MetadataNodeList l;
    m = reader->getMetadata();

    n = m.children("GranuleUR")[0];
    EXPECT_EQ("SC:ILVIS2.001:51203496", n.value());

    l = m.children("DataFile");
    EXPECT_EQ(std::size_t{2}, l.size());
    EXPECT_EQ("SHA1", l[1].children("ChecksumType")[0].value());

    l = m.children("Platform")[0].children("Instrument")[0].children("Sensor")[0].children("SensorCharacteristic");
    EXPECT_EQ(std::size_t{2}, l.size());
    EXPECT_EQ("CharName1", l[0].children("CharacteristicName")[0].value());
    EXPECT_EQ("MyValue", l[1].children("CharacteristicValue")[0].value());
}


TEST(Ilvis2ReaderTest, testNoMetadataFile)
{
    Option filename("filename",
        Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT"));
    Options options(filename);
    std::shared_ptr<Ilvis2Reader> reader(new Ilvis2Reader);
    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    reader->execute(table);

    MetadataNode m;
    MetadataNodeList l;
    m = reader->getMetadata();

    l = m.children("GranuleUR");
    EXPECT_EQ(std::size_t{0}, l.size());

    l = m.children("DataFile");
    EXPECT_EQ(std::size_t{0}, l.size());

    l = m.children("Platform");
    EXPECT_EQ(std::size_t{0}, l.size());

    l = m.children("ConvexHull");
    EXPECT_EQ(std::size_t{0}, l.size());
}
