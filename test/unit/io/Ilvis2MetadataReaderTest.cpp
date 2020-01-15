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

#include <io/Ilvis2MetadataReader.hpp>
#include "Support.hpp"

#include <iostream>
#include <fstream>

using namespace pdal;


TEST(Ilvis2MetadataReaderTest, testReadMetadata)
{
    Ilvis2MetadataReader reader;
    MetadataNode n;
    MetadataNodeList l,l1,l2,l3;
    std::ofstream outfile;

    auto m = std::unique_ptr<MetadataNode>(new MetadataNode());
    reader.readMetadataFile(Support::datapath("ilvis2/ILVIS2_TEST_FILE.TXT.xml"), m.get());
    
    n = m->children("GranuleUR")[0];
    EXPECT_EQ("SC:ILVIS2.001:51203496", n.value());

    n = m->children("DbID")[0];
    EXPECT_EQ(51203496L, n.value<long>());

    l = m->children("DataFile");
    EXPECT_EQ(std::size_t{2}, l.size());
    EXPECT_EQ("SHA1", l[1].children("ChecksumType")[0].value());

    l = m->children("Campaign");
    EXPECT_EQ(std::size_t{2}, l.size());

    l = m->children("PSA");
    EXPECT_EQ(std::size_t{3}, l.size());
    EXPECT_EQ("SIPSMetGenVersion", l[0].children("PSAName")[0].value());
    EXPECT_EQ("N426NA", l[2].children("PSAValue")[0].value());

    l = m->children("BrowseProductGranuleId");
    EXPECT_EQ(std::size_t{2}, l.size());

    l = m->children("PHProductGranuleId");
    EXPECT_EQ(std::size_t{1}, l.size());
    EXPECT_EQ("PH_ID", l[0].value());

    l = m->children("Platform");
    EXPECT_EQ(std::size_t{1}, l.size());
    l1 = l[0].children("Instrument");
    EXPECT_EQ(std::size_t{1}, l1.size());
    EXPECT_EQ(std::size_t{2}, l1[0].children("OperationMode").size());
    EXPECT_EQ("Safe", l1[0].children("OperationMode")[1].value());
    l2 = l1[0].children("Sensor");
    EXPECT_EQ(std::size_t{1}, l2.size());
    l3 = l2[0].children("SensorCharacteristic");
    EXPECT_EQ(std::size_t{2}, l3.size());
    EXPECT_EQ("CharName1", l3[0].children("CharacteristicName")[0].value());
    EXPECT_EQ("MyValue", l3[1].children("CharacteristicValue")[0].value());

    l = m->children("ConvexHull");
    EXPECT_EQ(std::size_t{1}, l.size());
    EXPECT_EQ(std::size_t{0}, l[0].value().find("POLYGON"));
}
