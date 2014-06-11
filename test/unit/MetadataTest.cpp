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

#include <boost/test/unit_test.hpp>
#include <sstream>
#include <iostream>
#include <string>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include "Support.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>


BOOST_AUTO_TEST_SUITE(MetadataTest)


BOOST_AUTO_TEST_CASE(test_construction)
{
    using namespace pdal;

    Metadata metadata;
    MetadataNode m = metadata.getNode();

    uint32_t u32(32u);
    int32_t i32(-32);
    uint64_t u64(64u);
    int64_t i64(-64);
    int8_t i8(-8);
    uint8_t u8(8);
    int16_t i16(-16);
    uint16_t u16(16);
    
    std::vector<uint8_t> v;
    for (uint8_t i = 0; i < 100; i++)
        v.push_back(i);

    ByteArray bytes(v);

    Bounds<double> b(1.1, 2.2, 3.3, 101.1, 102.2, 103.3);

    m.setValue(32);

    BOOST_CHECK_EQUAL(m.getValue<uint32_t>(), 32);
    BOOST_CHECK_EQUAL(m.getType(), "nonNegativeInteger");

    BOOST_CHECK_EQUAL(m.getValue<int32_t>(), 32);
    m.setValue(bytes);
    BOOST_CHECK_EQUAL(m.getType(), "base64Binary");

    std::string base64("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiYw==");
    BOOST_CHECK_EQUAL(boost::lexical_cast<std::string>(
        m.getValue<ByteArray>()), base64); 

    m.setValue(i8);
    BOOST_CHECK_EQUAL(m.getValue<int8_t>(), -8);
    BOOST_CHECK_EQUAL(m.getType(), "integer");

    m.setValue(i16);
    BOOST_CHECK_EQUAL(m.getValue<int16_t>(), -16);
    BOOST_CHECK_EQUAL(m.getType(), "integer");

    m.setValue(i32);
    BOOST_CHECK_EQUAL(m.getValue<int32_t>(), -32);
    BOOST_CHECK_EQUAL(m.getType(), "integer");

    m.setValue(i64);
    BOOST_CHECK_EQUAL(m.getValue<int64_t>(), -64);
    BOOST_CHECK_EQUAL(m.getType(), "integer");

    m.setValue(u8);
    BOOST_CHECK_EQUAL(m.getValue<uint8_t>(), 8);
    BOOST_CHECK_EQUAL(m.getType(), "nonNegativeInteger");

    m.setValue(u16);
    BOOST_CHECK_EQUAL(m.getValue<uint16_t>(), 16);
    BOOST_CHECK_EQUAL(m.getType(), "nonNegativeInteger");

    m.setValue(u32);
    BOOST_CHECK_EQUAL(m.getValue<uint32_t>(), 32);
    BOOST_CHECK_EQUAL(m.getType(), "nonNegativeInteger");

    m.setValue(u64);
    BOOST_CHECK_EQUAL(m.getValue<uint64_t>(), 64);
    BOOST_CHECK_EQUAL(m.getType(), "nonNegativeInteger");
}


#ifdef PDAL_SRS_ENABLED
BOOST_AUTO_TEST_CASE(test_construction_with_srs)
{
    using namespace pdal;

    Metadata metadata;
    MetadataNode m = metadata.getNode();
    SpatialReference ref("EPSG:4326");
    m.setValue(ref);
    BOOST_CHECK_EQUAL(m.getType(), "spatialreference");

    pdal::SpatialReference ref2 = m.getValue<pdal::SpatialReference>();
    // std::string ref_text("GEOGCS[\"WGS 84\","
    //     DATUM[\"WGS_1984\","
    //         SPHEROID[\"WGS 84\",6378137,298.257223563,
    //             AUTHORITY[\"EPSG\",\"7030\"]],
    //         AUTHORITY[\"EPSG\",\"6326\"]],
    //     PRIMEM[\"Greenwich\",0,
    //         AUTHORITY[\"EPSG\",\"8901\"]],
    //     UNIT[\"degree\",0.0174532925199433,
    //         AUTHORITY[\"EPSG\",\"9122\"]],
    //     AUTHORITY[\"EPSG\",\"4326\"]]");

    // std::cout << boost::lexical_cast<std::string>(m.getValue<pdal::SpatialReference>());
}
#endif


BOOST_AUTO_TEST_CASE(test_metadata_copy)
{
    using namespace pdal;

    Metadata metadata1;
    Metadata metadata2;
    Metadata metadata1prime;

    MetadataNode m1 = metadata1.getNode();
    MetadataNode m2 = metadata2.getNode();
    MetadataNode m1prime = metadata1prime.getNode();

    m1.setValue(2u);
    m2.setValue(1);
    m1prime.setValue(std::string("Some other metadata"));
    uint32_t t = m1.getValue<uint32_t>();
    BOOST_CHECK_EQUAL(t, 2u);
}

BOOST_AUTO_TEST_CASE(test_metadata_set)
{
    using namespace pdal;

    Metadata metadata1;
    Metadata metadata2;
    Metadata metadata1prime;

    MetadataNode m1 = metadata1.getNode();
    MetadataNode m2 = metadata2.getNode();
    MetadataNode m1prime = metadata1prime.getNode();

    m1.setValue(1u);
    m2.setValue(1);
    m1prime.setValue(std::string("Some other metadata"));

    Metadata metadatab;
    MetadataNode b = metadatab.getNode();

//    //ABELL
//    b.add(m1.getName(), m1);

    Metadata metadata3(metadata1);
    MetadataNode m3 = metadata3.getNode();
    BOOST_CHECK_EQUAL(m3.getValue<uint32_t>(), 1u);
    m3.setValue(64);
    BOOST_CHECK_EQUAL(m3.getValue<int64_t>(), 64);
}

BOOST_AUTO_TEST_CASE(test_metadata_stage)
{
    using namespace pdal;

    PointContext ctx;

    drivers::las::Reader reader(Support::datapath("interesting.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.prepare(ctx);

    MetadataNode file_metadata = ctx.metadata();

    BOOST_CHECK_EQUAL(file_metadata.toPTree().get_child("metadata").size(),
        32);

    PointContext readerCtx;
    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(
        Support::datapath("pipeline/pipeline_metadata_reader.xml"));
    Stage *stage = mgr.getStage();

    stage->prepare(readerCtx);
    MetadataNode pipeline_metadata = readerCtx.metadata();
    BOOST_CHECK_EQUAL(
        pipeline_metadata.toPTree().get_child("metadata").size(), 32);
}


BOOST_AUTO_TEST_SUITE_END()
