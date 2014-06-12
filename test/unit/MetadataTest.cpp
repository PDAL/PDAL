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

    uint32_t u32(32u);
    int32_t i32(-32);
    uint64_t u64(64u);
    int64_t i64(-64);
    int8_t i8(-8);
    uint8_t u8(8);
    int16_t i16(-16);
    uint16_t u16(16);
    
    {
        std::vector<uint8_t> v;
        for (uint8_t i = 0; i < 100; i++)
            v.push_back(i);

        ByteArray bytes(v);

        MetadataNode m;
        MetadataNode m2 = m.add("name", bytes);
        BOOST_CHECK_EQUAL(m2.type(), "base64Binary");

        std::string base64("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiYw==");
        BOOST_CHECK_EQUAL(m2.value(), base64);
    }


    {
        MetadataNode m;
        MetadataNode m2 = m.add<int8_t>("name", i8);
        BOOST_CHECK_EQUAL(m2.value(), "-8");
        BOOST_CHECK_EQUAL(m2.type(), "integer");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", i16);
        BOOST_CHECK_EQUAL(m2.value(), "-16");
        BOOST_CHECK_EQUAL(m2.type(), "integer");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", i32);
        BOOST_CHECK_EQUAL(m2.value(), "-32");
        BOOST_CHECK_EQUAL(m2.type(), "integer");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", i64);
        BOOST_CHECK_EQUAL(m2.value(), "-64");
        BOOST_CHECK_EQUAL(m2.type(), "integer");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", i64);
        BOOST_CHECK_EQUAL(m2.value(), "-64");
        BOOST_CHECK_EQUAL(m2.type(), "integer");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add<uint8_t>("name", u8);
        BOOST_CHECK_EQUAL(m2.value(), "8");
        BOOST_CHECK_EQUAL(m2.type(), "nonNegativeInteger");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", u16);
        BOOST_CHECK_EQUAL(m2.value(), "16");
        BOOST_CHECK_EQUAL(m2.type(), "nonNegativeInteger");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", u32);
        BOOST_CHECK_EQUAL(m2.value(), "32");
        BOOST_CHECK_EQUAL(m2.type(), "nonNegativeInteger");
    }

    {
        MetadataNode m;
        MetadataNode m2 = m.add("name", u64);
        BOOST_CHECK_EQUAL(m2.value(), "64");
        BOOST_CHECK_EQUAL(m2.type(), "nonNegativeInteger");
    }
}


#ifdef PDAL_SRS_ENABLED
BOOST_AUTO_TEST_CASE(test_construction_with_srs)
{
    using namespace pdal;

    MetadataNode m;
    SpatialReference ref("EPSG:4326");
    MetadataNode m2 = m.add("spatialreference", ref);
    BOOST_CHECK_EQUAL(m2.type(), "spatialreference");

    //pdal::SpatialReference ref2 = m.getValue<pdal::SpatialReference>();
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

    MetadataNode m;
    MetadataNode m2 = m.add("val", 2u);
    uint32_t t = boost::lexical_cast<uint32_t>(m2.value());
    BOOST_CHECK_EQUAL(t, 2u);
}

BOOST_AUTO_TEST_CASE(test_metadata_set)
{
    using namespace pdal;

    MetadataNode m;

    MetadataNode m1 = m.add("m1", 1u);
    MetadataNode m2 = m.add("m2", 2);
    MetadataNode m1prime = m.add("m1prime", "Some other metadata");

    MetadataNode mm(m);

    std::vector<MetadataNode> ms = mm.children();
    BOOST_CHECK_EQUAL(ms.size(), 3);

    auto pred = [](MetadataNode m)
        { return m.name() == "m1"; };
    MetadataNode node = mm.findChild(pred);
    BOOST_CHECK_EQUAL(node.value(), "1");
    auto pred2 = [](MetadataNode m)
        { return m.name() == "m2"; };
    node = mm.find(pred2);
    BOOST_CHECK_EQUAL(node.value(), "2");
    auto pred3 = [](MetadataNode m)
        { return m.name() == "m1prime"; };
    node = mm.find(pred3);
    BOOST_CHECK_EQUAL(node.value(), "Some other metadata");
}

BOOST_AUTO_TEST_CASE(test_metadata_stage)
{
//ABELL
/**
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
**/
}


BOOST_AUTO_TEST_SUITE_END()
