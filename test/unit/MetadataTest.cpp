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
#include <boost/algorithm/string.hpp>
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

using namespace pdal;

BOOST_AUTO_TEST_CASE(test_construction)
{
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
    MetadataNode m;
    MetadataNode m2 = m.add("val", 2u);
    uint32_t t = boost::lexical_cast<uint32_t>(m2.value());
    BOOST_CHECK_EQUAL(t, 2u);
}

BOOST_AUTO_TEST_CASE(test_metadata_set)
{
    MetadataNode m;

    MetadataNode m1 = m.add("m1", 1u);
    MetadataNode m2 = m.add("m2", 2);
    MetadataNode m1prime = m.add("m1prime", "Some other metadata");

    MetadataNode mm(m);

    std::vector<MetadataNode> ms = mm.children();
    BOOST_CHECK_EQUAL(ms.size(), 3);

    class Predicate
    {
    public:
        Predicate(const std::string& name) : m_name(name)
        {}

        bool operator()(MetadataNode m)
            { return m.name() == m_name; }
    private:
        std::string m_name;
    };

    MetadataNode node = mm.findChild(Predicate("m1"));
    BOOST_CHECK_EQUAL(node.value(), "1");
    node = mm.find(Predicate("m2"));
    BOOST_CHECK_EQUAL(node.value(), "2");
    node = mm.find(Predicate("m1prime"));
    BOOST_CHECK_EQUAL(node.value(), "Some other metadata");
    node = mm.find(Predicate("foo"));
    BOOST_CHECK_EQUAL(node.value(), "");
}

BOOST_AUTO_TEST_CASE(test_vlr_metadata)
{
    using namespace pdal;

    MetadataNode m;

    MetadataNode bogusvlr = m.add("vlr1", "VLR1VALUE", "VLR1DESC");
    MetadataNode vlr = m.add("vlr2", "VLR2VALUE", "VLR2DESC");
    std::string recordId("MYRECOREDID");
    std::string userId("MYUSERID");
    vlr.add("record_id", recordId);
    vlr.add("user_id", userId);
    // Find a node whose name starts with vlr and that has child nodes
    // with the name and recordId we're looking for.
    auto pred = [recordId,userId](MetadataNode n)
    {
        auto recPred = [recordId](MetadataNode n)
        {
            return n.name() == "record_id" &&
                n.value() == recordId;
        };
        auto userPred = [userId](MetadataNode n)
        {
            return n.name() == "user_id" &&
                n.value() == userId;
        };
        return (boost::algorithm::istarts_with(n.name(), "vlr") &&
            !n.findChild(recPred).empty() &&
            !n.findChild(userPred).empty());
    };

    MetadataNode found = m.find(pred);
    BOOST_CHECK_EQUAL(found.name(), "vlr2");
    BOOST_CHECK_EQUAL(found.value(), "VLR2VALUE");
    BOOST_CHECK_EQUAL(found.description(), "VLR2DESC");
}

BOOST_AUTO_TEST_CASE(find_child_string)
{
    MetadataNode top;
    MetadataNode m = top.add("level1");
    MetadataNode l21 = m.add("level2");
    MetadataNode l22 = m.add("level2");
    l21.add("210", "210");
    l22.add("220", "220");

    MetadataNode n = top.findChild("level1:level2:210");
    BOOST_CHECK_EQUAL(n.value(), "210");
    n = top.findChild("level1:level2:220");
    BOOST_CHECK_EQUAL(n.value(), "220");
}

BOOST_AUTO_TEST_CASE(toJSON)
{
/**
    MetadataNode top("stats");
    top.add("test", 1);
    MetadataNode l2 = top.add("test", 2);
    top.add("test", "3");
    top.add("test2", "foo");

    l2.add("foo", 3.56);
    l2.add("bar", "ballet");
    l2.add("foo", 25);
    std::string s = top.toJSON();
    std::cerr << s << "---\n";
**/
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
