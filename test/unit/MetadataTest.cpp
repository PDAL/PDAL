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

#include <pdal/Metadata.hpp>

#include <boost/property_tree/xml_parser.hpp>


BOOST_AUTO_TEST_SUITE(MetadataTest)


BOOST_AUTO_TEST_CASE(test_construction)
{
    
    pdal::Metadata m("test", "testNS");
    
    BOOST_CHECK_EQUAL(m.getName(), "test");
    BOOST_CHECK_EQUAL(m.getNamespace(), "testNS");
    
    boost::uint32_t u32(32u);
    boost::int32_t i32(-32);
    boost::uint64_t u64(64u);
    boost::int64_t i64(-64);
    boost::int8_t i8(-8);
    boost::uint8_t u8(8);
    boost::int16_t i16(-16);
    boost::uint16_t u16(16);
    
    std::vector<boost::uint8_t> v;
    for(boost::uint8_t i=0; i < 100; i++) v.push_back(i);
    
    pdal::ByteArray bytes(v);
        
    pdal::Bounds<double> b(1.1,2.2,3.3,101.1,102.2,103.3);
    
    m.setValue<boost::uint32_t>(u32);
    
    m.addAttribute("some_id", "some_value");
    m.addAttribute("another_id", "another_value");
    
    std::vector<std::string> names = m.getAttributeNames();
    BOOST_CHECK_EQUAL(names.size(), 2u);
    
    BOOST_CHECK_EQUAL(m.getAttribute("some_id"), "some_value");
    BOOST_CHECK_EQUAL(m.getAttribute("another_id"), "another_value");
    
    BOOST_CHECK_EQUAL(m.getValue<boost::uint32_t>(), 32u);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::UnsignedInteger);
    
    BOOST_CHECK_THROW(m.getValue<boost::int32_t>(), boost::bad_get);

    BOOST_CHECK_EQUAL(m.cast<boost::int32_t>(), 32); 
    m.setValue<pdal::ByteArray>(bytes);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::Bytes);
    
    std::string base64("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiYw==");
    BOOST_CHECK_EQUAL( boost::lexical_cast<std::string>(m.getValue<pdal::ByteArray>()), base64);
    BOOST_CHECK_THROW(m.getValue<boost::int32_t>(), boost::bad_get);
    
    pdal::SpatialReference ref("EPSG:4326");
    m.setValue<pdal::SpatialReference>(ref);    
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::SpatialReference);
    
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
    m.setValue<boost::int8_t>(i8);
    BOOST_CHECK_EQUAL(m.getValue<boost::int8_t>(), -8);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::SignedInteger);

    m.setValue<boost::int16_t>(i16);
    BOOST_CHECK_EQUAL(m.getValue<boost::int16_t>(), -16);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::SignedInteger);
    
    m.setValue<boost::int32_t>(i32);
    BOOST_CHECK_EQUAL(m.getValue<boost::int32_t>(), -32);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::SignedInteger);

    m.setValue<boost::int64_t>(i64);
    BOOST_CHECK_EQUAL(m.getValue<boost::int64_t>(), -64);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::SignedInteger);

    m.setValue<boost::uint8_t>(u8);
    BOOST_CHECK_EQUAL(m.getValue<boost::uint8_t>(), 8u);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::UnsignedInteger);

    m.setValue<boost::uint16_t>(u16);
    BOOST_CHECK_EQUAL(m.getValue<boost::uint16_t>(), 16u);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::UnsignedInteger);
    
    m.setValue<boost::uint32_t>(u32);
    BOOST_CHECK_EQUAL(m.getValue<boost::uint32_t>(), 32u);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::UnsignedInteger);

    m.setValue<boost::uint64_t>(u64);
    BOOST_CHECK_EQUAL(m.getValue<boost::uint64_t>(), 64u);
    BOOST_CHECK_EQUAL(m.getType(), pdal::metadata::UnsignedInteger);

    return;
}


BOOST_AUTO_TEST_CASE(test_parent_child)
{
    pdal::Metadata m1("m1","parent");
    pdal::Metadata m2("m2", "");
    pdal::Metadata m1prime("m1", "child");
    
    m1.createUUID();
    m1prime.createUUID();
    m1prime.setParent(m1.getUUID());
    m1.setValue<boost::uint32_t>(1u);
    m2.setValue<boost::int32_t>(1);
    m1prime.setValue<std::string>("Some other metadata");
    
    pdal::Metadatas b;
    
    b.addMetadata(m1);
    b.addMetadata(m2);
    b.addMetadata(m1prime);
    
    
    pdal::Metadata m11 = b.getMetadata("m1", "parent");
    BOOST_CHECK_EQUAL(m11.getValue<boost::uint32_t>(), 1u);
    
    pdal::Metadata m11prime = b.getMetadata("m1");
    BOOST_CHECK_EQUAL(m11prime.getValue<std::string>(), "Some other metadata");
    
    pdal::Metadata m22 = b.getMetadata("m2");
    BOOST_CHECK_EQUAL(m22.cast<boost::uint32_t>(), 1u);
    BOOST_CHECK_THROW(m22.getValue<boost::uint32_t>(), boost::bad_get);
    
    return;
}

BOOST_AUTO_TEST_CASE(test_metadata_copy)
{
    pdal::Metadata m1("m1","parent");
    pdal::Metadata m2("m2", "");
    pdal::Metadata m1prime("m1", "child");
    
    m1.createUUID();
    m1prime.createUUID();
    m1prime.setParent(m1.getUUID());
    m1.setValue<boost::uint32_t>(1u);
    m2.setValue<boost::int32_t>(1);
    m1prime.setValue<std::string>("Some other metadata");
    
    pdal::Metadatas b;
    
    b.addMetadata(m1);
    b.addMetadata(m1prime);
    b.addMetadata(m2);
    
    pdal::Metadatas b2;

    b2.setMetadata(b.getMetadata());

    pdal::Metadata m11 = b2.getMetadata("m1", "parent");
    BOOST_CHECK_EQUAL(m11.getValue<boost::uint32_t>(), 1u);
    
    pdal::Metadata m11prime = b2.getMetadata("m1");
    BOOST_CHECK_EQUAL(m11prime.getValue<std::string>(), "Some other metadata");
    
    pdal::Metadata m22 = b2.getMetadata("m2");
    BOOST_CHECK_EQUAL(m22.cast<boost::uint32_t>(), 1u);
    BOOST_CHECK_THROW(m22.getValue<boost::uint32_t>(), boost::bad_get);
    
    return;
}

BOOST_AUTO_TEST_CASE(test_metadata_set)
{
    pdal::Metadata m1("m1","parent");
    pdal::Metadata m2("m2", "");
    pdal::Metadata m1prime("m1", "child");
    
    m1.createUUID();
    m1prime.createUUID();
    m1prime.setParent(m1.getUUID());
    m1.setValue<boost::uint32_t>(1u);
    m2.setValue<boost::int32_t>(1);
    m1prime.setValue<std::string>("Some other metadata");
    
    pdal::Metadatas b;
    
    b.addMetadata(m1);

    pdal::Metadata m3(m1);
    BOOST_CHECK_EQUAL(m3.getValue<boost::uint32_t>(), 1u);
    m3.setValue<boost::int64_t>(64);
    BOOST_CHECK_EQUAL(m3.getValue<boost::int64_t>(), 64);
    
    b.setMetadata(m3);
    
    return;
}

BOOST_AUTO_TEST_SUITE_END()
