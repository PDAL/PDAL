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
    
    pdal::Metadata m("test", "testNS", pdal::metadata::SignedInteger);
    
    boost::uint32_t u32(32u);
    boost::int32_t i32(-32);
    boost::uint64_t u64(64u);
    boost::int64_t i64(-64);
    boost::int8_t i8(-8);
    boost::uint8_t u8(8);
    boost::int16_t i16(-16);
    boost::uint16_t u16(16);
    
    std::vector<boost::uint8_t> v;
    for(int i=0; i < 100; i++) v.push_back(i);
    
    pdal::metadata::ByteArray bytes(v);
    
    pdal::SpatialReference ref("EPSG:4326");
    
    pdal::Bounds<double> b(1.1,2.2,3.3,101.1,102.2,103.3);
    
    m.setValue<boost::uint32_t>(u32);
    
    m.addAttribute("some_id", "some_value");
    m.addAttribute("another_id", "another_value");
    
    std::vector<std::string> names = m.getAttributeNames();
    BOOST_CHECK_EQUAL(names.size(), 2);
    
    BOOST_CHECK_EQUAL(m.getAttribute("some_id"), "some_value");
    BOOST_CHECK_EQUAL(m.getAttribute("another_id"), "another_value");
    
    BOOST_CHECK_EQUAL(m.getValue<boost::uint32_t>(), 32u);
    BOOST_CHECK_THROW(m.getValue<boost::int32_t>(), boost::bad_get);

    BOOST_CHECK_EQUAL(m.cast<boost::int32_t>(), 32); 
    m.setValue<pdal::metadata::ByteArray>(bytes);
    
    std::string base64("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiYw==");
    BOOST_CHECK_EQUAL( boost::lexical_cast<std::string>(m.getValue<pdal::metadata::ByteArray>()), base64);
    BOOST_CHECK_THROW(m.getValue<boost::int32_t>(), boost::bad_get);
    
    return;
}




BOOST_AUTO_TEST_SUITE_END()
