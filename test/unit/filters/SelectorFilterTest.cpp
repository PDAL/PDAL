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

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/filters/Selector.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(SelectorFilterTest)

BOOST_AUTO_TEST_CASE(test1)
{
    pdal::Option option("filename", Support::datapath("filters/selector.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();
    
    pdal::Schema const& schema = reader.getSchema();
    pdal::PointBuffer data(schema, 1000);

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(reader.createSequentialIterator(data));
    
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1000u);
    }
    
    
    pdal::Schema const& new_schema = data.getSchema();
    
    // std::cout << new_schema << std::endl;
    
    BOOST_CHECK_EQUAL(new_schema.getDimension("Red").isIgnored(), true);
    BOOST_CHECK_EQUAL(new_schema.getDimension("Green").isIgnored(), true);
    BOOST_CHECK_EQUAL(new_schema.getDimension("Blue").isIgnored(), true);
    
    // ignore by default is true because not set on the pipeline
    BOOST_CHECK_EQUAL(new_schema.getDimension("PointSourceId").isIgnored(), true); 

    // We explicitly kept X
    BOOST_CHECK_EQUAL(new_schema.getDimension("X").isIgnored(), false);

    // We created Greenish
    BOOST_CHECK_EQUAL(new_schema.getDimension("Greenish").isIgnored(), false);

    return;
}




BOOST_AUTO_TEST_SUITE_END()
