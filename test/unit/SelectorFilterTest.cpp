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

BOOST_AUTO_TEST_CASE(SelectorFilterTest_test_1)
{
    pdal::Option option("filename", Support::datapath("pipeline/pipeline_scaling.xml"));
    pdal::Options options(option);

    pdal::drivers::pipeline::Reader reader(options);
    reader.initialize();
    
    // pdal::filters::Scaling const* filter = static_cast<pdal::filters::Scaling const*>(reader.getManager().getStage());
    // pdal::Options opt = filter->getCurrentOptions();
    // // std::cout << "filter ops: " << opt << std::endl;
    // 
    // const pdal::Schema& schema = filter->getSchema();
    // pdal::PointBuffer data(schema, 1);
    // 
    // pdal::StageSequentialIterator* iter = filter->createSequentialIterator(data);
    // 
    // boost::uint32_t numRead = iter->read(data);
    // BOOST_CHECK(numRead == 1);
    // delete iter;
    // 
    // pdal::Schema const& schema2 = data.getSchema();
    // 
    // boost::optional<pdal::Dimension const&> scaledDimX = schema2.getDimension("X", "filters.scaling");
    // boost::optional<pdal::Dimension const&> scaledDimY = schema2.getDimension("Y", "filters.scaling");
    // 
    // if (!scaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    // boost::int32_t x = data.getField<boost::int32_t>(*scaledDimX, 0);
    // boost::int32_t y = data.getField<boost::int32_t>(*scaledDimY, 0);
    // 
    // BOOST_CHECK_EQUAL(x, 6370112);
    // BOOST_CHECK_EQUAL(y, 8490263);
    // 
    // boost::optional<pdal::Dimension const&> unscaledDimX = schema2.getDimension("X", "drivers.las.reader");
    // boost::optional<pdal::Dimension const&> unscaledDimY = schema2.getDimension("Y", "drivers.las.reader");
    // 
    // if (!unscaledDimX) throw pdal::pdal_error("Hey, no dimension was selected");
    // x = data.getField<boost::int32_t>(*unscaledDimX, 0);
    // y = data.getField<boost::int32_t>(*unscaledDimY, 0);
    // 
    // BOOST_CHECK_EQUAL(x, 63701224);
    // BOOST_CHECK_EQUAL(y, 84902831);

    return;
}



BOOST_AUTO_TEST_SUITE_END()
