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
#include <boost/cstdint.hpp>

#include <pdal/filters/Chipper.hpp>
#include <pdal/drivers/liblas/Writer.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

using namespace pdal;
using namespace pdal::drivers::liblas;

BOOST_AUTO_TEST_SUITE(ChipperTest)


BOOST_AUTO_TEST_CASE(test_construction)
{
    LiblasReader reader(Support::datapath("1.2-with-color.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream
        
        pdal::Options options;
        pdal::Option<boost::uint32_t> capacity("capacity", 15, "capacity");
        options.add(capacity);
        
        pdal::filters::Chipper chipper(reader, options);
        chipper.initialize();

        const boost::uint64_t num_points = reader.getNumPoints();

        chipper.Chip();
        boost::uint32_t num_blocks = chipper.GetBlockCount();
        BOOST_CHECK(num_points == 1065);
        BOOST_CHECK(num_blocks==71);
        
        
        pdal::Bounds<double> const& bounds = chipper.GetBlock(0).GetBounds();

        
        std::vector< Range<double> > ranges = bounds.dimensions();
        
        pdal::Range<double> x = ranges[0];
        pdal::Range<double> y = ranges[1];
        
        BOOST_CHECK_CLOSE (x.getMinimum(), 635674.0, 0.05);
        BOOST_CHECK_CLOSE (x.getMaximum(), 635994.0, 0.05);
        BOOST_CHECK_CLOSE (y.getMinimum(), 848992.0, 0.05);
        BOOST_CHECK_CLOSE (y.getMaximum(), 849427.0, 0.05);

        std::vector<boost::uint32_t> ids = chipper.GetBlock(70).GetIDs();

        BOOST_CHECK(ids.size() == 15);
        BOOST_CHECK(ids[14] == 1050 );
        
        pdal::Schema const& schema = reader.getSchema();
        PointBuffer buffer(schema, 15);
        const int indexId = schema.getDimensionIndex(Dimension::Field_User1, Dimension::Int32);
        const int indexBlockId = schema.getDimensionIndex(Dimension::Field_User2, Dimension::Int32);        
        chipper.GetBlock(20).GetBuffer(reader, buffer, 70, indexId, indexBlockId);

        // 
        // std::cout << buffer.getField<boost::int32_t>(0, 0) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 0) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 0) << std::endl;
        // 
        // std::cout << buffer.getField<boost::int32_t>(0, 1) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 1) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 1) << std::endl;
        // 
        // std::cout << buffer.getField<boost::int32_t>(0, 2) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 2) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 2) << std::endl;

        // Check X's of first three points in block 20
        BOOST_CHECK(buffer.getField<boost::int32_t>(0, 0) == 63763550);
        BOOST_CHECK(buffer.getField<boost::int32_t>(1, 0) == 63765279);
        BOOST_CHECK(buffer.getField<boost::int32_t>(2, 0) == 63771207);
        
        // Check Y's of first three points in block 20
        BOOST_CHECK(buffer.getField<boost::int32_t>(0, 1) == 84992418);
        BOOST_CHECK(buffer.getField<boost::int32_t>(1, 1) == 85005705);
        BOOST_CHECK(buffer.getField<boost::int32_t>(2, 1) == 85005840);
        
        // Check Z's of first three points in block 20
        BOOST_CHECK(buffer.getField<boost::int32_t>(0, 2) == 42664);
        BOOST_CHECK(buffer.getField<boost::int32_t>(1, 2) == 43579);
        BOOST_CHECK(buffer.getField<boost::int32_t>(2, 2) == 42651);

    }

    return;
}


BOOST_AUTO_TEST_SUITE_END()
