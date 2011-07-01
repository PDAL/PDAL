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

#include <pdal/StageIterator.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/drivers/terrasolid/Reader.hpp>
#include <pdal/filters/CacheFilter.hpp>
#include "Support.hpp"

#include <iostream>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(TerraSolidReaderTest)


#define Compare(x,y)    BOOST_CHECK_CLOSE(x,y,0.00001);


void Check_Point(const pdal::PointBuffer& data, const ::pdal::Schema& schema, 
                       std::size_t index, 
                       double xref, double yref, double zref,
                       double tref)
{

    int offsetX = schema.getDimensionIndex(pdal::Dimension::Field_X, pdal::Dimension::Int32);
    int offsetY = schema.getDimensionIndex(pdal::Dimension::Field_Y, pdal::Dimension::Int32);
    int offsetZ = schema.getDimensionIndex(pdal::Dimension::Field_Z, pdal::Dimension::Int32);
    int offsetTime = schema.getDimensionIndex(pdal::Dimension::Field_Time, pdal::Dimension::Uint32);
    
    boost::int32_t x = data.getField<boost::int32_t>(index, offsetX);
    boost::int32_t y = data.getField<boost::int32_t>(index, offsetY);
    boost::int32_t z = data.getField<boost::int32_t>(index, offsetZ);
    boost::uint32_t t = data.getField<boost::uint32_t>(index, offsetTime);

    double x0 = schema.getDimension(offsetX).applyScaling<boost::int32_t>(x);
    double y0 = schema.getDimension(offsetY).applyScaling<boost::int32_t>(y);
    double z0 = schema.getDimension(offsetZ).applyScaling<boost::int32_t>(z);
    double t0 = schema.getDimension(offsetTime).applyScaling<boost::uint32_t>(t);

  
    // std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    // std::cout.precision(6);
    // std::cout << "expected x: " << xref << " y: " << yref << " z: " << zref << " t: " << tref << std::endl;
    // 
    // std::cout << "actual   x: " << x0 << " y: " << y0 << " z: " << z0 << " t0: " << t0 << std::endl;
    
    Compare(x0, xref);
    Compare(y0, yref);
    Compare(z0, zref);
    Compare(t0, tref);
}

BOOST_AUTO_TEST_CASE(test_10_word)
{
    pdal::Options options;
    // std::string filename = Support::datapath("20050903_231839.qi");

    std::string filename = Support::datapath("terrasolid/20020715-time-color.bin");

    
    boost::property_tree::ptree& tree = options.GetPTree();
    tree.put<std::string>("input", filename);
    pdal::drivers::terrasolid::Reader reader(options);
    BOOST_CHECK(reader.getDescription() == "TerraSolid Reader");
    BOOST_CHECK_EQUAL(reader.getName(), "drivers.terrasolid.reader");
    BOOST_CHECK_EQUAL(reader.getNumPoints(), 1000);

    const Schema& schema = reader.getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 3);
    
    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();
    
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead,3u);
    }

    delete iter;

    Check_Point(data, schema, 0, 363127.94, 3437612.33, 55.26, 580220.5528);
    Check_Point(data, schema, 1, 363128.12, 3437613.01, 55.33, 580220.5530);
    Check_Point(data, schema, 2, 363128.29, 3437613.66, 55.28, 580220.5530);


    return;
}
// 
// BOOST_AUTO_TEST_CASE(test_14_word)
// {
//     pdal::Options options;
//     // std::string filename = Support::datapath("20050903_231839.qi");
// 
//     std::string filename = Support::datapath("terrasolid/14-word.qi");
// 
//     
//     boost::property_tree::ptree& tree = options.GetPTree();
//     tree.put<std::string>("input", filename);
//     pdal::drivers::terrasolid::Reader reader(options);
// 
//     const Schema& schema = reader.getSchema();
//     SchemaLayout layout(schema);
// 
//     PointBuffer data(layout, 3);
//     
//     pdal::SequentialIterator* iter = reader.createSequentialIterator();
//     
//     {
//         boost::uint32_t numRead = iter->read(data);
//         BOOST_CHECK_EQUAL(numRead,3);
//     }
// 
// 
//     Check_Point(data, schema, 0, 35.623317, 244.306337, 1056830.000000, 903);
//     Check_Point(data, schema, 1, 35.623280, 244.306260, 1056409.000000, 903);
//     Check_Point(data, schema, 2, 35.623257, 244.306204, 1056483.000000, 903);
// 
// 
//     return;
// }



BOOST_AUTO_TEST_SUITE_END()
