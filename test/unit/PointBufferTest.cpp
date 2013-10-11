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
#include <boost/property_tree/xml_parser.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PointBufferTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1("Y", dimension::SignedInteger, 4);
    Dimension d2("X", dimension::SignedInteger, 4);
    Schema schema;
    schema.appendDimension(d1);
    schema.appendDimension(d2);

    PointBuffer data(schema, 10);

    BOOST_CHECK(data.getCapacity() == 10);
    BOOST_CHECK(data.getSchema() == schema);

    return;
}


PointBuffer* makeTestBuffer()
{
    Dimension d1("Classification", dimension::UnsignedInteger, 1);
    Dimension d2("X", dimension::SignedInteger, 4);
    Dimension d3("Y", dimension::Float, 8);
    Schema schema;
    schema.appendDimension(d1);
    schema.appendDimension(d2);
    schema.appendDimension(d3);

    std::size_t offX = schema.getDimension(0).getByteOffset();
    BOOST_CHECK(offX==0);
    std::size_t offY = schema.getDimension(1).getByteOffset();
    BOOST_CHECK(offY==1);
    std::size_t offZ = schema.getDimension(2).getByteOffset();
    BOOST_CHECK(offZ==5);

    boost::uint32_t capacity = 17;
    PointBuffer* data = new PointBuffer(schema, capacity);

    BOOST_CHECK(data->getCapacity() == capacity);

    Dimension const& dimC = data->getSchema().getDimension("Classification");
    Dimension const& dimX = data->getSchema().getDimension("X");
    Dimension const& dimY = data->getSchema().getDimension("Y");

    // write the data into the buffer
    for (boost::uint32_t i=0; i<data->getCapacity(); i++)
    {
        const boost::uint8_t x = static_cast<boost::uint8_t>(i)+1;
        const boost::int32_t y = i*10;
        const double z = i * 100;

        data->setField(dimC, i, x);
        data->setField(dimX, i, y);
        data->setField(dimY, i, z);
        data->setNumPoints(i+1);

    }
    BOOST_CHECK(data->getCapacity() ==17);
    BOOST_CHECK(data->getNumPoints() ==17);
    return data;
}


static void verifyTestBuffer(const PointBuffer& data)
{
    Dimension const& dimC = data.getSchema().getDimension("Classification");
    Dimension const& dimX = data.getSchema().getDimension("X");
    Dimension const& dimY = data.getSchema().getDimension("Y");

    // read the data back out
    for (int i=0; i<17; i++)
    {
        const boost::uint8_t x = data.getField<boost::uint8_t>(dimC, i);
        const boost::int32_t y = data.getField<boost::int32_t>(dimX, i);
        const double z = data.getField<double>(dimY, i);

        BOOST_CHECK(x == i+1);
        BOOST_CHECK(y == i*10);

        BOOST_CHECK(Utils::compare_approx(z, static_cast<double>(i)*100.0, (std::numeric_limits<double>::min)()) == true);

    }
}

BOOST_AUTO_TEST_CASE(test_get_set)
{
    PointBuffer* data = makeTestBuffer();
    verifyTestBuffer(*data);
    delete data;
}


BOOST_AUTO_TEST_CASE(test_copy)
{
    PointBuffer* data = makeTestBuffer();

    PointBuffer d2(data->getSchema(), 19);

    d2.copyPointFast(0, 10, *data);
    d2.copyPointFast(18, 11, *data);
    d2.copyPointsFast(1, 0, *data, 17);

    Dimension const& dimC = d2.getSchema().getDimension("Classification");
    Dimension const& dimX = d2.getSchema().getDimension("X");
    Dimension const& dimY = d2.getSchema().getDimension("Y");

    // read the data back out
    {
        const boost::uint8_t x = d2.getField<boost::uint8_t>(dimC, 0);
        const boost::int32_t y = d2.getField<boost::int32_t>(dimX, 0);
        const double z = d2.getField<double>(dimY, 0);

        int ii = 10;


        BOOST_CHECK(x == ii+1);
        BOOST_CHECK(y == ii*10);
        BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }
    for (int i=1; i<18; i++)
    {
        const boost::uint8_t x = d2.getField<boost::uint8_t>(dimC, i);
        const boost::int32_t y = d2.getField<boost::int32_t>(dimX, i);
        const double z = d2.getField<double>(dimY, i);

        int ii = i-1;
        BOOST_CHECK(x == ii+1);
        BOOST_CHECK(y == ii*10);
        BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }
    {
        const boost::uint8_t x = d2.getField<boost::uint8_t>(dimC, 18);
        const boost::int32_t y = d2.getField<boost::int32_t>(dimX, 18);
        const double z = d2.getField<double>(dimY, 18);

        int ii = 11;
        BOOST_CHECK(x == ii+1);
        BOOST_CHECK(y == ii*10);
        BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }

    delete data;
}

BOOST_AUTO_TEST_CASE(test_copy_constructor)
{
    PointBuffer* data = makeTestBuffer();

    PointBuffer d2(*data);
    verifyTestBuffer(d2);
    delete data;
}

BOOST_AUTO_TEST_CASE(test_assignment_constructor)
{
    PointBuffer* data = makeTestBuffer();

    PointBuffer d2 = *data;
    verifyTestBuffer(d2);
    delete data;
}


BOOST_AUTO_TEST_CASE(PointBufferTest_ptree)
{
    PointBuffer* data = makeTestBuffer();

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);

    boost::property_tree::ptree tree = data->toPTree();

    delete data;

    boost::property_tree::write_xml(ss1, tree);

    const std::string out1 = ss1.str();

    static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    const std::string ref = xml_header + "<0><Classification>1</Classification><X>0</X><Y>0</Y></0><1><Classification>2</Classification><X>10</X><Y>100</Y></1>";

    BOOST_CHECK_EQUAL(ref, out1.substr(0, ref.length()));

    return;
}


// BOOST_AUTO_TEST_CASE(PointBufferTest_large_buffer)
// {
// 
//     Dimension d1("Classification", dimension::UnsignedInteger, 1);
//     Dimension d2("X", dimension::SignedInteger, 4);
//     Dimension d3("Y", dimension::Float, 8);
// 
//     Schema schema;
//     schema.appendDimension(d1);
//     // schema.appendDimension(d2);
//     schema.appendDimension(d3);
// 
//     // boost::uint32_t extras(6);
//     // for (unsigned i = 0; i < extras; i++)
//     // {
//     //     std::ostringstream oss;
//     //     oss << "name" << i;
//     //     Dimension d(oss.str(), dimension::Float, 8);
//     // 
//     //     schema.appendDimension(d);        
//     //     
//     // }
//     
//     BOOST_CHECK_EQUAL(schema.getByteSize(),  8 + 1 );
//     boost::uint32_t capacity(4294967295);
//     
//     PointBuffer* data;
//     try
//     {
//         data = new PointBuffer(schema, capacity);
//         
//     } catch (std::bad_alloc&)
//     {
//         // We can't make one this big, test done.
//         BOOST_CHECK_EQUAL(1, 2);
//         return;
//     }
//     boost::uint64_t total_size = static_cast<boost::uint64_t>(data->getSchema().getByteSize()) * static_cast<boost::uint64_t>(data->getCapacity());
//     BOOST_CHECK_EQUAL(data->getCapacity(), capacity);
//     BOOST_CHECK_EQUAL(data->getBufferByteLength(), 38654705655u);
//     BOOST_CHECK_EQUAL(data->getBufferByteCapacity(), 38654705655u);
// 
//     Dimension const& cls = data->getSchema().getDimension("Classification");
//     boost::uint8_t c1 = 1u;
//     data->setField<boost::uint8_t>(cls, 4294967294, c1);
//     
//     boost::uint8_t c2 = data->getField<boost::uint8_t>(cls, 4294967294);
//     BOOST_CHECK_EQUAL(c2, c1);
// 
//     Dimension const& x = data->getSchema().getDimension("Classification");
//     boost::int32_t x1 = 12673;
//     data->setField<boost::int32_t>(x, 4294967293, x1);
//     
//     boost::int32_t x2 = data->getField<boost::int32_t>(x, 4294967293);
//     BOOST_CHECK_EQUAL(x1, x2);
//     
//     delete data;
// 
//     return;
// }


BOOST_AUTO_TEST_CASE(PointBufferTest_resetting)
{

    Dimension d1("Classification", dimension::UnsignedInteger, 1);
    Dimension d2("X", dimension::SignedInteger, 4);
    Dimension d3("Y", dimension::Float, 8);

    Schema schema;
    schema.appendDimension(d1);
    schema.appendDimension(d2);
    schema.appendDimension(d3);

    
    BOOST_CHECK_EQUAL(schema.getByteSize(), 8u + 4u + 1u);
    boost::uint32_t capacity(300);
    
    PointBuffer data(schema, capacity);
    boost::uint64_t total_size = static_cast<boost::uint64_t>(data.getSchema().getByteSize()) * static_cast<boost::uint64_t>(data.getCapacity());
    BOOST_CHECK_EQUAL(data.getCapacity(), capacity);
    BOOST_CHECK_EQUAL(data.getBufferByteCapacity(), 3900u);
    
    // resizing to something smaller isn't going to reallocate 
    // the array.
    data.resize(100);
    BOOST_CHECK_EQUAL(data.getBufferByteCapacity(), 1300u);
    BOOST_CHECK_EQUAL(data.getBufferByteLength(), 3900u);
    BOOST_CHECK_EQUAL(data.getCapacity(), 100u);

    data.resize(400);
    BOOST_CHECK_EQUAL(data.getBufferByteCapacity(), 5200u);
    BOOST_CHECK_EQUAL(data.getBufferByteLength(), 5200u);
    BOOST_CHECK_EQUAL(data.getCapacity(), 400u);
    

    return;
}


BOOST_AUTO_TEST_CASE(PointBufferTest_copy_like_Dimensions)
{

    Dimension d1("Classification", dimension::UnsignedInteger, 1);
    Dimension d2("X", dimension::SignedInteger, 4);
    Dimension d3("Y", dimension::Float, 8);

    Schema schema_a;
    schema_a.appendDimension(d1);
    schema_a.appendDimension(d2);
    schema_a.appendDimension(d3);

    Schema schema_b;
    schema_b.appendDimension(d1);
    schema_b.appendDimension(d2);
    
    
    BOOST_CHECK_EQUAL(schema_a.getByteSize(), 8u + 4u + 1u);
    BOOST_CHECK_EQUAL(schema_b.getByteSize(), 4u + 1u);

    boost::uint32_t capacity(200);
    PointBuffer data_a(schema_a, capacity);
    PointBuffer data_b(schema_b, capacity);
    
    Dimension const& cls = data_a.getSchema().getDimension("Classification");
    Dimension const& x = data_a.getSchema().getDimension("X");    
    for (boost::uint32_t i = 0; i < capacity; ++i)
    {
        data_a.setField<boost::uint8_t>(cls, i, 3);
        data_a.setField<boost::int32_t>(x, i, i);
    }
    
    BOOST_CHECK_EQUAL(150, data_a.getField<boost::int32_t>(x, 150));
    
    schema::DimensionMap* dimensions = data_a.getSchema().mapDimensions(data_b.getSchema());
    PointBuffer::copyLikeDimensions(data_a, data_b, *dimensions, 0, 0, 175);
    
    Dimension const& x2 = data_b.getSchema().getDimension("X");
    BOOST_CHECK_EQUAL(150, data_b.getField<boost::int32_t>(x2, 150));
    
    delete dimensions;

    return;
}

BOOST_AUTO_TEST_CASE(test_indexed)
{

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));
    BOOST_CHECK(reader.getDescription() == "Las Reader");
    reader.initialize();


    const Schema& schema = reader.getSchema();
    boost::uint32_t capacity(1000);
    PointBuffer data(schema, capacity);

    pdal::StageSequentialIterator* iter = reader.createSequentialIterator(data);

    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, capacity);
    }

    BOOST_CHECK_EQUAL(data.getCapacity(), capacity);
    BOOST_CHECK_EQUAL(data.getSchema(), schema);

    
    IndexedPointBuffer idata(data);
    BOOST_CHECK_EQUAL(idata.getCapacity(), capacity);
    BOOST_CHECK_EQUAL(idata.getSchema(), schema);

    idata.build();
    
    unsigned k = 8;
    
    // If the query distance is 0, just return the k nearest neighbors
    std::vector<size_t> ids = idata.neighbors(636199, 849238, 428.05, 0.0, k);
    BOOST_CHECK_EQUAL(ids.size(), k);
    BOOST_CHECK_EQUAL(ids[0], 8u);
    BOOST_CHECK_EQUAL(ids[1], 7u);
    BOOST_CHECK_EQUAL(ids[2], 9u);
    BOOST_CHECK_EQUAL(ids[3], 42u);
    BOOST_CHECK_EQUAL(ids[4], 40u);    
    
    std::vector<size_t> dist_ids = idata.neighbors(636199, 849238, 428.05, 100.0, 3);
    
    BOOST_CHECK_EQUAL(dist_ids.size(), 3u);
    BOOST_CHECK_EQUAL(dist_ids[0], 8u);        
    
    std::vector<size_t> nids = idata.neighbors(636199, 849238, 428.05, 0.0, k);
    
    BOOST_CHECK_EQUAL(nids.size(), k);
    BOOST_CHECK_EQUAL(nids[0], 8u);
    BOOST_CHECK_EQUAL(nids[1], 7u);
    BOOST_CHECK_EQUAL(nids[2], 9u);
    BOOST_CHECK_EQUAL(nids[3], 42u);
    BOOST_CHECK_EQUAL(nids[4], 40u);    
    
    std::vector<size_t> rids = idata.radius(637012.24, 849028.31, 431.66, 100000);
    BOOST_CHECK_EQUAL(rids.size(), 11u);    

    return;

}
BOOST_AUTO_TEST_SUITE_END()
