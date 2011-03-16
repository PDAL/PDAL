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

#include <libpc/drivers/faux/Reader.hpp>

using namespace libpc;

BOOST_AUTO_TEST_SUITE(FauxReaderTest)

BOOST_AUTO_TEST_CASE(test_constant)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Constant);
    BOOST_CHECK(reader.getName() == "Faux Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 750);
    
    boost::uint32_t numRead = reader.read(data);

    BOOST_CHECK(numRead == 750);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        float x = data.getField<float>(i, offsetX);
        float y = data.getField<float>(i, offsetY);
        float z = data.getField<float>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK( Utils::compare_approx( (double)x, 1.0, (std::numeric_limits<double>::min)()) == true);
        BOOST_CHECK(Utils::compare_approx( (double)y, 2.0, (std::numeric_limits<double>::min)()) == true);
        BOOST_CHECK(Utils::compare_approx( (double)z, 3.0, (std::numeric_limits<double>::min)()) == true);
        BOOST_CHECK(Utils::compare_approx( (double)t, (double) i, (std::numeric_limits<double>::min)()) == true);
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_random)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Random);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 750);
    
    boost::uint32_t numRead = reader.read(data);

    BOOST_CHECK(numRead == 750);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        float x = data.getField<float>(i, offsetX);
        float y = data.getField<float>(i, offsetY);
        float z = data.getField<float>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK(x >= 1.0 && x <= 101.0);
        BOOST_CHECK(y >= 2.0 && y <= 102.0);
        BOOST_CHECK(z >= 3.0 && z <= 103.0);
        BOOST_CHECK(t == i);
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_custom_fields)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);

    Dimension dimY(Dimension::Field_Y, Dimension::Uint8);
    Dimension dimX(Dimension::Field_X, Dimension::Uint8);
    std::vector<Dimension> dims;
    dims.push_back(dimY);
    dims.push_back(dimX);

    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Random, dims);

    const Schema& schema = reader.getHeader().getSchema();
    BOOST_CHECK(schema.getDimensions().size() == 2);
    BOOST_CHECK(schema.getDimension(0).getField() == Dimension::Field_Y);
    BOOST_CHECK(schema.getDimension(1).getField() == Dimension::Field_X);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
