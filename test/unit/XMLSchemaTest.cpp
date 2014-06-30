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
#include <boost/property_tree/ptree.hpp>

#include <boost/uuid/uuid_io.hpp>

#include <pdal/XMLSchema.hpp>
#include <pdal/Metadata.hpp>

#include <pdal/drivers/faux/Reader.hpp>

#include <pdal/drivers/las/Reader.hpp>

#include <pdal/StageIterator.hpp>
#include <pdal/FileUtils.hpp>

#include "Support.hpp"
#include "TestConfig.hpp"

#include <fstream>
using namespace pdal;




std::string ReadXML(std::string filename)
{

    std::istream* infile = FileUtils::openFile(filename);
    std::ifstream::pos_type size;
    // char* data;
    std::vector<char> data;
    if (infile->good())
    {
        infile->seekg(0, std::ios::end);
        size = infile->tellg();
        data.resize(static_cast<std::vector<char>::size_type>(size));
        // data = new char [size];
        infile->seekg(0, std::ios::beg);
        infile->read(&data.front(), size);
        // infile->close();

        // delete[] data;
        delete infile;
        return std::string(&data[0], data.size());
        // return data;
    }
    else
    {
        throw pdal_error("unable to open file!");
        // return data;
    }

}


BOOST_AUTO_TEST_SUITE(XMLSchemaTest)




BOOST_AUTO_TEST_CASE(test_schema_read)
{
    using namespace pdal;

    // std::istream* xml_stream = Utils::openFile(TestConfig::g_data_path+"schemas/8-dimension-schema.xml");
    // std::istream* xsd_stream = Utils::openFile(TestConfig::g_data_path+"/schemas/LAS.xsd");

    std::string xml =
        ReadXML(TestConfig::g_data_path+"../../schemas/8-dimension-schema.xml");
    std::string xsd = ReadXML(TestConfig::g_data_path+"../../schemas/LAS.xsd");
    schema::Reader reader(xml, xsd);

    Schema schema = reader.getSchema();

    MetadataNode m;

    MetadataNode m1 = m.add("m1", 1u);
    MetadataNode m2 = m.add("m2", 1);
    MetadataNode m1prime = m.add("m1prime", "Some other metadata");
    m1.add("uuid", boost::uuids::nil_uuid());

    pdal::schema::Writer writer(schema);
    writer.setMetadata(m1);
    std::string xml_output = writer.getXML();
    
    pdal::schema::Reader reader2(xml_output, xsd);
    pdal::Schema schema2 = reader2.getSchema();

    schema::index_by_index const& dims1 =
        schema.getDimensions().get<schema::index>();
    schema::index_by_index const& dims2 =
        schema2.getDimensions().get<schema::index>();

    BOOST_CHECK_EQUAL(dims1.size(), dims2.size());

    for (size_t i = 0; i < dims2.size(); ++i)
    {
        pdal::Dimension const& dim1 = dims1[i];
        pdal::Dimension const& dim2 = dims2[i];

        BOOST_CHECK_EQUAL(dim1.getName(), dim2.getName());
        BOOST_CHECK_EQUAL(dim1.getInterpretation(), dim2.getInterpretation());
        BOOST_CHECK_EQUAL(dim1.getByteSize(), dim2.getByteSize());

        BOOST_CHECK_EQUAL(dim1.getDescription(), dim2.getDescription());
    }
}


BOOST_AUTO_TEST_CASE(test_schema_orientation)
{

  
    Dimension cls("Classification", dimension::UnsignedInteger, 1);
    Dimension x("X", dimension::SignedInteger, 4);
    Dimension y("Y", dimension::Float, 8);
    
    Schema schema;
    schema.appendDimension(x);
    schema.appendDimension(y);
    schema.appendDimension(cls);
    schema.setOrientation(schema::DIMENSION_INTERLEAVED);
  
    
    pdal::schema::Writer writer(schema);

    std::string xml_output = writer.getXML();
std::ostream* out = FileUtils::createFile("orientation-schema.xml");
out->write(xml_output.c_str(), strlen(xml_output.c_str()));
FileUtils::closeFile(out);
    
    pdal::schema::Reader reader2(xml_output, std::string(""));
    pdal::Schema schema2 = reader2.getSchema();
    
    BOOST_CHECK_EQUAL(schema2.getOrientation(), schema.getOrientation());

    schema::index_by_index const& dims1 = schema.getDimensions().get<schema::index>();
    schema::index_by_index const& dims2 = schema2.getDimensions().get<schema::index>();

    // const std::vector<pdal::Dimension>& dims1 = schema.getDimensions();
    // const std::vector<pdal::Dimension>& dims2 = schema2.getDimensions();

    BOOST_CHECK_EQUAL(dims1.size(), dims2.size());

    for (boost::uint32_t i = 0; i < dims2.size(); ++i)
    {
        pdal::Dimension const& dim1 = dims1[i];
        pdal::Dimension const& dim2 = dims2[i];

        BOOST_CHECK_EQUAL(dim1.getName(), dim2.getName());
        BOOST_CHECK_EQUAL(dim1.getInterpretation(), dim2.getInterpretation());
        BOOST_CHECK_EQUAL(dim1.getByteSize(), dim2.getByteSize());

        BOOST_CHECK_EQUAL(dim1.getDescription(), dim2.getDescription());

    }

}



BOOST_AUTO_TEST_SUITE_END()
