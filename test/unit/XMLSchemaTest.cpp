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

#include <libpc/XMLSchema.hpp>


#include <libpc/drivers/faux/Reader.hpp>
#include <libpc/drivers/faux/Writer.hpp>

#include <libpc/drivers/las/Reader.hpp>

#include <libpc/Iterator.hpp>
#include <libpc/Utils.hpp>

#include "Support.hpp"
#include "TestConfig.hpp"

#include <fstream>
using namespace libpc;




std::string ReadXML(std::string filename)
{

    std::istream* infile = Utils::openFile(filename);
    std::ifstream::pos_type size;
    // char* data;
    std::vector<char> data;
    if (infile->good()){
        infile->seekg(0, std::ios::end);
        size = infile->tellg();
        data.resize(static_cast<std::vector<char>::size_type>(size));
        // data = new char [size];
        infile->seekg (0, std::ios::beg);
        infile->read (&data.front(), size);
        // infile->close();

        // delete[] data;
        delete infile;
        return std::string(&data[0], data.size());
        // return data; 
    } 
    else 
    {   
        throw libpc_error("unable to open file!");
        // return data;
    }
    
}


BOOST_AUTO_TEST_SUITE(XMLSchemaTest)




BOOST_AUTO_TEST_CASE(test_schema_read)
{
    // std::istream* xml_stream = Utils::openFile(TestConfig::g_data_path+"schemas/8-dimension-schema.xml");
    // std::istream* xsd_stream = Utils::openFile(TestConfig::g_data_path+"/schemas/LAS.xsd");
    
    std::string xml = ReadXML(TestConfig::g_data_path+"schemas/8-dimension-schema.xml");
    std::string xsd = ReadXML(TestConfig::g_data_path+"/schemas/LAS.xsd");
    libpc::schema::Reader reader(xml, xsd);
    
    libpc::Schema schema = reader.getSchema();
    
    libpc::schema::Writer writer(schema);
    std::string xml_output = writer.write();

    libpc::schema::Reader reader2(xml_output, xsd);
    libpc::Schema schema2 = reader2.getSchema();
    
    libpc::Schema::Dimensions const& dims1 = schema.getDimensions();
    libpc::Schema::Dimensions const& dims2 = schema2.getDimensions();
    
    BOOST_CHECK_EQUAL(dims1.size(), dims2.size());
    
    for (boost::uint32_t i = 0; i < dims2.size(); ++i)
    {
        libpc::Dimension const& dim1 = schema.getDimension(i);
        libpc::Dimension const& dim2 = schema2.getDimension(i);
    
        BOOST_CHECK_EQUAL(dim1.getDataType(), dim2.getDataType());
        BOOST_CHECK_EQUAL(dim1.getByteSize(), dim2.getByteSize());

        BOOST_CHECK_EQUAL(dim1.getField(), dim2.getField());
        BOOST_CHECK_EQUAL(dim1.getDescription(), dim2.getDescription());
        
    }
    
}


BOOST_AUTO_TEST_SUITE_END()
