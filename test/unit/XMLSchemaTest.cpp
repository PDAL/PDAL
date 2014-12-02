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

#include "UnitTest.hpp"
#include <boost/cstdint.hpp>
#include <boost/property_tree/ptree.hpp>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <pdal/XMLSchema.hpp>
#include <pdal/Metadata.hpp>
#include <LasReader.hpp>
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

    std::string xml =
        ReadXML(TestConfig::g_data_path+"../../schemas/8-dimension-schema.xml");
    std::string xsd = ReadXML(TestConfig::g_data_path+"../../schemas/LAS.xsd");

    XMLSchema s1;
    s1.read(xml, xsd);

    PointContext ctx;
    XMLDimList dims = s1.dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        XMLDim& dim = *di;
        dim.m_id = ctx.registerOrAssignDim(dim.m_name, dim.m_type);
    }

    MetadataNode m;

    MetadataNode m1 = m.add("m1", 1u);
    MetadataNode m2 = m.add("m2", 1);
    MetadataNode m1prime = m.add("m1prime", "Some other metadata");
    m1.add("uuid", boost::uuids::nil_uuid());

    XMLSchema s2;
    std::string xml_output = s2.getXML(s1.dimTypes(), m);

    XMLSchema s3;
    s3.read(xml_output, xsd);
    XMLDimList dims3 = s3.dims();

    BOOST_CHECK_EQUAL(dims.size(), dims3.size());

    auto di1 = dims.begin();
    auto di3 = dims3.begin();
    while (di1 != dims.end() && di3 != dims3.end())
    {
        XMLDim& dim1 = *di1;
        XMLDim& dim3 = *di3;

        BOOST_CHECK_EQUAL(dim1.m_name, dim3.m_name);
        BOOST_CHECK_EQUAL(dim1.m_type, dim3.m_type);
        di1++;
        di3++;
    }
}

BOOST_AUTO_TEST_SUITE_END()
