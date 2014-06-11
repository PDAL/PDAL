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

#include <pdal/Metadata.hpp>
#include <pdal/Utils.hpp>

#include <sstream>
#include <cstring>

#include <sstream>
#include <string>

#include <boost/algorithm/string.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>


namespace pdal
{

/**
Metadata::Metadata()

{
    setName("root");
    setType("blank");
    setValue<boost::blank>(boost::blank());
    return;
}

Metadata::Metadata(Metadata const& other)
    : m_tree(other.m_tree)
{}

Metadata::Metadata(std::string const& name)
{
    setType("blank");
    setValue<boost::blank>(boost::blank());
    setName(name);
}

Metadata::Metadata(boost::property_tree::ptree const& tree)
    : m_tree(tree)
{}

Metadata Metadata::operator+(const Metadata& rhs) const
{
    boost::property_tree::ptree tree = this->toPTree();
    tree.add_child(rhs.getName(), rhs.toPTree());
    return Metadata(tree);
}

bool Metadata::equal(Metadata const& rhs) const
{
    return m_tree == rhs.m_tree;
}


bool Metadata::operator==(Metadata const& rhs) const
{
    return equal(rhs);
}

bool Metadata::operator!=(Metadata const& rhs) const
{
    return !(equal(rhs));
}


**/

} // namespace pdal

namespace std
{

std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& data)
{

    std::string output = pdal::Utils::base64_encode(data.get());

    ostr << output;
    return ostr;
}

std::istream& operator>>(std::istream& istr, pdal::ByteArray& output)
{

    std::string data;
    istr >> data;
    std::vector<boost::uint8_t> d = pdal::Utils::base64_decode(data);

    output.set(d);
    return istr;
}


/**
std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& metadata)
{
    boost::property_tree::ptree tree = metadata.toPTree();

    boost::property_tree::write_json(ostr, tree);
    return ostr;
}
**/

}

