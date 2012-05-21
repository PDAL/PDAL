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

namespace metadata
{

Entry::Entry(std::string const& name)
    : boost::property_tree::ptree()
    
{
    setName(name);
    return;
}


std::string Entry::getTypeName() const
{
    switch (getType())
    {
        case metadata::Boolean:
            return std::string("boolean");
        case metadata::SignedInteger:
            return std::string("integer");
        case metadata::UnsignedInteger:
            return std::string("nonNegativeInteger");
        case metadata::Float:
            return std::string("float");
        case metadata::Double:
            return std::string("double");
        case metadata::String:
            return std::string("string");
        case metadata::Bytes:
            return std::string("base64Binary");
        case metadata::Bounds:
            return std::string("bounds");
        case metadata::SpatialReference:
            return std::string("spatialreference");
        case metadata::MData:
            return std::string("metadata");
        case metadata::UUID:
            return std::string("uuid");
        default:
            return std::string("none");
    }
}


// boost::property_tree::ptree Entry::toPTree() const
// {
//     using boost::property_tree::ptree;
//     ptree dim;
//     dim.put("name", getName());
//     std::ostringstream oss;
//     dim.put("value", get_value<std::string>());
//     dim.put("type", getTypeName());
// 
//     return dim;
// }
// 
// std::string Entry::to_xml()
// {
//     std::ostringstream oss;
// 
//     using boost::property_tree::ptree;
// 
//     ptree tree = toPTree();
// 
//     oss << "<Entry name=\"" << tree.get<std::string>("name") <<"\" type=\"" << tree.get<std::string>("type") <<"\"";
//     oss << ">";
// 
//     oss << "</Entry>";
//     return oss.str();
// 
// 
// }
// std::ostream& operator<<(std::ostream& ostr, const Entry& metadata)
// {
//     ostr << metadata.get_value<std::string>() << std::endl;
//     //ostr << metadata.getNamespace() << ":" << metadata.getName() << "=" << metadata.getVariant() << std::endl;
//     return ostr;
// }
// 

} // metadata


std::string Metadata::to_xml() const
{
    using namespace boost;

    property_tree::ptree m_tree = toPTree();

    property_tree::ptree::const_iterator iter = m_tree.begin();

    property_tree::ptree output;

    while (iter != m_tree.end())
    {
        if (iter->first != "entry")
            throw pdal_error("malformed Metadata ptree");
        const property_tree::ptree& e_tree = iter->second;

        property_tree::ptree entry;

        const std::string& name = e_tree.get_child("name").get_value<std::string>();
        const std::string& value = e_tree.get_child("value").get_value<std::string>();
        const std::string& type = e_tree.get_child("type").get_value<std::string>();

        entry.put_value(value);
        entry.put("<xmlattr>.name", name);
        entry.put("<xmlattr>.type", type);


        output.add_child("Entry", entry);
        ++iter;
    }

    property_tree::ptree tree;
    tree.add_child("Metadata", output);

    std::ostringstream oss;
    boost::property_tree::xml_parser::write_xml(oss, tree);
    return oss.str();

}


std::string Metadata::to_json() const
{
    using namespace boost;

    property_tree::ptree tree = toPTree();

    std::ostringstream oss;
    boost::property_tree::json_parser::write_json(oss, tree);

    return oss.str();

}

void Metadata::addEntry(metadata::Entry const& m)
{
    metadata::index_by_name& index = m_metadata.get<metadata::name>();

    std::pair<metadata::index_by_name::iterator, bool> q = index.insert(m);
    // if insert failed because an entry of the same name already exists, 
    // overwrite it
    if (!q.second)
    {
        bool didSet = setEntry(m);
        if (!didSet)
            throw pdal_error("Unable to addEntry because same name exists!");
    }

    return;
}


metadata::Entry const& Metadata::getEntry(std::string const& t) const
{
    metadata::index_by_name const& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::const_iterator it = name_index.find(t);

    if (it != name_index.end())
    {
        return *it;
    }

    std::ostringstream oss;
    oss << "Entry with name '" << t << "' not found, unable to Metadata::getMetadata";

    throw metadata_not_found(oss.str());


}

metadata::Entry const& Metadata::getEntry(std::size_t t) const
{
    metadata::index_by_index const& idx = m_metadata.get<metadata::index>();

    if (t >= idx.size())
        throw dimension_not_found("Index position is not valid");

    return idx.at(t);
}

boost::optional<metadata::Entry const&> Metadata::getEntryOptional(std::size_t t) const
{
    try
    {
        metadata::Entry const& m = getEntry(t);
        return boost::optional<metadata::Entry const&>(m);
    }
    catch (pdal::dimension_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }
}


boost::optional<metadata::Entry const&> Metadata::getEntryOptional(std::string const& t) const
{

    try
    {
        metadata::Entry const& m = getEntry(t);
        return boost::optional<metadata::Entry const&>(m);
    }
    catch (pdal::metadata_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }

}

bool Metadata::setEntry(metadata::Entry const& m)
{
    metadata::index_by_name& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::iterator it = name_index.find(m.getName());

    if (it != name_index.end())
    {
        bool didReplace = name_index.replace(it, m);
        if (!didReplace) 
            return false;
        return true;
    }
    else
    {
        return false;
    }

    return false;
}

Metadata::Metadata(Metadata const& other)
    : m_metadata(other.m_metadata)
{
}

Metadata& Metadata::operator=(Metadata const& rhs)
{
    if (&rhs != this)
    {
        m_metadata = rhs.m_metadata;
    }
    return *this;
}

Metadata Metadata::operator+(const Metadata& rhs) const
{

    Metadata output;

    metadata::EntryMap const& idx = rhs.getMetadata();

    metadata::index_by_index const& that_datums = idx.get<metadata::index>();

    for (metadata::index_by_index::const_iterator i = that_datums.begin();
            i != that_datums.end();
            ++i)
    {
        output.addEntry(*i);
    }


    metadata::index_by_index const& this_datums = m_metadata.get<metadata::index>();

    for (metadata::index_by_index::const_iterator i = this_datums.begin();
            i != this_datums.end();
            ++i)
    {
        output.addEntry(*i);
    }

    return output;
}


boost::property_tree::ptree Metadata::toPTree() const
{
    boost::property_tree::ptree tree;

    metadata::index_by_index const& idx = m_metadata.get<metadata::index>();

    for (metadata::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const metadata::Entry& entry = *iter;
        // tree.add_child("entry", entry.toPTree());
    }

    return tree;
}

} // namespace pdal


namespace std
{

std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& data)
{

    std::string output = pdal::Utils::base64_encode(data.get());

    ostr << output;
    return ostr;
}

std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& metadata)
{
    boost::property_tree::ptree tree = metadata.toPTree();

    boost::property_tree::write_json(ostr, tree);
    return ostr;
}


}
