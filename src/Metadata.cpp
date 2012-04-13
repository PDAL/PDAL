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

namespace metadata {

Entry::Entry( std::string const& name) 
    : m_name(name)
    , m_type(metadata::String)
    , m_description("")
{
    return;
}


Entry::Entry(const Entry& other)
    : m_variant(other.m_variant)
    , m_name(other.m_name)
    , m_type(other.m_type)
    , m_attributes(other.m_attributes)
    , m_description(other.m_description)
{
    return;
}

Entry& Entry::operator=(const Entry& rhs)
{
    if (&rhs != this)
    {   
        m_variant = rhs.m_variant;
        m_name = rhs.m_name;
        m_description = rhs.m_description;
        m_type = rhs.m_type;
        m_attributes = rhs.m_attributes;
    }
    return *this;
}

std::vector<std::string> Entry::getAttributeNames() const
{
    std::vector<std::string> output;
    metadata::MetadataAttributeM::const_iterator i = m_attributes.begin();
    while (i != m_attributes.end())
    {
        output.push_back(i->first);
        ++i;
    }
    
    return output;
}


void Entry::addAttribute(std::string const& name, std::string const value)
{
    std::pair<std::string, std::string> p(name, value);
    m_attributes.insert(p);
}

std::string Entry::getAttribute(std::string const& name) const
{
    metadata::MetadataAttributeM::const_iterator p = m_attributes.find(name);
    if (p != m_attributes.end())
        return p->second;
    else
        return std::string("");
}

std::string Entry::getTypeName() const
{
    switch(getType())
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
        case metadata::UUID:
            return std::string("uuid");
        default:
            return std::string("none");
    }
}

boost::optional<Metadata const&> Entry::getMetadata() const
{
    if (m_metadata.get())
        return boost::optional<Metadata const&>(*m_metadata.get());
    else
        return boost::optional<Metadata const&>();
}

void Entry::setMetadata(Metadata const& mdata) 
{
    metadata::MetadataPtr p = metadata::MetadataPtr(new Metadata(mdata));
    m_metadata = p;
}

boost::property_tree::ptree Entry::toPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    std::ostringstream oss;
    oss << m_variant;
    dim.put("value", oss.str());
    dim.put("type", getTypeName());
    
    for (metadata::MetadataAttributeM::const_iterator i = m_attributes.begin(); i != m_attributes.end(); ++i)
    {
        ptree attribute;
        attribute.put("value", i->second);
        attribute.put("name", i->first);
        dim.add_child("attribute", attribute);
    }
    
    return dim;
}

std::string Entry::to_xml()
{
    std::ostringstream oss;
    
    using boost::property_tree::ptree;
    
    ptree tree = toPTree();
    
    oss << "<Entry name=\"" << tree.get<std::string>("name") <<"\" type=\"" << tree.get<std::string>("type") <<"\"";
    oss << ">";
    
    for (metadata::MetadataAttributeM::const_iterator i = m_attributes.begin(); i != m_attributes.end(); ++i)
    {
        oss <<"<Attribute name=\"" << i->first <<"\">" << i->second <<"</Attribute>";
    }    

    oss << "</Entry>";
    return oss.str();
    
    
}
std::ostream& operator<<(std::ostream& ostr, const Entry& metadata)
{
    ostr << metadata.getVariant() << std::endl;
    //ostr << metadata.getNamespace() << ":" << metadata.getName() << "=" << metadata.getVariant() << std::endl;
    return ostr;
}


} // metadata 


boost::property_tree::ptree ByteArray::toPTreeImpl() const
{
    using namespace boost;
    
    property_tree::ptree tree;
    
    std::ostringstream data;
    data << *this;
    
    tree.put("data", data.str());
    tree.put("size", m_bytes.size());
    return tree;
}
    


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

        std::pair<property_tree::ptree::const_assoc_iterator, property_tree::ptree::const_assoc_iterator> ret = e_tree.equal_range("attribute");

        for (property_tree::ptree::const_assoc_iterator  o = ret.first; o != ret.second; ++o)
        {
            property_tree::ptree const& tree = o->second;
            std::string const& name =  tree.get_child("name").get_value<std::string>();
            std::string const& value =  tree.get_child("value").get_value<std::string>();

            boost::property_tree::ptree a;
            a.put_value(value);
            a.put("<xmlattr>.name", name);            
            entry.add_child("attribute", a);
        }
        
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
    if (!q.second) 
    {
        index.replace(q.first, m);
    }

    return;
}


metadata::Entry const& Metadata::getEntry(std::string const& t) const
{
    metadata::index_by_name const& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::const_iterator it = name_index.find(t);
    
    if (it != name_index.end()) {
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
    } catch (pdal::dimension_not_found&)
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
    } catch (pdal::metadata_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }

}

bool Metadata::setEntry(metadata::Entry const& m)
{
    metadata::index_by_name& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::iterator it = name_index.find(m.getName());
    
    // FIXME: If there are two metadata with the same name here, we're 
    // screwed if they both have the same namespace too
    if (it != name_index.end()) {
        while (it != name_index.end())
        {
            // if (boost::equals(m.getNamespace(), it->getNamespace()))
            // {
                name_index.replace(it, m);
                return true;
            // }
            ++it;
        }
    } else {
        std::ostringstream oss;
        oss << "Metadata with name '" << m.getName() << "' not found, unable to Metadata::setMetadata";
        throw metadata_not_found(oss.str());
    }

    return true;
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
        tree.add_child("entry", entry.toPTree());
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
