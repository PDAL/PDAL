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
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <boost/property_tree/json_parser.hpp>


namespace pdal
{

namespace metadata {
    
Entry::Entry( std::string const& name, 
                    std::string const& ns) 
    : m_name(name)
    , m_namespace(ns) 
    , m_type(metadata::String)
    , m_uuid(boost::uuids::nil_uuid())
    , m_parentDimensionID(boost::uuids::nil_uuid())
{
    return;
}


Entry::Entry(const Entry& other)
    : m_variant(other.m_variant)
    , m_name(other.m_name)
    , m_namespace(other.m_namespace)
    , m_type(other.m_type)
    , m_uuid(other.m_uuid)
    , m_parentDimensionID(other.m_parentDimensionID)
{
    return;
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

void Entry::setUUID(std::string const& id)
{
    boost::uuids::string_generator gen;
    m_uuid = gen(id);
}

void Entry::createUUID()
{
    m_uuid = boost::uuids::random_generator()();
}

boost::property_tree::ptree Entry::toPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    dim.put("namespace", getNamespace());
    std::ostringstream oss;
    oss << m_variant;
    dim.put("value", oss.str());
    dim.put("type", getType());
    dim.put("uid", getUUID());
    
    for (metadata::MetadataAttributeM::const_iterator i = m_attributes.begin(); i != m_attributes.end(); ++i)
    {
        ptree at;
        at.put(i->first, i->second);
        dim.add_child("attribute", at);
    }
    
    dim.put("parent", getParent());
    return dim;
}

std::ostream& operator<<(std::ostream& ostr, const Entry& metadata)
{
    ostr << metadata.getVariant() << std::endl;
    //ostr << metadata.getNamespace() << ":" << metadata.getName() << "=" << metadata.getVariant() << std::endl;
    return ostr;
}


} // metadata 

void Metadata::addMetadata(metadata::Entry const& m)
{
    metadata::index_by_name& index = m_metadata.get<metadata::name>();

    std::pair<metadata::index_by_name::iterator, bool> q = index.insert(m);
    if (!q.second) 
    {
        std::ostringstream oss;
        oss << "Could not insert into schema index because of " << q.first->getName();
        throw metadata_error(oss.str());
    }

    return;
}


metadata::Entry const& Metadata::getMetadata(std::string const& t, std::string const& ns) const
{
    metadata::index_by_name const& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::const_iterator it = name_index.find(t);
    
    metadata::index_by_name::size_type count = name_index.count(t);

    std::ostringstream oss;
    oss << "Entry with name '" << t << "' not found, unable to Metadata::getMetadata";

    if (it != name_index.end()) {
        
        if (ns.size())
        {
            while (it != name_index.end())
            {
                if (boost::equals(ns, it->getNamespace()))
                    return *it;
                ++it;
            }
            
        } 
        
        if (count > 1) {

            std::pair<metadata::index_by_name::const_iterator, metadata::index_by_name::const_iterator> ret = name_index.equal_range(t);
            boost::uint32_t num_parents(0);
            boost::uint32_t num_children(0);
            std::map<metadata::id, metadata::id> relationships;
            
            // Test to make sure that the number of parent dimensions all with 
            // the same name is equal to only 1. If there are multiple 
            // dimensions with the same name, but no relationships defined, 
            // we are in an error condition
            for (metadata::index_by_name::const_iterator  o = ret.first; o != ret.second; ++o)
            {
                // Put a map together that maps parents to children that 
                // we are going to walk to find the very last child in the 
                // graph.
                std::pair<metadata::id, metadata::id> p( o->getParent(), o->getUUID());
                relationships.insert(p);
                
                // The parent dimension should have a nil parent of its own.
                // nil_uuid is the default parent of all dimensions as the y
                // are created
                if (o->getParent().is_nil()) 
                {
                    num_parents++;
                }
                else
                {
                    num_children++;
                }
                
            }
            
            if (num_parents != 1)
            {
                std::ostringstream oss;
                
                oss << "PointBuffer has multiple dimensions with name '" << t << "', but "
                       "their parent/child relationships are not coherent. Multiple "
                       "parents are present.";
                
                throw multiple_parent_metadata(oss.str());
            }
            
            metadata::id parent = boost::uuids::nil_uuid();
            
            // Starting at the parent (nil uuid), walk the child/parent graph down to the 
            // end.  When we're done finding dimensions, what's left is the child 
            // at the end of the graph.
            std::map<metadata::id, metadata::id>::const_iterator p = relationships.find(parent);
            pdal::metadata::id child;
            while (p != relationships.end())
            {
                child = p->second;
                p = relationships.find(p->second);
            }
            metadata::index_by_uid::const_iterator pi = m_metadata.get<metadata::uid>().find(child);
            if (pi != m_metadata.get<metadata::uid>().end())
            {
                return *pi;
            } 
            else 
            {
                std::ostringstream errmsg;
                errmsg << "Unable to fetch subjugate metadata entry with id '" << child << "' in PointBuffer";
                throw metadata_not_found(errmsg.str());
            }
        }
        return *it;
    } else {
        boost::uuids::uuid ps1;
        try
        {
            boost::uuids::string_generator gen;
            ps1 = gen(t);
        } catch (std::runtime_error&)
        {
            // invalid string for uuid
            throw metadata_not_found(oss.str());
        }

        metadata::index_by_uid::const_iterator i = m_metadata.get<metadata::uid>().find(ps1);

        if (i != m_metadata.get<metadata::uid>().end())
        {
            if (ns.size())
            {
                while (i != m_metadata.get<metadata::uid>().end())
                {
                    if (boost::equals(ns, i->getNamespace()))
                        return *i;
                    ++i;
                }
            
            }
            
            return *i;
        } else 
        {
            oss.str("");
            oss << "Entry with name '" << t << "' not found, unable to Metadata::getMetadata";
            throw metadata_not_found(oss.str());
        }

    }

}

metadata::Entry const& Metadata::getMetadata(std::size_t t) const
{
    metadata::index_by_index const& idx = m_metadata.get<metadata::index>();
    
    if (t >= idx.size())
        throw dimension_not_found("Index position is not valid");
    
    return idx.at(t);
}

boost::optional<metadata::Entry const&> Metadata::getMetadataOptional(std::size_t t) const
{
    try
    {
        metadata::Entry const& m = getMetadata(t);
        return boost::optional<metadata::Entry const&>(m);
    } catch (pdal::dimension_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }
}

metadata::Entry const& Metadata::getMetadata(metadata::id const& t) const
{
    metadata::index_by_uid::const_iterator it = m_metadata.get<metadata::uid>().find(t);

    if (it != m_metadata.get<metadata::uid>().end())
    {
        return *it;
    }    
    
    std::ostringstream oss;
    oss << "getMetadata: metadata entry not found with uuid '" << boost::lexical_cast<std::string>(t) << "'";
    throw metadata_not_found(oss.str());

}

boost::optional<metadata::Entry const&> Metadata::getMetadataOptional(metadata::id const& t) const
{
    try
    {
        metadata::Entry const& m = getMetadata(t);
        return boost::optional<metadata::Entry const&>(m);
    } catch (pdal::metadata_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }
}


boost::optional<metadata::Entry const&> Metadata::getMetadataOptional(std::string const& t, std::string const& ns) const
{

    try
    {
        metadata::Entry const& m = getMetadata(t, ns);
        return boost::optional<metadata::Entry const&>(m);
    } catch (pdal::metadata_not_found&)
    {
        return boost::optional<metadata::Entry const&>();
    }

}

bool Metadata::setMetadata(metadata::Entry const& m)
{
    metadata::index_by_name& name_index = m_metadata.get<metadata::name>();
    metadata::index_by_name::iterator it = name_index.find(m.getName());
    
    // FIXME: If there are two metadata with the same name here, we're 
    // screwed if they both have the same namespace too
    if (it != name_index.end()) {
        while (it != name_index.end())
        {
            if (boost::equals(m.getNamespace(), it->getNamespace()))
            {
                name_index.replace(it, m);
                return true;
            }
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
    
    metadata::index_by_index const& datums = idx.get<metadata::index>();
    
    for (metadata::index_by_index::const_iterator i = datums.begin();
         i != datums.end();
         ++i)
    {
        output.addMetadata(*i);
    }
    return output;
}

std::vector<metadata::Entry> Metadata::getEntriesForNamespace(std::string const& ns) const
{
    std::vector<metadata::Entry> output;

    metadata::index_by_namespace const& idx = m_metadata.get<metadata::ns>();
    metadata::index_by_namespace::iterator it = idx.find(ns);

    while (it != idx.end())
    {
        output.push_back(*it);
        ++it;
    }

    return output;
}

metadata::EntryMap::size_type Metadata::size(std::string const& ns) const
{

    std::vector<metadata::Entry> output;

    metadata::index_by_namespace const& idx = m_metadata.get<metadata::ns>();
    metadata::index_by_namespace::iterator it = idx.find(ns);
    metadata::index_by_namespace::size_type i(0);
    
    while (it != idx.end())
    {
        ++i;
        ++it;
    }

    return i;
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
