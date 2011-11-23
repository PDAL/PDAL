/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Schema implementation for C++ libLAS
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/Schema.hpp>

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>


#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

namespace pdal
{


Schema::Schema()
    : m_byteSize(0)
{
    return;
}

Schema::Schema(std::vector<Dimension> const& dimensions)
    : m_byteSize(0)
{
    
    for (   std::vector<Dimension>::const_iterator i = dimensions.begin(); 
            i != dimensions.end(); ++i)
    {
        appendDimension(*i);
    }
}

/// copy constructor
Schema::Schema(Schema const& other) 
    : m_dimensions(other.m_dimensions)
    , m_byteSize(other.m_byteSize)
    , m_index(other.m_index)
    , m_dimensions_map(other.m_dimensions_map)
{

}


// assignment constructor
Schema& Schema::operator=(Schema const& rhs)
{
    if (&rhs != this)
    {
        m_dimensions = rhs.m_dimensions;
        m_byteSize = rhs.m_byteSize;
        m_dimensions_map = rhs.m_dimensions_map;
        m_index = rhs.m_index;
    }

    return *this;
}


bool Schema::operator==(const Schema& other) const
{
    if (m_byteSize != other.m_byteSize) return false;
    if (m_dimensions.size() != other.m_dimensions.size()) return false;
    std::vector<Dimension>::size_type i(0);
    for (i = 0; i < m_dimensions.size(); ++i)
    {
        if (!(m_dimensions[i] == other.m_dimensions[i])) 
        {
            return false;
        }
    }

    return true;
}


bool Schema::operator!=(const Schema& other) const
{
  return !(*this==other);
}

    
void Schema::calculateSizes()
{
    // to make life easy, for now we are going to assume that each Dimension 
    // is byte-aligned and occupies an integral number of bytes

    std::size_t offset = 0;

    int i=0;
    for (std::vector<Dimension>::iterator iter = m_dimensions.begin(); iter != m_dimensions.end(); ++iter)
    {
        Dimension& dim = *iter;

        dim.setByteOffset(offset);

        offset += dim.getByteSize();

        dim.setPosition(i);
   
        ++i;
        
    }

    return;
}


void Schema::appendDimension(const Dimension& dim)
{
    m_dimensions.push_back(dim);
    std::pair<DimensionId::Id, std::size_t> p(dim.getId(), m_dimensions.size()-1);
    m_dimensions_map.insert(p);

    // Add/reset the dimension ptr on the dimensions map
    Dimension d(dim);
    
    schema::index_by_position& position_index = m_index.get<schema::position>();

    schema::index_by_position::const_reverse_iterator r = position_index.rbegin();
    
    // If we do not have anything in the index, set our current values.
    // Otherwise, use the values from the last entry in the index as our 
    // starting point
    if (r != position_index.rend()) 
    {
        Dimension const& t = *r;
    
        m_byteSize = m_byteSize + d.getByteSize();
        
        d.setByteOffset( t.getByteOffset() + t.getByteSize());
    } 
    else
    {
        m_byteSize = d.getByteSize();
        d.setByteOffset(0);
    }

    d.setPosition(m_index.size());

    std::pair<schema::index_by_position::iterator, bool> q = position_index.insert(d);
    if (! q.second) 
    {
        std::ostringstream oss;
        oss << "Could not insert into schema index because of " << q.first->getName();
        throw schema_error(oss.str());
    }

    assert(m_index.size() == m_dimensions.size());

    calculateSizes();

    return;
}


const Dimension& Schema::getDimension(std::size_t t) const
{
    schema::index_by_index const& idx = m_index.get<schema::index>();
    
    if (t >= idx.size())
        throw schema_error("Index position is not valid");
    
    // return idx.at(t);
    return m_dimensions[t];
}

bool Schema::setDimension(Dimension const& dim)
{
    schema::index_by_name& name_index = m_index.get<schema::name>();
    schema::index_by_name::iterator it = name_index.find(dim.getName());
    
    // FIXME: If there are two dimensions with the same name here, we're 
    // scrwed
    if (it != name_index.end()) {
        name_index.replace(it, dim);
    } else {
        std::ostringstream oss;
        oss << "Dimension with name '" << dim.getName() << "' not found, unable to Schema::setDimension";
        throw schema_error(oss.str());
    }
    
    for(std::vector<Dimension>::iterator i = m_dimensions.begin(); i != m_dimensions.end(); ++i)
    {
        if (i->getName() == dim.getName())
            *i = dim;
    }
    return true;
}


int Schema::getDimensionIndex(const DimensionId::Id& id) const
{
    std::map<DimensionId::Id, std::size_t>::const_iterator i = m_dimensions_map.find(id);
    
    int m = 0;
    
    if (i == m_dimensions_map.end()) 
        m = -1;
    else
        m = i->second;

    return m;
}


int Schema::getDimensionIndex(const Dimension& dim) const
{
    return getDimensionIndex(dim.getId());
}


bool Schema::hasDimension(const DimensionId::Id& field) const
{
    int t = getDimensionIndex(field);
    if (t == -1)
        return false;
    return true;
}


const Dimension& Schema::getDimension(const DimensionId::Id& field) const
{
    int t = getDimensionIndex(field);
    return m_dimensions[t];
}





Schema Schema::from_xml(std::string const& xml, std::string const& xsd)
{
#ifdef PDAL_HAVE_LIBXML2

    pdal::schema::Reader reader(xml, xsd);
    
    pdal::Schema schema = reader.getSchema();
    return schema;

#else
    return Schema();
#endif
}

Schema Schema::from_xml(std::string const& xml)
{
#ifdef PDAL_HAVE_LIBXML2
    
    std::string xsd("");
    
    pdal::schema::Reader reader(xml, xsd);
    
    pdal::Schema schema = reader.getSchema();
    return schema;

#else
    return Schema();
#endif
}

std::string Schema::to_xml(Schema const& schema)
{
#ifdef PDAL_HAVE_LIBXML2

    pdal::schema::Writer writer(schema);
    
    return writer.getXML();

#else
    return std::string("");
#endif
}


boost::property_tree::ptree Schema::toPTree() const
{
    boost::property_tree::ptree tree;

    for (std::vector<Dimension>::const_iterator iter = m_dimensions.begin(); iter != m_dimensions.end(); ++iter)
    {
        const Dimension& dim = *iter;
        tree.add_child("dimension", dim.toPTree());
    }

    return tree;
}


void Schema::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, pdal::Schema const& schema)
{
    boost::property_tree::ptree tree = schema.toPTree();

    boost::property_tree::write_json(os, tree);

    return os;
}


} // namespace pdal
