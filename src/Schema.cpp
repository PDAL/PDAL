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

#include <pdal/exceptions.hpp>

#ifdef LIBPC_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

namespace pdal
{


Schema::Schema()
{
    for (int i=0; i<Dimension::Field_LAST; i++)
    {
        m_indexTable[i] = -1;
    }
    return;
}


/// copy constructor
Schema::Schema(Schema const& other) 
    : m_dimensions(other.m_dimensions)
{
    for (int i=0; i<Dimension::Field_LAST; i++)
    {
        m_indexTable[i] = other.m_indexTable[i];
    }
}


// assignment constructor
Schema& Schema::operator=(Schema const& rhs)
{
    if (&rhs != this)
    {
        m_dimensions = rhs.m_dimensions;
        for (int i=0; i<Dimension::Field_LAST; i++)
        {
            m_indexTable[i] = rhs.m_indexTable[i];
        }
    }

    return *this;
}


bool Schema::operator==(const Schema& other) const
{
    if (m_dimensions == other.m_dimensions)
    {
        for (int i=0; i<Dimension::Field_LAST; i++)
        {
            if (m_indexTable[i] != other.m_indexTable[i]) return false;
        }
        return true;
    }

    return false;
}


bool Schema::operator!=(const Schema& other) const
{
  return !(*this==other);
}


boost::property_tree::ptree Schema::getPTree() const
{
    using boost::property_tree::ptree;
    ptree pt;

    for (DimensionsCIter iter = m_dimensions.begin(); iter != m_dimensions.end(); ++iter)
    {
        pt.add_child("LASSchema.dimensions.dimension", (*iter).GetPTree());
    }

    return pt;
}


void Schema::addDimensions(const std::vector<Dimension>& dims)
{
    for (DimensionsCIter iter = dims.begin(); iter != dims.end(); ++iter)
    {
        const Dimension& dim = *iter;
        addDimension(dim);
    }

    return;
}


void Schema::addDimension(Dimension const& dim)
{
    std::size_t index = m_dimensions.size();

    m_dimensions.push_back(dim);

    const Dimension::Field field = dim.getField();
    assert(m_indexTable[field] == -1);
    m_indexTable[field] = (int)index;

    return;
}


void Schema::removeDimension(Dimension const& dim)
{
    const Dimension::Field field = dim.getField();
    assert(m_indexTable[field] != -1);
    m_indexTable[field] = -1;

    //std::size_t index = getDimensionIndex(dim);
    //m_dimensions[index] = Dimension(Dimension::Field_INVALID, Dimension::Int32);

    return;
}


const Dimension& Schema::getDimension(std::size_t index) const
{
    return m_dimensions[index];
}


Dimension& Schema::getDimension(std::size_t index)
{
    return m_dimensions[index];
}


const Schema::Dimensions& Schema::getDimensions() const
{
    return m_dimensions;
}


int Schema::getDimensionIndex(Dimension::Field field, Dimension::DataType datatype) const
{
    const int index = m_indexTable[field];
    
    // assert(index != -1);
    if (index == -1)
    {
        return -1;
        // throw pdal_error("Requested dimension field not present");
    }
    
    const Dimension& dim = m_dimensions[index];
    
    // assert(dim.getDataType() == datatype);
    if (dim.getDataType() != datatype)
    {
        throw dimension_not_found("Requested dimension field present, but with different datatype");
    }
    
    return index;
}


int Schema::getDimensionIndex(const Dimension& dim) const
{
    return getDimensionIndex(dim.getField(), dim.getDataType());
}


bool Schema::hasDimension(Dimension::Field field, Dimension::DataType datatype) const
{
    const int index = m_indexTable[field];
    if (index == -1) return false;
    const Dimension& dim = m_dimensions[index];
    if (dim.getDataType() != datatype) return false;
    return true;
}


bool Schema::hasDimension(const Dimension& dim) const
{
    return hasDimension(dim.getField(), dim.getDataType());
}


void Schema::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, Schema const& schema)
{
    using boost::property_tree::ptree;
    ptree tree = schema.getPTree();

    os << "---------------------------------------------------------" << std::endl;
    os << "  Schema Summary" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    ptree::const_iterator i;

    //ptree dims = tree.get_child("LASSchema.dimensions");
    ///////////os << "  Point Format ID:             " << tree.get<std::string>("LASSchema.formatid") << std::endl;
    os << "  Number of dimensions:        " << schema.getDimensions().size() << std::endl;
//    os << "  Size in bytes:               " << schema.getByteSize() << std::endl;

    os << std::endl;
    os << "  Dimensions" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    os << "  ";

    const Schema::Dimensions& dimensions = schema.getDimensions();
    for (Schema::DimensionsCIter iter = dimensions.begin(); iter != dimensions.end(); ++iter)
    {
        os << *iter;
        os << "  ";
    }


    os << std::endl;

    return os;
}


#ifdef LIBPC_HAVE_LIBXML2


#endif


Schema Schema::from_xml(std::string const& xml, std::string const& xsd)
{
#ifdef LIBPC_HAVE_LIBXML2

    pdal::schema::Reader reader(xml, xsd);
    
    pdal::Schema schema = reader.getSchema();
    return schema;

#endif
    return Schema();
}

Schema Schema::from_xml(std::string const& xml)
{
#ifdef LIBPC_HAVE_LIBXML2
    
    std::string xsd("");
    
    pdal::schema::Reader reader(xml, xsd);
    
    pdal::Schema schema = reader.getSchema();
    return schema;

#endif
    return Schema();
}

std::string Schema::to_xml(Schema const& schema)
{
#ifdef LIBPC_HAVE_LIBXML2

    pdal::schema::Writer writer(schema);
    
    return writer.getXML();

#endif
    return std::string("");
}

} // namespace pdal
