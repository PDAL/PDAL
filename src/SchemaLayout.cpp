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

#include <pdal/SchemaLayout.hpp>

namespace pdal
{


SchemaLayout::SchemaLayout(const Schema& schema)
    : 
    m_schema(schema)
    , m_byteSize(0)
{
    calculateSizes();
    return;
}


/// copy constructor
SchemaLayout::SchemaLayout(SchemaLayout const& other) 
    : 
    m_schema(other.m_schema)
    , m_dimensionLayouts(other.m_dimensionLayouts)
    , m_byteSize(other.m_byteSize)
{
}


// assignment constructor
SchemaLayout& SchemaLayout::operator=(SchemaLayout const& rhs)
{
    if (&rhs != this)
    {
        m_dimensionLayouts = rhs.m_dimensionLayouts;
        m_byteSize = rhs.m_byteSize;
        m_schema = rhs.m_schema;
    }

    return *this;
}


bool SchemaLayout::operator==(const SchemaLayout& other) const
{
    if (m_byteSize == other.m_byteSize &&
        m_dimensionLayouts == other.m_dimensionLayouts &&
        m_schema == other.m_schema)
    {
        return true;
    }

    return false;
}


bool SchemaLayout::operator!=(const SchemaLayout& other) const
{
  return !(*this==other);
}


void SchemaLayout::calculateSizes()
{
    // to make life easy, for now we are going to assume that each Dimension 
    // is byte-aligned and occupies an integral number of bytes

    std::size_t offset = 0;

    Schema::Dimensions dims = m_schema.getDimensions();

    m_dimensionLayouts.clear();

    int i=0;
    for (Schema::DimensionsCIter iter = dims.begin(); iter != dims.end(); ++iter)
    {
        const Dimension& dim = *iter;

        DimensionLayout layout(dim); 
        layout.setByteOffset(offset);

        offset += dim.getByteSize();

        layout.setPosition(i);
        ++i;

        m_dimensionLayouts.push_back(layout);
    }

    m_byteSize = offset;

    return;
}


std::ostream& operator<<(std::ostream& os, SchemaLayout const&)
{
    ////using property_tree::ptree;
    ////ptree tree = schema.getPTree();

    ////os << "---------------------------------------------------------" << std::endl;
    ////os << "  Schema Summary" << std::endl;
    ////os << "---------------------------------------------------------" << std::endl;

    ////ptree::const_iterator i;

    //////ptree dims = tree.get_child("LASSchema.dimensions");
    ///////////////os << "  Point Format ID:             " << tree.get<std::string>("LASSchema.formatid") << std::endl;
    ////os << "  Number of dimensions:        " << schema.getDimensions().size() << std::endl;
    ////os << "  Size in bytes:               " << schema.getByteSize() << std::endl;

    ////os << std::endl;
    ////os << "  Dimensions" << std::endl;
    ////os << "---------------------------------------------------------" << std::endl;

    ////os << "  ";

    ////const Schema::Dimensions& dimensions = schema.getDimensions();
    ////for (Schema::DimensionsCIter iter = dimensions.cbegin(); iter != dimensions.cend(); ++iter)
    ////{
    ////    os << *iter;
    ////    os << "  ";
    ////}


    ////os << std::endl;

    return os;
}


} // namespace pdal
