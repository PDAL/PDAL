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

#ifndef LIBPC_SCHEMA_HPP_INCLUDED
#define LIBPC_SCHEMA_HPP_INCLUDED

// std
#include <vector>
#include <list>

// boost

#include "libpc/export.hpp"
#include "libpc/Dimension.hpp"


namespace libpc
{


/// Schema definition
class LIBPC_DLL Schema
{
public:
    typedef std::vector<Dimension> Dimensions;
    typedef std::vector<Dimension>::iterator DimensionsIter;
    typedef std::vector<Dimension>::const_iterator DimensionsCIter;

public:
    Schema();
    Schema(Schema const& other);

    Schema& operator=(Schema const& rhs);

    bool operator==(const Schema& other) const;
    bool operator!=(const Schema& other) const;

    void addDimension(Dimension const& dim);
    void addDimensions(const std::vector<Dimension>& dims);

    const Dimension& getDimension(std::size_t index) const
    {
        return m_dimensions[index];
    }

    Dimension& getDimension(std::size_t index)
    {
        return m_dimensions[index];
    }

    const Dimensions& getDimensions() const
    {
        return m_dimensions;
    }

    // returns true if found and sets index, otherwise returns false
    // BUG: this is deadly slow, need to use old assoc array or fixed enum for field names
    bool findDimensionIndex(Dimension::Field field, std::size_t& index) const; 
    bool hasDimension(Dimension::Field field) const;

    // this version throws if the index not found
    std::size_t getDimensionIndex(Dimension::Field field) const; 

    boost::property_tree::ptree getPTree() const;

private:
    std::vector<Dimension> m_dimensions;

    // BUG: use boost::array?
    int m_indexTable[Dimension::Field_LAST]; // mapping from field name to index position, or -1 if field not present
};


class LIBPC_DLL SchemaLayout
{
public:
    typedef std::vector<DimensionLayout> DimensionLayouts;
    typedef std::vector<DimensionLayout>::iterator DimensionLayoutsIter;
    typedef std::vector<DimensionLayout>::const_iterator DimensionLayoutsCIter;

    SchemaLayout(const Schema&);
    SchemaLayout(SchemaLayout const& other);

    SchemaLayout& operator=(SchemaLayout const& rhs);

    bool operator==(const SchemaLayout& other) const;
    bool operator!=(const SchemaLayout& other) const;

    const Schema& getSchema() const 
    {
      return m_schema;
    }

    /// Fetch total byte size -- sum of all dimensions
    std::size_t getByteSize() const;

    void calculateSizes();

    const DimensionLayout& getDimensionLayout(std::size_t index) const
    {
        return m_dimensionLayouts[index];
    }

    DimensionLayout& getDimensionLayout(std::size_t index)
    {
        return m_dimensionLayouts[index];
    }

    const std::vector<DimensionLayout>& getDimensionLayouts() const
    {
        return m_dimensionLayouts;
    }

private:
    Schema m_schema;
    std::vector<DimensionLayout> m_dimensionLayouts;
    std::size_t m_byteSize;
};


LIBPC_DLL std::ostream& operator<<(std::ostream& os, Schema const&);
LIBPC_DLL std::ostream& operator<<(std::ostream& os, SchemaLayout const&);


} // namespace liblas

#endif // LIBPC_SCHEMA_HPP_INCLUDED
