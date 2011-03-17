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

#include <libpc/export.hpp>
#include <libpc/Dimension.hpp>


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

    bool hasDimension(Dimension::Field field) const
    {
        int index = m_indexTable[field];
        return index != -1;
    }

    bool hasDimension(Dimension::Field field, Dimension::DataType datatype) const
    {
        int index = m_indexTable[field];
        if (index == -1) return false;
        const Dimension& dim = m_dimensions[index];
        if (dim.getDataType() != datatype) return false;
        return true;
    }

    bool hasDimension(const Dimension& dim) const
    {
        return hasDimension(dim.getField(), dim.getDataType());
    }

    // assumes the index does, in fact, exist
    int getDimensionIndex(Dimension::Field field) const
    {
        int index = m_indexTable[field];
        assert(index != -1);
        return index;
    }
    
    boost::property_tree::ptree getPTree() const;

private:
    std::vector<Dimension> m_dimensions;

    // BUG: use boost::array?
    int m_indexTable[Dimension::Field_LAST]; // mapping from field name to index position, or -1 if field not present
};


LIBPC_DLL std::ostream& operator<<(std::ostream& os, Schema const&);


} // namespace liblas

#endif // LIBPC_SCHEMA_HPP_INCLUDED
