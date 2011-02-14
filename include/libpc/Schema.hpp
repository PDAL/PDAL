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
    typedef std::list<Dimension> Dimensions;
    typedef std::list<Dimension>::iterator DimensionsIter;
    typedef std::list<Dimension>::const_iterator DimensionsCIter;

public:
    Schema();
    Schema& operator=(Schema const& rhs);
    Schema(Schema const& other);
     
    ~Schema() {}

    /// Fetch total byte size -- sum of all dimensions
    std::size_t getByteSize() const;

    void calculateSizes();

    void addDimension(Dimension const& dim);

    std::vector<std::string> getDimensionNames() const;
    const std::list<Dimension> getDimensions() const
    {
        return m_dimensions;
    }

    boost::property_tree::ptree getPTree() const;

protected:
    std::list<Dimension> m_dimensions;
    std::size_t m_byteSize;

private:
};


LIBPC_DLL std::ostream& operator<<(std::ostream& os, Schema const&);


// BUG: move elsewhere, this is only here temporarily
class LIBPC_DLL LasSchema : public Schema
{
public:
    /// Versions of point record format.
    enum PointFormatName
    {
        ePointFormat0 = 0,  ///< Point Data Format \e 0
        ePointFormat1 = 1,  ///< Point Data Format \e 1    
        ePointFormat2 = 2,  ///< Point Data Format \e 2
        ePointFormat3 = 3,  ///< Point Data Format \e 3
        ePointFormat4 = 4,  ///< Point Data Format \e 3
        ePointFormat5 = 5,  ///< Point Data Format \e 3
        ePointFormatUnknown = -99 ///< Point Data Format is unknown
    };

public:
    LasSchema(PointFormatName data_format_id);
    LasSchema::LasSchema(LasSchema const& other);
    LasSchema& LasSchema::operator=(LasSchema const& rhs);

private:
    void add_record0_dimensions();
    void add_time();
    void add_color();
    void update_required_dimensions(PointFormatName data_format_id);

    PointFormatName m_data_format_id;
};



} // namespace liblas

#endif // LIBPC_SCHEMA_HPP_INCLUDED
