/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Dimension implementation for C++ libLAS
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

#ifndef PDAL_DIMENSIONLAYOUT_HPP_INCLUDED
#define PDAL_DIMENSIONLAYOUT_HPP_INCLUDED

#include <pdal/pdal.hpp>

#include <pdal/Dimension.hpp>


namespace pdal
{


class PDAL_DLL DimensionLayout
{
public:
    DimensionLayout(const Dimension&);
    DimensionLayout& operator=(DimensionLayout const& rhs);
    DimensionLayout(DimensionLayout const& other);

    bool operator==(const DimensionLayout& other) const;
    bool operator!=(const DimensionLayout& other) const;

    const Dimension& getDimension() const
    {
        return m_dimension;
    }

    /// The byte location to start reading/writing
    /// point data from in a composited schema.  liblas::Schema
    /// will set these values for you when liblas::Dimension are
    /// added to the liblas::Schema.
    inline std::size_t getByteOffset() const
    {
        return m_byteOffset;
    }

    inline void setByteOffset(std::size_t v)
    {
        m_byteOffset = v;
    }

    /// The index position of the index.  In a standard ePointFormat0
    /// data record, the X dimension would have a position of 0, while
    /// the Y dimension would have a position of 1, for example.
    inline std::size_t getPosition() const
    {
        return m_position;
    }

    inline void setPosition(std::size_t v)
    {
        m_position = v;
    }

    inline bool operator < (DimensionLayout const& dim) const 
    {
        return m_position < dim.m_position;
    }
    inline bool operator > (DimensionLayout const& dim) const 
    {
        return m_position > dim.m_position;
    }

private:
    Dimension m_dimension;
    std::size_t m_byteOffset;
    std::size_t m_position;
};


PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::DimensionLayout const& d);


} // namespace pdal

#endif // PDAL_DIMENSIONLAYOUT_HPP_INCLUDED
