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

#include "libpc/Dimension.hpp"
#include "libpc/Utils.hpp"

#include <iostream>

using namespace boost;

namespace libpc
{


Dimension::Dimension(std::string const& name, std::size_t size_in_bits) :
    m_name(name),
    m_bit_size(size_in_bits),
    m_required(false),
    m_active(false),
    m_description(std::string("")),
    m_min(0),
    m_max(0),
    m_numeric(false),
    m_signed(false),
    m_integer(false),
    m_position(0),
    m_byte_offset(0),
    m_bit_offset(0)
{
    if (0 == size_in_bits)
    {
        throw std::runtime_error("The bit size of the dimension is 0, the dimension is invalid.");
    }
}

/// copy constructor
Dimension::Dimension(Dimension const& other) :
    m_name(other.m_name)
    , m_bit_size(other.m_bit_size)
    , m_required(other.m_required)
    , m_active(other.m_active)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_numeric(other.m_numeric)
    , m_signed(other.m_signed)
    , m_integer(other.m_integer)
    , m_position(other.m_position)
    , m_byte_offset(other.m_byte_offset)
    , m_bit_offset(other.m_bit_offset)
{
}

/// assignment operator
Dimension& Dimension::operator=(Dimension const& rhs)
{
    if (&rhs != this)
    {
        m_name = rhs.m_name;
        m_bit_size = rhs.m_bit_size;
        m_required = rhs.m_required;
        m_active = rhs.m_active;
        m_description = rhs.m_description;
        m_min = rhs.m_min;
        m_max = rhs.m_max;
        m_numeric = rhs.m_numeric;
        m_signed = rhs.m_signed;
        m_integer = rhs.m_integer;
        m_position = rhs.m_position;
        m_byte_offset = rhs.m_byte_offset;
        m_bit_offset = rhs.m_bit_offset;
    }

    return *this;
}

std::size_t Dimension::GetByteSize() const
{
    std::size_t const bit_position = m_bit_size % 8;
    if (bit_position > 0)
    {
        // For dimensions that are not byte aligned,
        // we need to determine how many bytes they
        // will take.  We have to read at least one byte if the
        // size in bits is less than 8.  If it is more than 8,
        // we need to read the number of bytes it takes + 1 extra.
        if (m_bit_size > 8)
        {
            return m_bit_size/8 + 1;
        }
        else
        {
            return 1;
        }
    }
    return m_bit_size / 8;
}

property_tree::ptree Dimension::GetPTree() const
{
    using property_tree::ptree;
    ptree dim;
    dim.put("name", GetName());
    dim.put("description", GetDescription());
    dim.put("position", GetPosition());
    dim.put("active", static_cast<boost::uint32_t>(IsActive()));
    dim.put("size", GetBitSize());
    dim.put("integer", static_cast<boost::uint32_t>(IsInteger()));
    dim.put("signed", static_cast<boost::uint32_t>(IsSigned()));
    dim.put("required", static_cast<boost::uint32_t>(IsRequired()));
    dim.put("byteoffset", GetByteOffset());
    dim.put("bitoffset" , GetBitOffset());
    dim.put("bytesize", GetByteSize());

    if (IsNumeric())
    {
        if (! (Utils::compare_distance(GetMinimum(), GetMaximum() )
                && Utils::compare_distance(0.0, GetMaximum())))
        {
            dim.put("minimum", GetMinimum());
            dim.put("maximum", GetMaximum());
        }
    }

    return dim;
}


std::ostream& operator<<(std::ostream& os, libpc::Dimension const& d)
{
    using boost::property_tree::ptree;
    ptree tree = d.GetPTree();

    std::string const name = tree.get<std::string>("name");

    std::ostringstream quoted_name;
    quoted_name << "'" << name << "'";
    std::ostringstream pad;
    std::string const& cur = quoted_name.str();
    std::string::size_type size = cur.size();
    std::string::size_type pad_size = 30 - size;

    for (std::string::size_type i=0; i != pad_size; i++ )
    {
        pad << " ";
    }
    os << quoted_name.str() << pad.str() <<" -- "<< " size: " << tree.get<boost::uint32_t>("size");
    os << " offset: " << tree.get<boost::uint32_t>("byteoffset");
    os << std::endl;

    return os;
}

} // namespace libpc
