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


Dimension::Dimension(std::string const& name, DataType dataType, std::size_t size_in_bits)
    : m_dataType(dataType)
    , m_name(name)
    , m_description(std::string(""))
    , m_min(0)
    , m_max(0)
    , m_position(0)
    , m_bitSize(size_in_bits)
    , m_byteSize(0)
    , m_byteOffset(0)
{
    if (m_dataType == bits_t)
    {
        assert(size_in_bits > 0);
        m_byteSize = (size_in_bits % 8 == 0) ? (size_in_bits/8) : (size_in_bits/8 + 1);
    }
    else
    {
        assert(size_in_bits == 0);
        m_byteSize = getDataTypeSize(m_dataType);
    }

    //if (0 == size_in_bits)
    //{
    //    throw std::runtime_error("The bit size of the dimension is 0, the dimension is invalid.");
    //}
}

/// copy constructor
Dimension::Dimension(Dimension const& other) 
    : m_name(other.m_name)
    , m_dataType(other.m_dataType)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_position(other.m_position)
    , m_bitSize(other.m_bitSize)
    , m_byteSize(other.m_byteSize)
    , m_byteOffset(other.m_byteOffset)
{
}

/// assignment operator
Dimension& Dimension::operator=(Dimension const& rhs)
{
    if (&rhs != this)
    {
        m_name = rhs.m_name;
        m_dataType = rhs.m_dataType;
        m_description = rhs.m_description;
        m_min = rhs.m_min;
        m_max = rhs.m_max;
        m_position = rhs.m_position;
        m_byteSize = rhs.m_byteSize;
        m_byteOffset = rhs.m_byteOffset;
        m_bitSize = rhs.m_bitSize;
    }

    return *this;
}


bool Dimension::operator==(const Dimension& other) const
{
    if (m_name == other.m_name)
    {
        assert(m_dataType == other.m_dataType);
        assert(m_description == other.m_description);
        assert(m_min == other.m_min);
        assert(m_max == other.m_max);
        assert(m_position == other.m_position);
        assert(m_precise == other.m_precise);
        assert(m_numericScale == other.m_numericScale);
        assert(m_numericOffset == other.m_numericOffset);
        assert(m_byteSize == other.m_byteSize);
        assert(m_byteOffset == other.m_byteOffset);
        assert(m_bitSize == other.m_bitSize);
        return true;
    }

    return false;
}



property_tree::ptree Dimension::GetPTree() const
{
    using property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    dim.put("datatype", getDataTypeName(getDataType()));
    dim.put("description", getDescription());
    dim.put("position", getPosition());
    dim.put("byteoffset", getByteOffset());
    dim.put("bytesize", getByteSize());
    dim.put("bitsize", getBitSize());

    if (isNumeric())
    {
        if (! (Utils::compare_distance(getMinimum(), getMaximum()) && 
               Utils::compare_distance(0.0, getMaximum())))
        {
            dim.put("minimum", getMinimum());
            dim.put("maximum", getMaximum());
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
    os << quoted_name.str() << pad.str() <<" -- "<< " size: " << tree.get<boost::uint32_t>("bytesize");
    os << " offset: " << tree.get<boost::uint32_t>("byteoffset");
    os << std::endl;

    return os;
}

std::string Dimension::getDataTypeName(DataType type)
{
    switch (type)
    {
    case int8_t:
        return "int8_t";
    case uint8_t:
        return "uint8_t";
    case int16_t:
        return "int16_t";
    case uint16_t:
        return "uint16_t";
    case int32_t:
        return "int32_t";
    case uint32_t:
        return "uint32_t";
    case int64_t:
        return "int64_t";
    case uint64_t:
        return "uint64_t";
    case float_t:
        return "float_t";
    case double_t:
        return "double_t";
    case bits_t:
        return "bits_t";
    }
    throw;
}


std::size_t Dimension::getDataTypeSize(DataType type)
{
    switch (type)
    {
   case int8_t:
        return 1;
    case uint8_t:
        return 1;
    case int16_t:
        return 2;
    case uint16_t:
        return 2;
    case int32_t:
        return 4;
    case uint32_t:
        return 4;
    case int64_t:
        return 8;
    case uint64_t:
        return 8;
    case float_t:
        return 4;
    case double_t:
        return 8;
   case bits_t:
        throw;
        //return 0;
    }
    throw;
}


bool Dimension::getDataTypeIsNumeric(DataType type)
{
    switch (type)
    {
    case int8_t:
    case uint8_t:
    case int16_t:
    case uint16_t:
    case int32_t:
    case uint32_t:
    case int64_t:
    case uint64_t:
        return true;
    case float_t:
    case double_t:
        return true;
    case bits_t:
        return false;
    }
    throw;
}


bool Dimension::getDataTypeIsSigned(DataType type)
{
    switch (type)
    {
    case uint8_t:
    case uint16_t:
    case uint32_t:
    case uint64_t:
        return false;
    case int8_t:
    case int16_t:
    case int32_t:
    case int64_t:
        return true;
    case float_t:
    case double_t:
        return true;
    case bits_t:
        return false;
    }
    throw;
}


bool Dimension::getDataTypeIsInteger(DataType type)
{
    switch (type)
    {
    case uint8_t:
    case uint16_t:
    case uint32_t:
    case uint64_t:
        return true;
    case int8_t:
    case int16_t:
    case int32_t:
    case int64_t:
        return true;
    case float_t:
    case double_t:
        return false;
    case bits_t:
        return false;
    }
    throw;
}


Dimension::DataType Dimension::getDataTypeFromString(const std::string& s)
{
    if (s == "int8_t") return int8_t;
    if (s == "uint8_t") return uint8_t;
    if (s == "int16_t") return int16_t;
    if (s == "uint16_t") return uint16_t;
    if (s == "int32_t") return int32_t;
    if (s == "uint32_t") return uint32_t;
    if (s == "int64_t") return int64_t;
    if (s == "uint64_t") return uint64_t;
    if (s == "float_t") return float_t;
    if (s == "double_t") return double_t;
    if (s == "bits_t") return bits_t;
    throw;
}

} // namespace libpc
