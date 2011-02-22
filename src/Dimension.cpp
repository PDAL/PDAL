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


Dimension::Dimension(std::string const& name, DataType dataType)
    : m_dataType(dataType)
    , m_name(name)
    , m_description(std::string(""))
    , m_min(0)
    , m_max(0)
    , m_position(0)
    , m_byteSize(0)
    , m_byteOffset(0)
{
    m_byteSize = getDataTypeSize(m_dataType);
}

/// copy constructor
Dimension::Dimension(Dimension const& other) 
    : m_name(other.m_name)
    , m_dataType(other.m_dataType)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_position(other.m_position)
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
        return true;
    }

    return false;
}


bool Dimension::operator!=(const Dimension& other) const
{
  return !(*this==other);
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
    case Int8:
        return "Int8";
    case Uint8:
        return "Uint8";
    case Int16:
        return "Int16";
    case Uint16:
        return "Uint16";
    case Int32:
        return "Int32";
    case Uint32:
        return "Uint32";
    case Int64:
        return "Int64";
    case Uint64:
        return "Uint64";
    case Float:
        return "Float";
    case Double:
        return "Double";
    }
    throw;
}


std::size_t Dimension::getDataTypeSize(DataType type)
{
    switch (type)
    {
    case Int8:
        return 1;
    case Uint8:
        return 1;
    case Int16:
        return 2;
    case Uint16:
        return 2;
    case Int32:
        return 4;
    case Uint32:
        return 4;
    case Int64:
        return 8;
    case Uint64:
        return 8;
    case Float:
        return 4;
    case Double:
        return 8;
    }
    throw;
}


bool Dimension::getDataTypeIsNumeric(DataType type)
{
    switch (type)
    {
    case Int8:
    case Uint8:
    case Int16:
    case Uint16:
    case Int32:
    case Uint32:
    case Int64:
    case Uint64:
        return true;
    case Float:
    case Double:
        return true;
    }
    throw;
}


bool Dimension::getDataTypeIsSigned(DataType type)
{
    switch (type)
    {
    case Uint8:
    case Uint16:
    case Uint32:
    case Uint64:
        return false;
    case Int8:
    case Int16:
    case Int32:
    case Int64:
        return true;
    case Float:
    case Double:
        return true;
    }
    throw;
}


bool Dimension::getDataTypeIsInteger(DataType type)
{
    switch (type)
    {
    case Uint8:
    case Uint16:
    case Uint32:
    case Uint64:
        return true;
    case Int8:
    case Int16:
    case Int32:
    case Int64:
        return true;
    case Float:
    case Double:
        return false;
    }
    throw;
}


Dimension::DataType Dimension::getDataTypeFromString(const std::string& s)
{
    if (s == "Int8") return Int8;
    if (s == "Uint8") return Uint8;
    if (s == "Int16") return Int16;
    if (s == "Uint16") return Uint16;
    if (s == "Int32") return Int32;
    if (s == "Uint32") return Uint32;
    if (s == "Int64") return Int64;
    if (s == "Uint64") return Uint64;
    if (s == "Float") return Float;
    if (s == "Double") return Double;
    throw;
}

} // namespace libpc
