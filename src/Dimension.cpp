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

#include <pdal/dimension/Dimension.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/uuid/string_generator.hpp>
#include <map>

namespace pdal
{


Dimension::Dimension(DimensionId::Id id)
    : m_dataType(Undefined)
    , m_id(id)
    , m_name(std::string(""))
    , m_flags(0)
    , m_endian(pdal::Endian_Little)
    , m_byteSize(0)
    , m_description(std::string(""))
    , m_min(0.0)
    , m_max(0.0)
    , m_numericScale(1.0)
    , m_numericOffset(0.0)
    , m_byteOffset(0)
    , m_position(0)
    , m_uuid(boost::uuids::random_generator()())
    , m_namespace(std::string(""))
{
    boost::uint32_t dt;
    DimensionId::lookupKnownDimension(id, dt, m_name, m_description);
    m_dataType = (DataType)dt;

    m_byteSize = getDataTypeSize(m_dataType);
    m_interpretation = getInterpretation(m_dataType);
}


Dimension::Dimension(DimensionId::Id id, DataType dataType, std::string name, std::string description)
    : m_dataType(dataType)
    , m_id(id)
    , m_name(name)
    , m_flags(0)
    , m_endian(pdal::Endian_Little)
    , m_byteSize(0)
    , m_description(description)
    , m_min(0.0)
    , m_max(0.0)
    , m_numericScale(1.0)
    , m_numericOffset(0.0)
    , m_byteOffset(0)
    , m_position(-1)
    , m_uuid(boost::uuids::random_generator()())
    , m_namespace(std::string(""))
{
    assert(!DimensionId::hasKnownDimension(id));
    
    m_byteSize = getDataTypeSize(m_dataType);
    m_interpretation = getInterpretation(dataType);
}

Dimension::Dimension(   std::string const& name, 
                        dimension::Interpretation interpretation,
                        dimension::size_type sizeInBytes,
                        std::string description)
    : m_name(name)
    , m_flags(0)
    , m_endian(pdal::Endian_Little)    
    , m_byteSize(sizeInBytes)
    , m_description(description)
    , m_min(0.0)
    , m_max(0.0)
    , m_numericScale(1.0)
    , m_numericOffset(0.0)
    , m_byteOffset(0)
    , m_position(-1)    
    , m_interpretation(interpretation)
    , m_uuid(boost::uuids::random_generator()())
    , m_namespace(std::string(""))
    
{
    m_id = DimensionId::getIdForDimension(*this);
    if (interpretation == dimension::UnsignedInteger)
    {
        if (sizeInBytes == 1)
            m_dataType = Uint8;
        if (sizeInBytes == 2)
            m_dataType = Uint16;
        if (sizeInBytes == 4)
            m_dataType = Uint32;
        if (sizeInBytes == 8)
            m_dataType = Uint64;
    }
    if (interpretation == dimension::SignedInteger)
    {
        if (sizeInBytes == 1)
            m_dataType = Int8;
        if (sizeInBytes == 2)
            m_dataType = Int16;
        if (sizeInBytes == 4)
            m_dataType = Int32;
        if (sizeInBytes == 8)
            m_dataType = Int64;
        
    }
    if (interpretation == dimension::Float)
    {
        if (sizeInBytes == 4)
            m_dataType = Float;
        if (sizeInBytes == 8)
            m_dataType = Double;
        
    }
}

/// copy constructor
Dimension::Dimension(Dimension const& other) 
    : m_dataType(other.m_dataType)
    , m_id(other.m_id)
    , m_name(other.m_name)
    , m_flags(other.m_flags)
    , m_endian(other.m_endian)
    , m_byteSize(other.m_byteSize)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_numericScale(other.m_numericScale)
    , m_numericOffset(other.m_numericOffset)
    , m_byteOffset(other.m_byteOffset)
    , m_position(other.m_position)
    , m_interpretation(other.m_interpretation)
    , m_uuid(other.m_uuid)
    , m_namespace(other.m_namespace)
{
    return;
}

/// assignment operator
Dimension& Dimension::operator=(Dimension const& rhs)
{
    if (&rhs != this)
    {
        m_dataType = rhs.m_dataType;
        m_id = rhs.m_id;
        m_name = rhs.m_name;
        m_flags = rhs.m_flags;
        m_endian = rhs.m_endian;
        m_byteSize = rhs.m_byteSize;
        m_description = rhs.m_description;
        m_min = rhs.m_min;
        m_max = rhs.m_max;
        m_numericScale = rhs.m_numericScale;
        m_numericOffset = rhs.m_numericOffset;
        m_byteOffset = rhs.m_byteOffset;
        m_position = rhs.m_position;
        m_interpretation = rhs.m_interpretation;
        m_uuid = rhs.m_uuid;
        m_namespace = rhs.m_namespace;
    }

    return *this;
}


bool Dimension::operator==(const Dimension& other) const
{
    if (m_dataType == other.m_dataType &&
        m_id == other.m_id &&
        boost::iequals(m_name, other.m_name) &&
        m_flags == other.m_flags &&
        m_endian == other.m_endian &&
        m_byteSize == other.m_byteSize &&
        m_description == other.m_description &&
        Utils::compare_approx(m_min, other.m_min, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_max, other.m_max, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_numericScale, other.m_numericScale, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_numericOffset, other.m_numericOffset, (std::numeric_limits<double>::min)()) &&
        m_byteOffset == other.m_byteOffset &&
        m_position == other.m_position &&
        m_interpretation == other.m_interpretation && 
        m_uuid == other.m_uuid
        )
    {
        return true;
    }

    return false;
}


bool Dimension::operator!=(const Dimension& other) const
{
  return !(*this==other);
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
    case Pointer:
        return "Pointer";
    case Int64:
        return "Int64";
    case Uint64:
        return "Uint64";
    case Float:
        return "Float";
    case Double:
        return "Double";
    case Undefined:
        return "Undefined";
    }
    throw;
}

dimension::Interpretation Dimension::getInterpretation(DataType type)
{
    switch (type)
    {
    case Int8:
        return dimension::SignedByte;
    case Uint8:
        return dimension::UnsignedByte;
    case Uint16:
    case Uint32:
    case Uint64:
        return dimension::UnsignedInteger;
    case Pointer:
        return dimension::Pointer;
    case Int16:
    case Int32:
    case Int64:
        return dimension::SignedInteger;
    case Float:
    case Double:
        return dimension::Float;
    case Undefined:
        return dimension::Undefined;
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
    case Pointer:
        return sizeof(void*);
    case Int64:
        return 8;
    case Uint64:
        return 8;
    case Float:
        return 4;
    case Double:
        return 8;
    case Undefined:
        throw;
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
    if (s == "Pointer") return Pointer;
    if (s == "Float") return Float;
    if (s == "Double") return Double;
    throw;
}


std::string const& Dimension::getName() const
{
    return m_name;
}





boost::property_tree::ptree Dimension::toPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    dim.put("datatype", getDataTypeName(getDataType()));
    dim.put("description", getDescription());
    dim.put("bytesize", getByteSize());
    
    std::string e("little");
    if (getEndianness() == Endian_Big) 
        e = std::string("big");
    dim.put("endianness", e);


    if (! (Utils::compare_distance(getMinimum(), getMaximum()) && 
           Utils::compare_distance(0.0, getMaximum())))
    {
        dim.put("minimum", getMinimum());
        dim.put("maximum", getMaximum());
    }
    if (! (Utils::compare_distance(getNumericScale(), 0.0)))
    {
        dim.put("scale", getNumericScale());
    }
    if (! (Utils::compare_distance(getNumericOffset(), 0.0)))
    {
        dim.put("offset", getNumericOffset());
    }
    
    dim.put("scale", getNumericScale());

    dim.put("isValid", isValid());

    return dim;
}

void Dimension::setUUID(std::string const& id)
{
    boost::uuids::string_generator gen;
    m_uuid = gen(id);
}

void Dimension::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d)
{
    using boost::property_tree::ptree;
    ptree tree = d.toPTree();

    std::string const name = tree.get<std::string>("name");

    std::ostringstream quoted_name;
    quoted_name << "'" << name << "'";
    std::ostringstream pad;
    std::string const& cur = quoted_name.str();
    std::string::size_type size = cur.size();
    std::string::size_type pad_size = 24 - size;

    for (std::string::size_type i=0; i != pad_size; i++ )
    {
        pad << " ";
    }
    os << quoted_name.str() << pad.str() <<" -- "<< " size: " << tree.get<boost::uint32_t>("bytesize");

    try {
        double value = tree.get<double>("scale");
        boost::uint32_t precision = Utils::getStreamPrecision(value);
        os.setf(std::ios_base::fixed, std::ios_base::floatfield);
        os.precision(precision);
        os << " scale: " << value;
    }
    catch (boost::property_tree::ptree_bad_path const& ) {
    }    

    try {
        double value = tree.get<double>("offset");
        boost::uint32_t precision = Utils::getStreamPrecision(value);
        os.setf(std::ios_base::fixed, std::ios_base::floatfield);
        os.precision(precision);
        os << " offset: " << value;
    }
    catch (boost::property_tree::ptree_bad_path const& ) {
    }    
    
    //os << " offset: " << tree.get<boost::uint32_t>("byteoffset");
    os << std::endl;

    return os;
}


} // namespace pdal
