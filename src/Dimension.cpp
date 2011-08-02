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

#include <pdal/Dimension.hpp>

#include <pdal/exceptions.hpp>

namespace pdal
{

std::string Dimension::s_fieldNames[Field_LAST];
bool Dimension::s_fieldNamesValid = false;


Dimension::Dimension(Field field, DataType dataType)
    : m_dataType(dataType)
    , m_field(field)
    , m_endian(pdal::Endian_Little)
    , m_byteSize(0)
    , m_description(std::string(""))
    , m_min(0.0)
    , m_max(0.0)
    , m_precise(false)
    , m_numericScale(0.0)
    , m_numericOffset(0.0)
{
    m_byteSize = getDataTypeSize(m_dataType);
}

/// copy constructor
Dimension::Dimension(Dimension const& other) 
    : m_dataType(other.m_dataType)
    , m_field(other.m_field)
    , m_endian(other.m_endian)
    , m_byteSize(other.m_byteSize)
    , m_description(other.m_description)
    , m_min(other.m_min)
    , m_max(other.m_max)
    , m_precise(other.m_precise)
    , m_numericScale(other.m_numericScale)
    , m_numericOffset(other.m_numericOffset)
{
}

/// assignment operator
Dimension& Dimension::operator=(Dimension const& rhs)
{
    if (&rhs != this)
    {
        m_dataType = rhs.m_dataType;
        m_field = rhs.m_field;
        m_endian = rhs.m_endian;
        m_byteSize = rhs.m_byteSize;
        m_description = rhs.m_description;
        m_min = rhs.m_min;
        m_max = rhs.m_max;
        m_precise = rhs.m_precise;
        m_numericScale = rhs.m_numericScale;
        m_numericOffset = rhs.m_numericOffset;
    }

    return *this;
}


bool Dimension::operator==(const Dimension& other) const
{
    if (m_dataType == other.m_dataType &&
        m_field == other.m_field &&
        m_endian == other.m_endian &&
        m_byteSize == other.m_byteSize &&
        m_description == other.m_description &&
        Utils::compare_approx(m_min, other.m_min, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_max, other.m_max, (std::numeric_limits<double>::min)()) &&
        m_precise == other.m_precise &&

        Utils::compare_approx(m_numericScale, other.m_numericScale, (std::numeric_limits<double>::min)()) &&
        Utils::compare_approx(m_numericOffset, other.m_numericOffset, (std::numeric_limits<double>::min)()) 
        
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


boost::property_tree::ptree Dimension::GetPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getFieldName());
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
    return dim;
}


std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d)
{
    using boost::property_tree::ptree;
    ptree tree = d.GetPTree();

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
    case Undefined:
        return "Undefined";
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
    case Undefined:
        throw;
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
    case Undefined:
        throw;
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
    case Undefined:
        throw;
        
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
    if (s == "Float") return Float;
    if (s == "Double") return Double;
    throw;
}


std::string const& Dimension::getFieldName() const
{
    return getFieldName(m_field);
}


std::string const& Dimension::getFieldName(Field field)
{
    if (!s_fieldNamesValid)
        initFieldNames();

    if (field > Field_LAST)
        throw pdal_error("invalid field value (too large)");

    const std::string& s =  s_fieldNames[field];
    if (s.empty())
    {
        throw pdal_error("Field name not set for built-in field value");
    }        

    return s;
}


void Dimension::initFieldNames()
{
    for (int i=0; i<Dimension::Field_LAST; i++)
    {
        std::ostringstream ostr;
        ostr << "Unnamed field " << i;
        s_fieldNames[i] = ostr.str();
    }

    // BUG: not threadsafe
    s_fieldNames[Field_INVALID] = "invalid";
    s_fieldNames[Field_X] = "X";
    s_fieldNames[Field_Y] = "Y";
    s_fieldNames[Field_Z] = "Z";
    s_fieldNames[Field_Intensity] = "Intensity";
    s_fieldNames[Field_ReturnNumber] = "ReturnNumber";
    s_fieldNames[Field_NumberOfReturns] = "NumberOfReturns";
    s_fieldNames[Field_ScanDirectionFlag] = "ScanDirectionFlag";
    s_fieldNames[Field_EdgeOfFlightLine] = "EdgeOfFlightLine";
    s_fieldNames[Field_Classification] = "Classification";
    s_fieldNames[Field_ScanAngleRank] = "ScanAngleRank";
    s_fieldNames[Field_UserData] = "UserData";
    s_fieldNames[Field_PointSourceId] = "PointSourceId";
    s_fieldNames[Field_Time] = "Time";
    s_fieldNames[Field_Red] = "Red";
    s_fieldNames[Field_Green] = "Green";
    s_fieldNames[Field_Blue] = "Blue";
    s_fieldNames[Field_WavePacketDescriptorIndex] = "WavePacketDescriptorIndex";
    s_fieldNames[Field_WaveformDataOffset] = "WaveformDataOffset";
    s_fieldNames[Field_ReturnPointWaveformLocation] = "ReturnPointWaveformLocation";
    s_fieldNames[Field_WaveformXt] = "WaveformXt";
    s_fieldNames[Field_WaveformYt] = "WaveformYt";
    s_fieldNames[Field_WaveformZt] = "WaveformZt";
    s_fieldNames[Field_Alpha] = "Alpha";

    s_fieldNamesValid = true;
}


} // namespace pdal
