/******************************************************************************
 * Copyright (c) 2016, Hobu Inc.
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

#pragma once

#include <string>
#include <vector>

#include <pdal/util/Utils.hpp>

namespace pdal
{
namespace Dimension
{

enum class BaseType
{
    None = 0x000,
    Signed = 0x100,
    Unsigned = 0x200,
    Floating = 0x400
};

inline BaseType fromName(std::string name)
{
    if (name == "signed")
        return BaseType::Signed;
    else if (name == "unsigned")
        return BaseType::Unsigned;
    else if (name == "floating" || name == "float")
        return BaseType::Floating;
    return BaseType::None;
}

inline std::string toName(BaseType b)
{
    switch (b)
    {
    case BaseType::Signed:
        return "signed";
    case BaseType::Unsigned:
        return "unsigned";
    case BaseType::Floating:
        return "floating";
    default:
        return "";
    }
}

enum class Type
{
    None = 0,
    Unsigned8 = unsigned(BaseType::Unsigned) | 1,
    Signed8 = unsigned(BaseType::Signed) | 1,
    Unsigned16 = unsigned(BaseType::Unsigned) | 2,
    Signed16 = unsigned(BaseType::Signed) | 2,
    Unsigned32 = unsigned(BaseType::Unsigned) | 4,
    Signed32 = unsigned(BaseType::Signed) | 4,
    Unsigned64 = unsigned(BaseType::Unsigned) | 8,
    Signed64 = unsigned(BaseType::Signed) | 8,
    Float = unsigned(BaseType::Floating) | 4,
    Double = unsigned(BaseType::Floating) | 8
};

inline std::size_t size(Type t)
{
    return Utils::toNative(t) & 0xFF;
}

inline BaseType base(Type t)
{
    return BaseType(Utils::toNative(t) & 0xFF00);
}

static const int COUNT = 1024;
static const int PROPRIETARY = 512;

/// Get a string reresentation of a datatype.
/// \param[in] dimtype  Dimension type.
/// \return  String representation of dimension type.
inline std::string interpretationName(Type dimtype)
{
    switch (dimtype)
    {
    case Type::None:
        return "unknown";
    case Type::Signed8:
        return "int8_t";
    case Type::Signed16:
        return "int16_t";
    case Type::Signed32:
        return "int32_t";
    case Type::Signed64:
        return "int64_t";
    case Type::Unsigned8:
        return "uint8_t";
    case Type::Unsigned16:
        return "uint16_t";
    case Type::Unsigned32:
        return "uint32_t";
    case Type::Unsigned64:
        return "uint64_t";
    case Type::Float:
        return "float";
    case Type::Double:
        return "double";
    }
    return "unknown";
}


/// Get the type corresponding to a type name.
/// \param s  Name of type.
/// \return  Corresponding type enumeration value.
inline Type type(std::string s)
{
    s = Utils::tolower(s);

    if (s == "int8_t" || s == "int8" || s == "char")
       return Type::Signed8;
    if (s == "int16_t" || s == "int16" || s == "short")
       return Type::Signed16;
    if (s == "int32_t" || s == "int32" || s == "int")
       return Type::Signed32;
    if (s == "int64_t" || s == "int64" || s == "long")
       return Type::Signed64;
    if (s == "uint8_t" || s == "uint8" || s == "uchar")
        return Type::Unsigned8;
    if (s == "uint16_t" || s == "uint16" || s == "ushort")
        return Type::Unsigned16;
    if (s == "uint32_t" || s == "uint32" || s == "uint")
        return Type::Unsigned32;
    if (s == "uint64_t" || s == "uint64" || s == "ulong")
        return Type::Unsigned64;
    if (s == "float" || s == "float32")
        return Type::Float;
    if (s == "double" || s == "float64")
        return Type::Double;
    return Type::None;
}

inline Type type(const std::string& baseType, size_t size)
{
    BaseType base = fromName(baseType);
    if (base == BaseType::None)
        return Type::None;
    if (size != 1 && size != 2 && size != 4 && size != 8)
        return Type::None;
    if ((size == 1 || size == 2) && base == BaseType::Floating)
        return Type::None;

    return static_cast<Type>((size_t)(base) | size);
}

/// Extract a dimension name of a string.  Dimension names start with an alpha
/// and continue with numbers or underscores.
/// \param s  String from which to extract dimension name.
/// \param p  Position at which to start extracting.
/// \return  Number of characters in the extracted name.
inline std::size_t extractName(const std::string& s, std::string::size_type p)
{
    if (!std::isalpha(s[p++]))
        return 0;
    auto isvalid = [](int c)
    {
        return std::isalpha(c) || std::isdigit(c) || c == '_';
    };
    return Utils::extract(s, p, isvalid) + 1;
}

inline std::string fixName(std::string name)
{
    size_t pos = 0;
    while (true)
    {
        pos = extractName(name, 0);
        if (pos == name.size())
            break;
        name[pos] = '_';
    }
    return name;
}

inline bool nameValid(std::string name)
{
    name = Utils::toupper(name);
    if (extractName(name, 0) != name.size())
        return false;
    if (name == "WHERE")
        return false;
    return true;
}

inline std::istream& operator>>(std::istream& in, Dimension::Type& type)
{
    std::string sval;

    in >> sval;
    type = Dimension::type(sval);
    if (type == Dimension::Type::None)
        in.setstate(std::ios_base::failbit);
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const Dimension::Type& type)
{
    out << Dimension::interpretationName(type);
    return out;
}

} // namespace Dimension

} // namespace pdal

