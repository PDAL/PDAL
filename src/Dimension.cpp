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

#include <pdal/GlobalEnvironment.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <map>

#include <time.h>
#include <cstdlib>

namespace pdal
{



Dimension::Dimension(std::string const& name,
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
    , m_namespace(std::string(""))
    , m_parentDimensionID(boost::uuids::nil_uuid())

{
    GlobalEnvironment& env = pdal::GlobalEnvironment::get();
    boost::uuids::basic_random_generator<boost::mt19937> gen(env.getRNG());
    m_uuid = gen();


    if (!m_name.size())
    {
        // Generate a random name from the time
        ::time_t seconds;
        ::time(&seconds);

        std::ostringstream oss;
        srand(static_cast<unsigned int>(seconds));
        oss << "unnamed" << rand();
        m_name = oss.str();

    }

}


bool Dimension::operator==(const Dimension& other) const
{
    return (boost::iequals(m_name, other.m_name) &&
        m_flags == other.m_flags &&
        m_endian == other.m_endian &&
        m_byteSize == other.m_byteSize &&
        boost::iequals(m_description, other.m_description) &&
        Utils::compare_approx(m_min, other.m_min,
            std::numeric_limits<double>::min()) &&
        Utils::compare_approx(m_max, other.m_max,
            std::numeric_limits<double>::min()) &&
        Utils::compare_approx(m_numericScale, other.m_numericScale,
            std::numeric_limits<double>::min()) &&
        Utils::compare_approx(m_numericOffset, other.m_numericOffset,
            std::numeric_limits<double>::min()) &&
        m_byteOffset == other.m_byteOffset &&
        m_position == other.m_position &&
        m_interpretation == other.m_interpretation &&
        m_uuid == other.m_uuid &&
        m_parentDimensionID == other.m_parentDimensionID);
}


bool Dimension::operator!=(const Dimension& other) const
{
    return !(*this==other);
}

boost::property_tree::ptree Dimension::toPTree() const
{
    using boost::property_tree::ptree;
    ptree dim;
    dim.put("name", getName());
    dim.put("namespace", getNamespace());
    dim.put("parent", getParent());
    dim.put("description", getDescription());
    dim.put("bytesize", getByteSize());

    std::string e("little");
    if (getEndianness() == Endian_Big)
        e = std::string("big");
    dim.put("endianness", e);

    dim.put("minimum", getMinimum());
    dim.put("maximum", getMaximum());
    dim.put("scale", getNumericScale());
    dim.put("offset", getNumericOffset());

    dim.put("scale", getNumericScale());

    dim.put("position", getPosition());
    dim.put("byteoffset", getByteOffset());
    dim.put("isIgnored", isIgnored());
    dim.put("interpretation", getInterpretationName());
    
    std::stringstream oss;

    dimension::id t =  getUUID();
    oss << t;

    dim.put("uuid", oss.str());
    return dim;
}

void Dimension::setUUID(std::string const& id)
{
    boost::uuids::string_generator gen;
    m_uuid = gen(id);
}

void Dimension::createUUID()
{
    // Global RNG
    GlobalEnvironment& env = pdal::GlobalEnvironment::get();
    boost::uuids::basic_random_generator<boost::mt19937> gen(env.getRNG());
    m_uuid = gen();

    // Stack-allocated, uninitialized RNG
    // boost::mt19937 ran;
    // boost::uuids::basic_random_generator<boost::mt19937> gen(&ran);
    // m_uuid = gen();

    // Single call, uninitialized RNG
    // m_uuid = boost::uuids::random_generator()();
}

std::string Dimension::getInterpretationName() const
{
    std::ostringstream type;
    dimension::Interpretation t = getInterpretation();
    boost::uint32_t bytesize = getByteSize();

    switch (t)
    {
        case dimension::RawByte:
            if (bytesize == 1)
                type << "uint8_t";
            break;

        case dimension::SignedInteger:
            if (bytesize == 1)
                type << "int8_t";
            else if (bytesize == 2)
                type << "int16_t";
            else if (bytesize == 4)
                type << "int32_t";
            else if (bytesize == 8)
                type << "int64_t";
            else
                type << "unknown";
            break;
        case dimension::UnsignedInteger:
            if (bytesize == 1)
                type << "uint8_t";
            else if (bytesize == 2)
                type << "uint16_t";
            else if (bytesize == 4)
                type << "uint32_t";
            else if (bytesize == 8)
                type << "uint64_t";
            else
                type << "unknown";
            break;

        case dimension::Float:
            if (bytesize == 4)
                type << "float";
            else if (bytesize == 8)
                type << "double";
            else
                type << "unknown";
            break;

        case dimension::Pointer:
            type << "pointer";
            break;
        case dimension::Undefined:
            type << "unknown";
            break;
    }

    return type.str();
}


dimension::Interpretation
Dimension::getInterpretation(std::string const& interpretation)
{

    if (boost::iequals(interpretation, "int8_t") ||
            boost::iequals(interpretation, "int8"))
        return dimension::SignedInteger;

    if (boost::iequals(interpretation, "uint8_t") ||
            boost::iequals(interpretation, "uint8"))
        return dimension::UnsignedInteger;

    if (boost::iequals(interpretation, "int16_t") ||
            boost::iequals(interpretation, "int16"))
        return dimension::SignedInteger;

    if (boost::iequals(interpretation, "uint16_t") ||
            boost::iequals(interpretation, "uint16"))
        return dimension::UnsignedInteger;


    if (boost::iequals(interpretation, "int32_t") ||
            boost::iequals(interpretation, "int32"))
        return dimension::SignedInteger;

    if (boost::iequals(interpretation, "uint32_t") ||
            boost::iequals(interpretation, "uint32"))
        return dimension::UnsignedInteger;

    if (boost::iequals(interpretation, "int64_t") ||
            boost::iequals(interpretation, "int64"))
        return dimension::SignedInteger;

    if (boost::iequals(interpretation, "uint64_t") ||
            boost::iequals(interpretation, "uint64"))
        return dimension::UnsignedInteger;

    if (boost::iequals(interpretation, "float"))
        return dimension::Float;

    if (boost::iequals(interpretation, "double"))
        return dimension::Float;

    return dimension::Undefined;
}

std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d)
{
    using boost::property_tree::ptree;
    ptree tree = d.toPTree();

    std::string const name = tree.get<std::string>("name");
    std::string const ns = tree.get<std::string>("namespace");

    std::ostringstream quoted_name;
    if (ns.size())
        quoted_name << "'" << ns << "." << name << "'";
    else
        quoted_name << "'" << name << "'";
    std::ostringstream pad;
    std::string const& cur = quoted_name.str();
    std::string::size_type size = cur.size();
    std::string::size_type buffer(24);
    std::string::size_type pad_size = size > buffer ? 4 : buffer - size;
    os << quoted_name.str() << std::string(pad_size, ' ') << " -- "<<
        " size: " << tree.get<boost::uint32_t>("bytesize");

    double scale = tree.get<double>("scale");
    os.setf(std::ios_base::fixed, std::ios_base::floatfield);
    os.precision(14);
    os << " scale: " << scale;

    double offset = tree.get<double>("offset");
    os << " offset: " << offset;
    os << " ignored: " << tree.get<bool>("isIgnored");
    os << " uid: " << tree.get<std::string>("uuid");
    os << " parent: " << tree.get<std::string>("parent");
    os << std::endl;

    return os;
}


} // namespace pdal
