/******************************************************************************
* Copyright (c) 2012, Howard Butler hobu.inc@gmail.com
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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

#ifndef INCLUDED_METADATA_HPP
#define INCLUDED_METADATA_HPP

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>


#include <boost/shared_array.hpp>
#include <boost/variant.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <boost/blank.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <boost/optional.hpp>

#include <boost/algorithm/string.hpp>


#include <vector>
#include <map>

namespace pdal
{

/// ByteArray simply wrapps a std::vector<boost::uint8_t> such that it can then
/// be dumped to an ostream in a base64 encoding. For now, it makes a copy of the data
/// it is given, and should not be used for slinging big data around.
class PDAL_DLL ByteArray
{
public:

    /** @name Constructors
    */
    /// Constructs a ByteArray instance with the given array of data.
    ByteArray(std::vector<boost::uint8_t> const& data)
        : m_bytes(data)
    {
    }
    
    ByteArray()
    {
    }

    /// Copy constructor
    ByteArray(const ByteArray& rhs)
        : m_bytes(rhs.m_bytes)

    {
    }

    /// Assignment operator
    inline ByteArray& operator=(ByteArray const& rhs)
    {
        if (&rhs != this)
        {
            m_bytes = rhs.m_bytes;
        }
        return *this;
    }

    /** @name Data manipulation
    */
    /// resets the array
    inline void set(std::vector<boost::uint8_t> const& input)
    {
        m_bytes = input;
    }

    /// fetches a reference to the array
    inline std::vector<boost::uint8_t> const& get() const
    {
        return m_bytes;
    }


    /** @name private attributes
    */
private:

    std::vector<boost::uint8_t> m_bytes;
};

namespace metadata
{

typedef boost::uuids::uuid id;

enum Type
{
    /// boolean
    Boolean = 0,
    /// equivalent to int32_t
    SignedInteger,
    /// equivalent to uint32_t
    UnsignedInteger,
    /// equivalent to float
    Float,
    /// equivalent to double
    Double,
    /// equivalent to std::string
    String,
    /// raw binary data in the form std::vector<boost::uint8_t>
    Bytes,
    /// A pdal::Bounds instance
    Bounds,
    /// A pdal::SpatialReference instance
    SpatialReference,
    /// A boost::uuids::uuid instance
    UUID,
    /// A pdal::Metadata instance
    MData,
    /// Nothing
    Blank
};



}


/// metadata::Entry is a container for metadata entries that pdal::Stage and
/// pdal::PointBuffer carry around as part of their internal operations. Bits of
/// information might come from a pdal::Reader that opens a file, or a
/// pdal::Filter that processes as an intermediate stage, and
/// pdal::metadata::Entry is what is used to hold and pass those metadata
/// around.  pdal::metadata::Entry values must be of type
/// pdal::metadata::Variant and it is required that they are serializeable to
/// std::string.

/// pdal::metadata::Entry instances also carry with them a map of key/value
/// pairs called attributes that are metadata about the metadata entry. For
/// example, a LAS VLR might have a name of "classification", a pdal::ByteArray
/// for its data, and a set of attributes that are "userid":"4321" and
/// "vlrid":"1234". These other metadata may be useful given the context, and
/// any auxiliary data about the metadata entry should be provided via
/// attributes. It is up to you to determine where the line of attribute and new
/// metadata entry exists.
class PDAL_DLL Metadata
{
public:

    /** @name Constructors
    */
    /// Base constructor
    /// @param name entry name to use for this metadata entry
    Metadata();
    Metadata(std::string const& name);
    Metadata(Metadata const& other);

    template <typename T>
    Metadata(std::string const& name, const T& value, std::string const& description="")
    {
        setName(name);
        setDescription(description);
        setValue<T>(value);
    }
    
    template <typename T>
    void addMetadata(std::string const& name, const T& value, std::string const& description="");
    
    void addMetadata(  Metadata const& m); 

    inline boost::property_tree::ptree toPTree() const 
    {
        return m_tree; 
    } 

    /** @name entry type
    */
    /// returns the pdal::metadata::Type for the metadata entry
    inline metadata::Type getType() const
    {
        return static_cast<metadata::Type>(m_tree.get<boost::uint32_t>("type"));
    }

    /// sets the pdal::metadata::Type for the metadata entry
    /// @param t pdal::metadata::Type value for the entry
    inline void setType(metadata::Type t)
    {
        m_tree.put<boost::uint32_t>("type", static_cast<boost::uint32_t>(t));
        m_tree.put<std::string>("typename", getTypeName());
    }

    /// returns a std::string representation of the type
    std::string getTypeName() const;
    
    /** @name entry value
    */
    /// @param v value of type pdal::metadata::Variant to set for the entry
    template <class T> inline void setValue(T const& v);
    template <class T> inline T getValue() const { return m_tree.get<T>("value"); } 

    /** @name entry name
    */
    /// returns the name for the metadata entry
    inline std::string getName() const
    {
        return m_tree.get<std::string>("name");
    }

    /// resets the name for the metadata entry
    /// @param name value to use for new name
    inline void setName(std::string const& name)
    {
        m_tree.put("name", name);
    }

    /** @name description
    */
    /// Overwrites the description given in the constructor
    /// @param description new value to use for the description of the Option
    inline void setDescription(const std::string& description)
    {
        m_tree.put("description", description);
    }

    /// @return the description of the Option
    inline std::string getDescription() const
    {
        return m_tree.get<std::string>("description");
    }

private:
    boost::property_tree::ptree m_tree;
    
};


template <>
inline void Metadata::setValue<bool>(bool const& v)
{
    setType(metadata::Boolean);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<std::string>(std::string const& v)
{
    setType(metadata::String);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::ByteArray>(pdal::ByteArray const& v)
{
    setType(metadata::Bytes);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<float>(float const& v)
{
    setType(metadata::Float);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<double>(double const& v)
{
    setType(metadata::Double);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::SpatialReference>(pdal::SpatialReference const& v)
{
    setType(metadata::SpatialReference);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::Bounds<double> >(pdal::Bounds<double> const& v)
{
    setType(metadata::Bounds);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint8_t>(boost::uint8_t const& v)
{
    setType(metadata::UnsignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint16_t>(boost::uint16_t const& v)
{
    setType(metadata::UnsignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint32_t>(boost::uint32_t const& v)
{
    setType(metadata::UnsignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint64_t>(boost::uint64_t const& v)
{
    setType(metadata::UnsignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int8_t>(boost::int8_t const& v)
{
    setType(metadata::SignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int16_t>(boost::int16_t const& v)
{
    setType(metadata::SignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int32_t>(boost::int32_t const& v)
{
    setType(metadata::SignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int64_t>(boost::int64_t const& v)
{
    setType(metadata::SignedInteger);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uuids::uuid>(boost::uuids::uuid const& v)
{
    setType(metadata::UUID);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::Metadata>(pdal::Metadata const& v)
{
    setType(metadata::MData);
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::blank>(boost::blank const& v)
{
    setType(metadata::Blank);
    m_tree.put("value",v);
}

inline void Metadata::addMetadata(  Metadata const& m)
{
    
    std::string n = boost::algorithm::ireplace_all_copy(m.getName(), ".", "_");
    m_tree.add_child("entries."+n, m.toPTree());
}

template <typename T>
inline void Metadata::addMetadata(  std::string const& name, 
                                    T const& value, 
                                    std::string const& description)
{
    Metadata m(name, value, description);
    addMetadata(m);
}


} // namespace pdal

namespace std
{

///
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& output);
extern PDAL_DLL std::istream& operator>>(std::istream& istr, pdal::ByteArray& output);
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& m);


// extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& metadata);
}

#endif
