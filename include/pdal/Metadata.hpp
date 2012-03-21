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
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

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
    ByteArray(std::vector<boost::uint8_t> const& data) : m_bytes(data) {}

/** @name Data manipulation
*/     
    /// resets the array 
    inline void set(std::vector<boost::uint8_t> const& input) { m_bytes = input; }
    
    /// fetches a reference to the array
    inline std::vector<boost::uint8_t> const& get() const { return m_bytes; }

/** @name private attributes
*/     
private:
    
    std::vector<boost::uint8_t> m_bytes;
};

namespace metadata {
    
    typedef std::map<std::string, std::string> MetadataAttributeM;
    typedef boost::uuids::uuid id;
    
    enum Type
    {
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
        SpatialReference 
    };


    typedef boost::variant< 
                            float,
                            double,
                            boost::int8_t,
                            boost::uint8_t,
                            boost::int16_t,
                            boost::uint16_t,
                            boost::int32_t,
                            boost::uint32_t,
                            boost::int64_t,
                            boost::uint64_t,
                            std::string, 
                            pdal::ByteArray, 
                            pdal::SpatialReference, 
                            pdal::Bounds<double> > Variant;
} // metadata


/// Metadata is a container for metadata entries that pdal::Stage and pdal::PointBuffer
/// carry around as part of their internal operations. Bits of information might 
/// come from a pdal::Reader that opens a file, or a pdal::Filter that processes 
/// as an intermediate stage, and pdal::Metadata is what is used to hold and 
/// pass those metadata around.  pdal::Metadata values must be of type pdal::metadata::Variant 
/// and it is required that they are serializeable to std::string.

/// pdal::Metadata instances also carry with them a map of key/value pairs 
/// called attributes that are metadata about the metadata entry. For example, 
/// a LAS VLR might have a name of "classification", a pdal::ByteArray for its data, 
/// and a set of attributes that are "userid":"4321" and "vlrid":"1234". These 
/// other metadata may be useful given the context, and any auxiliary data 
/// about the metadata entry should be provided via attributes. It is up to you 
/// to determine where the line of attribute and new metadata entry exists. 
class PDAL_DLL Metadata 
{
public:

/** @name Constructors
*/  
    /// Base constructor
    /// @param name entry name to use for this metadata entry
    /// @param ns namespace to use for this metadata entry
    Metadata(   std::string const& name, 
                std::string const& ns);

    /// Copy constructor
    Metadata(const Metadata&);

/** @name entry type
*/  
    /// returns the pdal::metadata::Type for the metadata entry
    inline metadata::Type getType() const { return m_type; }
    
    /// sets the pdal::metadata::Type for the metadata entry
    /// @param t pdal::metadata::Type value for the entry
    inline void setType(metadata::Type t) { m_type = t; }

/** @name entry value
*/  
    /// sets the pdal::metadata::Variant value for the entry
    /// @param v value of type pdal::metadata::Variant to set for the entry
    template <class T> inline void setValue(T const& v); 
    
    /// gets the metadata entry value as type T.  Throws boost::bad_cast if 
    /// unable to do so.  Use pdal::metdata::Type to determine which type T to 
    /// request of the metadata entry. Alternatively, use pdal::Metdata::cast() 
    /// to attempt to explicitly cast the metadata entry to your own type 
    /// via boost::lexical_cast
    template <class T> inline T getValue() { return boost::get<T>(m_variant); }
    
    /// explicitly casts the metadata entry to your type T via boost::lexical_cast
    template <class T> inline T cast() { return boost::lexical_cast<T>(m_variant); }    

    /// returns the pdal::metadata::Variant instance 
    inline metadata::Variant const& getVariant() const { return m_variant; }

/** @name entry name
*/  
    /// returns the name for the metadata entry
    inline std::string const& getName() const { return m_name; }
    
    /// resets the name for the metadata entry
    /// @param name value to use for new name
    inline void setName(std::string const& name) { m_name = name; }

/** @name entry name
*/  
    /// returns the name for the metadata entry
    inline metadata::id const& getUUID() const { return m_uuid; }
    
    /// resets the id for the metadata entry
    /// @param v value to use for new id
    inline void setUUID(metadata::id const& v) { m_uuid = v; }

    /// resets the id for the metadata entry
    /// @param v value to use for new id
    void setUUID(std::string const& v);

    /// creates a random metadata::id for the entry
    void createUUID();
    
/** @name entry namespace
*/
    /// returns the namespace for the metadata entry
    inline std::string const& getNamespace() const { return m_namespace; }

    /// resets the namespace for the metadata entry
    /// @param ns value to use for new namespace
    inline void setNamespace(std::string const& ns) { m_namespace = ns; }
    
/** @name entry attributes
*/
    /// returns the list of attribute keys for the metadata entry
    std::vector<std::string> getAttributeNames() const;
    
    /// adds a new metadata key/value pair to the metadata entry
    /// @param name to use for the attribute pair
    /// @param value to use for the attribute pair
    void addAttribute(std::string const& name, std::string const value);
    
    /// returns the attribute value for a given attribute key
    std::string getAttribute(std::string const& name) const;    

/** @name private attributes
*/
private:
    metadata::Variant m_variant;
    std::string m_name;
    std::string m_namespace;
    metadata::Type m_type;
    metadata::MetadataAttributeM m_attributes;
    metadata::id m_uuid;
    
};


extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Metadata& srs);



template <class T>
inline void Metadata::setValue(T const& v)
{

    m_variant = v;
    
    try 
    {
        boost::get<std::string>(m_variant);
        m_type = metadata::String;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<pdal::ByteArray>(m_variant);
        m_type = metadata::Bytes;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<float>(m_variant);
        m_type = metadata::Float;
        return;
    } catch (boost::bad_get)
    {}
    
    try 
    {
        boost::get<double>(m_variant);
        m_type = metadata::Double;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<pdal::SpatialReference>(m_variant);
        m_type = metadata::SpatialReference;
        return;
    } catch (boost::bad_get)
    {}
    
    try 
    {
        boost::get<pdal::Bounds<double> >(m_variant);
        m_type = metadata::Bounds;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<boost::uint8_t>(m_variant);
        m_type = metadata::UnsignedInteger;
        return;
    } catch (boost::bad_get)
    {}
        
    try 
    {
        boost::get<boost::uint16_t>(m_variant);
        m_type = metadata::UnsignedInteger;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<boost::uint32_t>(m_variant);
        m_type = metadata::UnsignedInteger;
        return;
    } catch (boost::bad_get)
    {}
    
    try 
    {
        boost::get<boost::uint64_t>(m_variant);
        m_type = metadata::UnsignedInteger;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<boost::int8_t>(m_variant);
        m_type = metadata::SignedInteger;
        return;
    } catch (boost::bad_get)
    {}
    
    try 
    {
        boost::get<boost::int16_t>(m_variant);
        m_type = metadata::SignedInteger;
        return;
    } catch (boost::bad_get)
    {}
    
    try 
    {
        boost::get<boost::int32_t>(m_variant);
        m_type = metadata::SignedInteger;
        return;
    } catch (boost::bad_get)
    {}

    try 
    {
        boost::get<boost::int64_t>(m_variant);
        m_type = metadata::SignedInteger;
        return;
    } catch (boost::bad_get)
    {}
    
    
}

} // namespace pdal

namespace std
{

///
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& output);
}

#endif
