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
#include <boost/uuid/uuid_io.hpp>

#include <boost/optional.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/functional/hash.hpp>



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
        return;
    }

    /// Copy constructor
    ByteArray(const ByteArray& rhs)
        : m_bytes(rhs.m_bytes)
    {
        return;
    }

    /// Destructor
    ~ByteArray()
    {
        return;
    }

    /// Assignment operator
    ByteArray& operator=(ByteArray const& rhs)
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
        /// boolean
        Boolean,
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
        UUID
    };


    typedef boost::variant< 
                            bool,
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
                            boost::uuids::uuid,
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

    ~Metadata()
    {
        return;
    }

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
    template <class T> inline T getValue() const { return boost::get<T>(m_variant); }
    
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

/** @name Parent/child relationships
*/
    /// denotes the parent relationship of this instance to another 
    /// with a given metadata::id
    /// @param id the metadata::id of the parent dimension to this instance
    inline void setParent( metadata::id const& id)
    { 
        m_parentDimensionID = id;
    }
    
    /// @return the metadata::id of the parent Metadata entry to this one.
    inline metadata::id const& getParent( ) const
    {
        return m_parentDimensionID;
    }


/** @name private attributes
*/
private:
    metadata::Variant m_variant;
    std::string m_name;
    std::string m_namespace;
    metadata::Type m_type;
    metadata::MetadataAttributeM m_attributes;
    metadata::id m_uuid;
    metadata::id m_parentDimensionID;
    
};


extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Metadata& srs);


template <>
inline void Metadata::setValue<bool>(bool const& v)
{
    m_variant = v;
    m_type = metadata::Boolean;
}

template <>
inline void Metadata::setValue<std::string>(std::string const& v)
{
    m_variant = v;
    m_type = metadata::String;
}

template <>
inline void Metadata::setValue<pdal::ByteArray>(pdal::ByteArray const& v)
{
    m_variant = v;
    m_type = metadata::Bytes;
}

template <>
inline void Metadata::setValue<float>(float const& v)
{
    m_variant = v;
    m_type = metadata::Float;
}

template <>
inline void Metadata::setValue<double>(double const& v)
{
    m_variant = v;
    m_type = metadata::Double;
}

template <>
inline void Metadata::setValue<pdal::SpatialReference>(pdal::SpatialReference const& v)
{
    m_variant = v;
    m_type = metadata::SpatialReference;
}

template <>
inline void Metadata::setValue<pdal::Bounds<double> >(pdal::Bounds<double> const& v)
{
    m_variant = v;
    m_type = metadata::Bounds;
}

template <>
inline void Metadata::setValue<boost::uint8_t>(boost::uint8_t const& v)
{
    m_variant = v;
    m_type = metadata::UnsignedInteger;
}

template <>
inline void Metadata::setValue<boost::uint16_t>(boost::uint16_t const& v)
{
    m_variant = v;
    m_type = metadata::UnsignedInteger;
}

template <>
inline void Metadata::setValue<boost::uint32_t>(boost::uint32_t const& v)
{
    m_variant = v;
    m_type = metadata::UnsignedInteger;
}

template <>
inline void Metadata::setValue<boost::uint64_t>(boost::uint64_t const& v)
{
    m_variant = v;
    m_type = metadata::UnsignedInteger;
}

template <>
inline void Metadata::setValue<boost::int8_t>(boost::int8_t const& v)
{
    m_variant = v;
    m_type = metadata::SignedInteger;
}

template <>
inline void Metadata::setValue<boost::int16_t>(boost::int16_t const& v)
{
    m_variant = v;
    m_type = metadata::SignedInteger;
}

template <>
inline void Metadata::setValue<boost::int32_t>(boost::int32_t const& v)
{
    m_variant = v;
    m_type = metadata::SignedInteger;
}

template <>
inline void Metadata::setValue<boost::int64_t>(boost::int64_t const& v)
{
    m_variant = v;
    m_type = metadata::SignedInteger;
}

template <>
inline void Metadata::setValue<boost::uuids::uuid>(boost::uuids::uuid const& v)
{
    m_variant = v;
    m_type = metadata::UUID;
}

namespace metadata {


    struct name{};
    struct index{};
    struct uid{};

    typedef boost::multi_index::multi_index_container<
      Metadata,
      boost::multi_index::indexed_by<

        boost::multi_index::random_access<boost::multi_index::tag<index> >,
        // sort by less<string> on GetName
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<name>, boost::multi_index::const_mem_fun<Metadata,std::string const&,&Metadata::getName> >,
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<uid>, boost::multi_index::const_mem_fun<Metadata,metadata::id const&,&Metadata::getUUID> >
          >
    > MetadataMap;

    typedef MetadataMap::index<name>::type index_by_name;
    typedef MetadataMap::index<index>::type index_by_index;
    typedef MetadataMap::index<uid>::type index_by_uid;

    
}

/// Metadatas is a container for Metadata entries. 
class PDAL_DLL Metadatas
{
public:

/** @name Constructors
*/  

    Metadatas() {};

    /// Copy constructor
    Metadatas(const Metadatas&);
    Metadatas& operator=(const Metadatas&); 

    ~Metadatas()
    {
        return;
    }

/** @name entry type
*/  

    /// add a Metadata entry to the PointBuffer's metadata map
    void addMetadata(pdal::Metadata const& entry);

    /*! add a new value T metadata for the given Metadata key and namespace. 
        \param name Metadata entry key to use
        \param ns namespace to use for Metadata entry.
        \param value the T value to set.
    */   
    template<class T> void addMetadata(std::string const& name, T value, std::string const& ns="");

    
    /// @return a const& to a Metadata entry with the given name and/or namespace
    /// If none is found, pdal::metadata_not_found is thrown.
    /// @param name name to use when searching
    /// @param ns to use when searching for metadata entry
    Metadata const& getMetadata(std::string const& name, std::string const& ns="") const;
    
    /// @return a const& to Metadata entry with given metadata::id. 
    /// If none is found, pdal::metadata_not_found is thrown.
    /// @param v metadata::id to search for.
    Metadata const& getMetadata(metadata::id const& v) const;

    /// @return a const& to Metadata with the given index. If the 
    /// index is out of range, pdal::metadata_not_found is thrown.
    /// @param index position index to return.
    Metadata const& getMetadata(std::size_t index) const;

    /// @return the number of Metadata entries in the map
    inline metadata::MetadataMap::size_type getMetadataCount() const { return m_metadata.get<metadata::index>().size(); }

    /// @return a MetadataMap copy to use for setting the Metadata on another 
    /// PointBuffer with setMetadata()
    /// @param index position index to return.
    inline metadata::MetadataMap const& getMetadata() const { return m_metadata; }

    /// @return a boost::optional-wrapped const& to a Metadata with the given name 
    /// and namespace. If no matching metadata entry is found, the optional will be empty.
    /// @param name name to use when searching
    /// @param ns namespace to use when searching. If none is given, the first 
    /// matching Metadata instance with name \b name is returned.
    boost::optional<Metadata const&> getMetadataOptional(std::string const& name, std::string const& ns="") const;

    /// @return a boost::optional-wrapped const& to a Metadata with the given metadata::id.
    /// If no matching dimension is found, the optional will be empty.
    /// @param id id to use when searching
    boost::optional<Metadata const&> getMetadataOptional(metadata::id const& id) const;
    
    /// @return a boost::optional-wrapped const& to a Metadata with the given
    /// index. If the index is out of range, the optional will be empty.
    /// @param index position index to return.
    boost::optional<Metadata const&> getMetadataOptional(std::size_t index) const;

    /*! overwrites an existing Metadata with the same name as m
        \param m the Metadata instance that contains the name and namespace 
        to overwrite in the PointBuffer.
        \verbatim embed:rst 
        .. note::
                
            If no namespace is given, the *first* metadata entry with a matching 
            :cpp:func:`pdal::Metadata::getName()` will be overwritten. To be 
            sure, have set the namespace of the pdal::Metadata using
            :cpp:func:`pdal::Metadata::setNamespace()` beforehand.

        \endverbatim
    */    
    bool setMetadata(Metadata const& m);

    /*! reset the value T metadata for the given Metadata key and namespace. 
        \param name Metadata entry key to use
        \param ns namespace to use for Metadata entry.
        \param value the T value to set.
    */   
    template<class T> void setMetadata(std::string const& name, T value, std::string const& ns="");

    
    /// sets the MetadataMap for the PointBuffer
    /// @param v MetadataMap instance to use (typically from another PointBuffer)
    void setMetadata(metadata::MetadataMap const& v) { m_metadata = v; }    



/** @name private attributes
*/
private:

    metadata::MetadataMap m_metadata;    
    
};

template <class T>
inline void Metadatas::addMetadata(std::string const& name, T value, std::string const& ns)
{
    Metadata m(name, ns);
    m.setValue<T>(value);
    addMetadata(m);
    return;
}

template <class T>
inline void Metadatas::setMetadata(std::string const& name, T value, std::string const& ns)
{
    Metadata m(name, ns);
    m.setValue<T>(value);
    setMetadata(m);
    return;
}


} // namespace pdal

namespace std
{

///
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& output);
}

#endif
